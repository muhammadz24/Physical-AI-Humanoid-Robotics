import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';
import { useAuth } from '@site/src/components/AuthProvider';
import { apiRequest, API_ENDPOINTS, API_BASE_URL } from '@site/src/utils/api';
import ConfirmationModal from '@site/src/components/UI/ConfirmationModal';

const ChatWidget = () => {
  // Auth context
  const { user, isAuthenticated } = useAuth();

  // UI State
  const [isOpen, setIsOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [isLoadingHistory, setIsLoadingHistory] = useState(false);

  // Chat State
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');

  // Selection State
  const [selectedText, setSelectedText] = useState(null);
  const [tooltipVisible, setTooltipVisible] = useState(false);
  const [tooltipPosition, setTooltipPosition] = useState({ x: 0, y: 0 });

  // Modal State
  const [modalConfig, setModalConfig] = useState({
    isOpen: false,
    title: '',
    message: '',
    onConfirm: () => {},
    confirmText: 'Confirm',
    isDanger: false
  });

  // Refs
  const messageListRef = useRef(null);
  const inputRef = useRef(null);

  // Helper: Generate temporary ID for new messages
  const generateTempId = () => {
    // Use crypto.randomUUID if available (modern browsers), otherwise fallback
    if (typeof crypto !== 'undefined' && crypto.randomUUID) {
      return crypto.randomUUID();
    }
    // Fallback: Use timestamp + random number with 'temp_' prefix
    return `temp_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  };

  // Helper: Check if ID is a real database UUID (vs temp ID)
  const isRealDatabaseId = (id) => {
    if (!id) return false;
    // Real UUIDs from database don't have 'temp_' prefix
    if (id.startsWith('temp_')) return false;
    // Real UUIDs match the pattern: xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx
    const uuidPattern = /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/i;
    return uuidPattern.test(id);
  };

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    if (messageListRef.current) {
      messageListRef.current.scrollTop = messageListRef.current.scrollHeight;
    }
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  // HYBRID STORAGE: Load chat history on mount
  useEffect(() => {
    const loadChatHistory = async () => {
      setIsLoadingHistory(true);

      try {
        if (isAuthenticated) {
          // USER: Fetch from database
          const response = await apiRequest(API_ENDPOINTS.CHAT.HISTORY + '?limit=50', {
            method: 'GET',
          });

          if (response.ok) {
            const data = await response.json();

            // Map database format to UI format (with message IDs)
            const loadedMessages = data.data.map(chat => ([
              {
                type: 'user',
                content: chat.query,
                timestamp: chat.created_at,
                id: chat.id  // Store message ID for deletion
              },
              {
                type: 'bot',
                content: chat.response,
                citations: chat.metadata?.citations || [],
                confidence: chat.metadata?.confidence,
                timestamp: chat.created_at,
                id: chat.id  // Store same ID for the pair
              }
            ])).flat();

            setMessages(loadedMessages);
          }
        } else {
          // GUEST: Load from sessionStorage
          const storedHistory = sessionStorage.getItem('chat_history');
          if (storedHistory) {
            try {
              const parsedHistory = JSON.parse(storedHistory);
              setMessages(parsedHistory);
            } catch (e) {
              console.error('Failed to parse chat history from sessionStorage:', e);
            }
          }
        }
      } catch (error) {
        console.error('Failed to load chat history:', error);
      } finally {
        setIsLoadingHistory(false);
      }
    };

    loadChatHistory();
  }, [isAuthenticated]);

  // HYBRID STORAGE: Save guest messages to sessionStorage
  useEffect(() => {
    // Only save for guests (authenticated users auto-save to DB)
    if (!isAuthenticated && messages.length > 0) {
      try {
        sessionStorage.setItem('chat_history', JSON.stringify(messages));
      } catch (e) {
        console.error('Failed to save chat history to sessionStorage:', e);
      }
    }
  }, [messages, isAuthenticated]);

  // Select-to-Ask: Listen for text selection
  useEffect(() => {
    const handleTextSelection = () => {
      const selection = window.getSelection();
      const selectedText = selection.toString().trim();

      // Only show tooltip for meaningful selections (5-1000 chars)
      if (selectedText.length >= 5 && selectedText.length <= 1000) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        setSelectedText(selectedText);
        setTooltipPosition({
          x: rect.right + window.scrollX,
          y: rect.bottom + window.scrollY + 5
        });
        setTooltipVisible(true);
      } else {
        setTooltipVisible(false);
      }
    };

    const handleClickOutside = () => {
      setTooltipVisible(false);
    };

    document.addEventListener('mouseup', handleTextSelection);
    document.addEventListener('touchend', handleTextSelection);
    document.addEventListener('mousedown', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      document.removeEventListener('touchend', handleTextSelection);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  // Handle Ask AI about selection
  const handleAskAboutSelection = () => {
    setIsOpen(true);
    setInputValue(`Explain: ${selectedText}`);
    setTooltipVisible(false);

    // Auto-submit after a brief delay to allow modal to open
    setTimeout(() => {
      handleSendQuery(selectedText);
    }, 100);
  };

  // Delete a single message by ID
  const handleDeleteMessage = (messageId) => {
    if (!messageId) {
      return;
    }

    // Show custom confirmation modal
    setModalConfig({
      isOpen: true,
      title: 'Delete Message',
      message: 'Delete this message? This will remove both the question and answer.',
      confirmText: 'Delete',
      isDanger: true,
      onConfirm: async () => {
        // Close modal first for instant feedback
        setModalConfig(prev => ({ ...prev, isOpen: false }));

        try {
          // Check if this is a real database ID or a temporary ID
          const isRealId = isRealDatabaseId(messageId);

          if (isAuthenticated && isRealId) {
            // USER with real DB ID: Delete from database
            const response = await apiRequest(API_ENDPOINTS.CHAT.DELETE_MESSAGE(messageId), {
              method: 'DELETE',
            });

            if (!response.ok) {
              throw new Error('Failed to delete message from server');
            }

            console.log('Message deleted from database');
          } else {
            // GUEST or temp ID: Just remove from state (sessionStorage auto-saves via useEffect)
            console.log('Message removed from state (temp ID or guest mode)');
          }

          // Remove both user and bot messages with this ID from UI
          setMessages(prev => prev.filter(msg => msg.id !== messageId));

        } catch (error) {
          console.error('Failed to delete message:', error);
          // Show error modal
          setModalConfig({
            isOpen: true,
            title: 'Error',
            message: 'Failed to delete message. Please try again.',
            confirmText: 'OK',
            isDanger: false,
            onConfirm: () => setModalConfig(prev => ({ ...prev, isOpen: false }))
          });
        }
      }
    });
  };

  // Delete all chat history
  const handleDeleteHistory = () => {
    // Show custom confirmation modal
    setModalConfig({
      isOpen: true,
      title: 'Delete All Chat History',
      message: 'Are you sure you want to delete all chat history? This action cannot be undone.',
      confirmText: 'Delete All',
      isDanger: true,
      onConfirm: async () => {
        // Close modal first for instant feedback
        setModalConfig(prev => ({ ...prev, isOpen: false }));

        try {
          if (isAuthenticated) {
            // USER: Delete from database
            const response = await apiRequest(API_ENDPOINTS.CHAT.DELETE_HISTORY, {
              method: 'DELETE',
            });

            if (!response.ok) {
              throw new Error('Failed to delete chat history from server');
            }

            console.log('Chat history deleted from database');
          } else {
            // GUEST: Delete from sessionStorage
            sessionStorage.removeItem('chat_history');
            console.log('Chat history deleted from sessionStorage');
          }

          // Clear UI
          setMessages([]);
        } catch (error) {
          console.error('Failed to delete chat history:', error);
          // Show error modal
          setModalConfig({
            isOpen: true,
            title: 'Error',
            message: 'Failed to delete chat history. Please try again.',
            confirmText: 'OK',
            isDanger: false,
            onConfirm: () => setModalConfig(prev => ({ ...prev, isOpen: false }))
          });
        }
      }
    });
  };

  // Send query to backend API
  const handleSendQuery = async (contextText = null) => {
    const query = inputValue.trim();

    if (!query) {
      return;
    }

    // Generate a temporary ID for this message pair
    const tempId = generateTempId();

    // Add user message to chat with temp ID
    const userMessage = {
      type: 'user',
      content: query,
      timestamp: new Date().toISOString(),
      id: tempId  // Temporary ID for instant deletion capability
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call backend API
      // Note: Backend auto-saves for authenticated users
      const response = await fetch(`${API_BASE_URL}/api/chat`, {
        method: 'POST',
        mode: 'cors',
        credentials: 'include', // Send JWT cookie for auth detection
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: query,
          context: contextText,
          top_k: 5
        })
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data = await response.json();

      // Add bot response to chat with same temp ID (so they can be deleted together)
      const botMessage = {
        type: 'bot',
        content: data.answer,
        citations: data.citations || [],
        confidence: data.confidence,
        timestamp: new Date().toISOString(),
        id: tempId  // Same ID as user message for paired deletion
      };

      setMessages(prev => [...prev, botMessage]);

      // Note: For authenticated users, backend already saved to DB with a real UUID
      // Temp IDs allow instant deletion; on page reload, real UUIDs are loaded
      // For guests, useEffect will save to sessionStorage with temp IDs

    } catch (error) {
      console.error('Chat API error:', error);

      // Add error message with DEBUG info and temp ID
      const errorMessage = {
        type: 'bot',
        content: 'DEBUG ERROR: ' + (error.message || JSON.stringify(error)),
        citations: [],
        timestamp: new Date().toISOString(),
        id: tempId  // Same ID for deletion
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle Enter key in input
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendQuery();
    }
  };

  // Handle citation click
  const handleCitationClick = (url) => {
    window.location.href = url;
    setIsOpen(false);
  };

  return (
    <>
      {/* Floating Button */}
      {!isOpen && (
        <button
          className={styles.floatingButton}
          onClick={() => setIsOpen(true)}
          aria-label="Open chat"
        >
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        </button>
      )}

      {/* Chat Modal with Backdrop */}
      {isOpen && (
        <>
          {/* Backdrop - Click to close */}
          <div
            className={styles.modalBackdrop}
            onClick={() => setIsOpen(false)}
            aria-label="Close chat"
          ></div>

          {/* Chat Window */}
          <div className={styles.chatModal}>
            {/* Header */}
            <div className={styles.chatHeader}>
              <div className={styles.headerTitle}>
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
                </svg>
                <span>AI Assistant</span>
                {isAuthenticated && user && (
                  <span className={styles.userBadge}>({user.name})</span>
                )}
              </div>
              <div className={styles.headerActions}>
                {/* Delete All History Button */}
                {messages.length > 0 && (
                  <button
                    className={styles.deleteAllButton}
                    onClick={handleDeleteHistory}
                    aria-label="Delete all chat history"
                    title="Delete all chat history"
                  >
                    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                      <polyline points="3 6 5 6 21 6"></polyline>
                      <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"></path>
                      <line x1="10" y1="11" x2="10" y2="17"></line>
                      <line x1="14" y1="11" x2="14" y2="17"></line>
                    </svg>
                    <span className={styles.deleteAllText}>Clear All</span>
                  </button>
                )}
                {/* Close Button */}
                <button
                  className={styles.closeButton}
                  onClick={() => setIsOpen(false)}
                  aria-label="Close chat"
                >
                  <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <line x1="18" y1="6" x2="6" y2="18"></line>
                    <line x1="6" y1="6" x2="18" y2="18"></line>
                  </svg>
                </button>
              </div>
            </div>

            {/* Message List */}
            <div className={styles.messageList} ref={messageListRef}>
              {isLoadingHistory && (
                <div className={styles.emptyState}>
                  <p>Loading chat history...</p>
                </div>
              )}

              {!isLoadingHistory && messages.length === 0 && (
                <div className={styles.emptyState}>
                  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <circle cx="12" cy="12" r="10"></circle>
                    <path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3"></path>
                    <line x1="12" y1="17" x2="12.01" y2="17"></line>
                  </svg>
                  <p>Ask me anything about the Physical AI & Humanoid Robotics textbook!</p>
                  {!isAuthenticated && (
                    <p className={styles.guestNote}>
                      <small>Guest mode: History saved in browser session</small>
                    </p>
                  )}
                </div>
              )}

              {messages.map((msg, idx) => (
                <div key={idx} className={msg.type === 'user' ? styles.userMessage : styles.botMessage}>
                  <div className={styles.messageWrapper}>
                    <div className={styles.messageContent}>
                      {msg.content}
                    </div>

                    {/* Delete button for individual user messages (all users with message IDs) */}
                    {msg.type === 'user' && msg.id && (
                      <button
                        className={styles.deleteMessageButton}
                        onClick={() => handleDeleteMessage(msg.id)}
                        aria-label="Delete this message"
                        title="Delete this message"
                      >
                        <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                          <polyline points="3 6 5 6 21 6"></polyline>
                          <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"></path>
                          <line x1="10" y1="11" x2="10" y2="17"></line>
                          <line x1="14" y1="11" x2="14" y2="17"></line>
                        </svg>
                      </button>
                    )}
                  </div>

                  {/* Citations */}
                  {msg.citations && msg.citations.length > 0 && (
                    <div className={styles.citations}>
                      <div className={styles.citationsLabel}>Sources:</div>
                      {msg.citations.map((citation, citIdx) => (
                        <button
                          key={citIdx}
                          className={styles.citationChip}
                          onClick={() => handleCitationClick(citation.url)}
                          title={`${citation.chapter_title} - ${citation.section}`}
                        >
                          <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                            <path d="M4 19.5A2.5 2.5 0 0 1 6.5 17H20"></path>
                            <path d="M6.5 2H20v20H6.5A2.5 2.5 0 0 1 4 19.5v-15A2.5 2.5 0 0 1 6.5 2z"></path>
                          </svg>
                          Chapter {citation.chapter}: {citation.section}
                        </button>
                      ))}
                    </div>
                  )}
                </div>
              ))}

              {/* Loading indicator */}
              {isLoading && (
                <div className={styles.botMessage}>
                  <div className={styles.loadingIndicator}>
                    <span className={styles.dot}></span>
                    <span className={styles.dot}></span>
                    <span className={styles.dot}></span>
                  </div>
                </div>
              )}
            </div>

            {/* Input Box */}
            <div className={styles.inputBox}>
              <textarea
                ref={inputRef}
                className={styles.input}
                placeholder="Ask a question about the textbook..."
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                rows={1}
                disabled={isLoading}
              />
              <button
                className={styles.sendButton}
                onClick={() => handleSendQuery()}
                disabled={!inputValue.trim() || isLoading}
                aria-label="Send message"
              >
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <line x1="22" y1="2" x2="11" y2="13"></line>
                  <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
                </svg>
              </button>
            </div>
          </div>
        </>
      )}

      {/* Select-to-Ask Tooltip */}
      {tooltipVisible && !isOpen && (
        <div
          className={styles.selectTooltip}
          style={{
            left: `${tooltipPosition.x}px`,
            top: `${tooltipPosition.y}px`
          }}
        >
          <button
            className={styles.tooltipButton}
            onClick={handleAskAboutSelection}
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <circle cx="12" cy="12" r="10"></circle>
              <path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3"></path>
              <line x1="12" y1="17" x2="12.01" y2="17"></line>
            </svg>
            Ask AI about this
          </button>
        </div>
      )}

      {/* Confirmation Modal */}
      <ConfirmationModal
        isOpen={modalConfig.isOpen}
        title={modalConfig.title}
        message={modalConfig.message}
        onConfirm={modalConfig.onConfirm}
        onCancel={() => setModalConfig(prev => ({ ...prev, isOpen: false }))}
        confirmText={modalConfig.confirmText}
        cancelText="Cancel"
        isDanger={modalConfig.isDanger}
      />
    </>
  );
};

export default ChatWidget;
