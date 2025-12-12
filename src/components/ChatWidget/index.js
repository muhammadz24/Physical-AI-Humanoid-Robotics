import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

const ChatWidget = () => {
  // UI State
  const [isOpen, setIsOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);

  // Chat State
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');

  // Selection State
  const [selectedText, setSelectedText] = useState(null);
  const [tooltipVisible, setTooltipVisible] = useState(false);
  const [tooltipPosition, setTooltipPosition] = useState({ x: 0, y: 0 });

  // Refs
  const messageListRef = useRef(null);
  const inputRef = useRef(null);

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

  // Send query to backend API
  const handleSendQuery = async (contextText = null) => {
    const query = inputValue.trim();

    if (!query) {
      return;
    }

    // Add user message to chat
    const userMessage = {
      type: 'user',
      content: query,
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call backend API
      const response = await fetch('http://localhost:8000/api/chat', {
        method: 'POST',
        mode: 'cors',
        credentials: 'include',
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

      // Add bot response to chat
      const botMessage = {
        type: 'bot',
        content: data.answer,
        citations: data.citations || [],
        confidence: data.confidence,
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, botMessage]);

    } catch (error) {
      console.error('Chat API error:', error);

      // Add error message with DEBUG info
      const errorMessage = {
        type: 'bot',
        content: 'DEBUG ERROR: ' + (error.message || JSON.stringify(error)),
        citations: [],
        timestamp: new Date().toISOString()
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

      {/* Chat Modal */}
      {isOpen && (
        <div className={styles.chatModal}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.headerTitle}>
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
              </svg>
              <span>AI Assistant</span>
            </div>
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

          {/* Message List */}
          <div className={styles.messageList} ref={messageListRef}>
            {messages.length === 0 && (
              <div className={styles.emptyState}>
                <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <circle cx="12" cy="12" r="10"></circle>
                  <path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3"></path>
                  <line x1="12" y1="17" x2="12.01" y2="17"></line>
                </svg>
                <p>Ask me anything about the Physical AI & Humanoid Robotics textbook!</p>
              </div>
            )}

            {messages.map((msg, idx) => (
              <div key={idx} className={msg.type === 'user' ? styles.userMessage : styles.botMessage}>
                <div className={styles.messageContent}>
                  {msg.content}
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
    </>
  );
};

export default ChatWidget;
