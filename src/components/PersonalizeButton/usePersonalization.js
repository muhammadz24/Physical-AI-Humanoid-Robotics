/**
 * usePersonalization Hook
 * Feature: 007-content-personalization
 *
 * Manages personalization state, API calls, and content caching.
 */

import { useState, useCallback, useRef } from 'react';
import { API_BASE_URL, API_ENDPOINTS } from '@site/src/utils/api';

/**
 * Personalization states
 */
const STATES = {
  IDLE: 'idle',           // Initial state, button ready
  LOADING: 'loading',     // API request in progress
  PERSONALIZED: 'personalized', // Personalized content available
  ERROR: 'error',         // Error occurred
};

/**
 * Custom hook for content personalization logic.
 *
 * @returns {object} Personalization state and control functions
 */
export function usePersonalization() {
  const [state, setState] = useState(STATES.IDLE);
  const [originalContent, setOriginalContent] = useState(null);
  const [personalizedContent, setPersonalizedContent] = useState(null);
  const [errorMessage, setErrorMessage] = useState(null);
  const [isShowingPersonalized, setIsShowingPersonalized] = useState(false);
  const [processingTimeMs, setProcessingTimeMs] = useState(null);

  // Use ref to track abort controller for request cancellation
  const abortControllerRef = useRef(null);

  /**
   * Extract chapter content from the current page's article element.
   *
   * @param {string} chapterId - Optional chapter identifier
   * @returns {string|null} Extracted chapter text or null if not found
   */
  const extractChapterContent = useCallback((chapterId) => {
    try {
      // Find the main article element (Docusaurus standard structure)
      const article = document.querySelector('article');

      if (!article) {
        console.error('No article element found on page');
        return null;
      }

      // Get the inner HTML text
      const chapterText = article.innerText || article.textContent;

      if (!chapterText || chapterText.trim().length < 100) {
        console.error('Chapter content too short or empty');
        return null;
      }

      return chapterText.trim();
    } catch (error) {
      console.error('Error extracting chapter content:', error);
      return null;
    }
  }, []);

  /**
   * Call POST /api/personalize endpoint to personalize content.
   *
   * @param {string} chapterId - Optional chapter identifier
   * @returns {Promise<void>}
   */
  const personalizeContent = useCallback(async (chapterId = null) => {
    // Cancel any in-flight request
    if (abortControllerRef.current) {
      abortControllerRef.current.abort();
    }

    // Create new abort controller for this request
    abortControllerRef.current = new AbortController();

    // Extract chapter content
    const chapterText = extractChapterContent(chapterId);

    if (!chapterText) {
      setState(STATES.ERROR);
      setErrorMessage('Failed to extract chapter content. Please refresh the page and try again.');
      return;
    }

    // Save original content before personalizing
    if (!originalContent) {
      const article = document.querySelector('article');
      setOriginalContent(article.innerHTML);
    }

    // Set loading state
    setState(STATES.LOADING);
    setErrorMessage(null);

    try {
      const response = await fetch(
        `${API_BASE_URL}${API_ENDPOINTS.PERSONALIZE.PERSONALIZE_CHAPTER}`,
        {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          credentials: 'include', // Send httpOnly cookies
          body: JSON.stringify({
            chapter_text: chapterText,
            chapter_id: chapterId,
          }),
          signal: abortControllerRef.current.signal,
        }
      );

      // Handle HTTP errors
      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        const errorDetail = errorData.detail || `HTTP ${response.status} error`;

        setState(STATES.ERROR);
        setErrorMessage(errorDetail);
        return;
      }

      // Parse successful response
      const data = await response.json();

      // Validate response
      if (!data.personalized_text || data.personalized_text.trim().length === 0) {
        setState(STATES.ERROR);
        setErrorMessage('Received empty personalized content. Please try again.');
        return;
      }

      // Save personalized content
      setPersonalizedContent(data.personalized_text);
      setProcessingTimeMs(data.processing_time_ms);

      // Replace article content with personalized version
      const article = document.querySelector('article');
      if (article) {
        article.innerHTML = `<div class="personalized-content">${data.personalized_text.replace(/\n/g, '<br>')}</div>`;
      }

      // Set state to personalized
      setState(STATES.PERSONALIZED);
      setIsShowingPersonalized(true);

    } catch (error) {
      // Handle abort (user navigated away)
      if (error.name === 'AbortError') {
        console.log('Personalization request aborted');
        return;
      }

      // Handle other errors
      console.error('Personalization error:', error);
      setState(STATES.ERROR);
      setErrorMessage('Failed to personalize content. Please check your internet connection and try again.');
    }
  }, [extractChapterContent, originalContent]);

  /**
   * Toggle between original and personalized content.
   */
  const toggleContent = useCallback(() => {
    const article = document.querySelector('article');
    if (!article) return;

    if (isShowingPersonalized) {
      // Show original
      article.innerHTML = originalContent;
      setIsShowingPersonalized(false);
    } else {
      // Show personalized
      article.innerHTML = `<div class="personalized-content">${personalizedContent.replace(/\n/g, '<br>')}</div>`;
      setIsShowingPersonalized(true);
    }
  }, [isShowingPersonalized, originalContent, personalizedContent]);

  /**
   * Retry personalization after error.
   */
  const retry = useCallback((chapterId) => {
    setState(STATES.IDLE);
    setErrorMessage(null);
    personalizeContent(chapterId);
  }, [personalizeContent]);

  /**
   * Cleanup function to abort in-flight requests.
   */
  const cleanup = useCallback(() => {
    if (abortControllerRef.current) {
      abortControllerRef.current.abort();
    }
  }, []);

  return {
    // State
    state,
    isIdle: state === STATES.IDLE,
    isLoading: state === STATES.LOADING,
    isPersonalized: state === STATES.PERSONALIZED,
    isError: state === STATES.ERROR,
    errorMessage,
    isShowingPersonalized,
    processingTimeMs,

    // Actions
    personalizeContent,
    toggleContent,
    retry,
    cleanup,
  };
}
