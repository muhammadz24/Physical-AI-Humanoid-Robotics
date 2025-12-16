/**
 * PersonalizeButton Component
 * Feature: 007-content-personalization
 *
 * Provides a button to personalize chapter content based on user's experience level.
 * Only visible to authenticated users.
 */

import React, { useEffect } from 'react';
import { useAuth } from '@site/src/components/AuthProvider';
import { usePersonalization } from './usePersonalization';
import styles from './PersonalizeButton.module.css';

/**
 * PersonalizeButton Component
 *
 * @param {object} props - Component props
 * @param {string} props.chapterId - Optional chapter identifier for API logging
 * @param {string} props.buttonText - Custom button text (default: "Personalize This Chapter")
 * @returns {JSX.Element|null} PersonalizeButton component or null if not authenticated
 */
export default function PersonalizeButton({ chapterId = null, buttonText = "Personalize This Chapter" }) {
  const { isAuthenticated, isLoading: authLoading } = useAuth();
  const {
    isIdle,
    isLoading,
    isPersonalized,
    isError,
    errorMessage,
    isShowingPersonalized,
    processingTimeMs,
    personalizeContent,
    toggleContent,
    retry,
    cleanup,
  } = usePersonalization();

  // Cleanup on unmount (abort in-flight requests)
  useEffect(() => {
    return () => {
      cleanup();
    };
  }, [cleanup]);

  // Don't render if auth is still loading
  if (authLoading) {
    return null;
  }

  // Don't render if user is not authenticated
  if (!isAuthenticated) {
    return null;
  }

  /**
   * Handle personalize button click
   */
  const handlePersonalize = () => {
    personalizeContent(chapterId);
  };

  /**
   * Handle retry button click
   */
  const handleRetry = () => {
    retry(chapterId);
  };

  return (
    <div className={styles.personalizeContainer}>
      {/* Idle State - Show Personalize Button */}
      {isIdle && (
        <button
          className={styles.personalizeButton}
          onClick={handlePersonalize}
        >
          <span>✨</span>
          <span>{buttonText}</span>
        </button>
      )}

      {/* Loading State - Show Spinner */}
      {isLoading && (
        <div className={styles.loading}>
          <div className={styles.spinner}></div>
          <span className={styles.loadingText}>
            Personalizing content for your experience level...
          </span>
        </div>
      )}

      {/* Personalized State - Show Toggle Buttons */}
      {isPersonalized && (
        <div>
          <div className={styles.toggleButtons}>
            <button
              className={`${styles.toggleButton} ${!isShowingPersonalized ? styles.active : ''}`}
              onClick={toggleContent}
              disabled={!isShowingPersonalized}
            >
              Show Original
            </button>
            <button
              className={`${styles.toggleButton} ${isShowingPersonalized ? styles.active : ''}`}
              onClick={toggleContent}
              disabled={isShowingPersonalized}
            >
              Show Personalized
            </button>
          </div>
          {processingTimeMs && (
            <p style={{ marginTop: '8px', fontSize: '12px', color: 'var(--ifm-color-emphasis-600)' }}>
              Personalized in {(processingTimeMs / 1000).toFixed(1)}s
            </p>
          )}
        </div>
      )}

      {/* Error State - Show Error Message and Retry Button */}
      {isError && (
        <div className={styles.errorMessage}>
          <p><strong>Personalization Failed</strong></p>
          <p>{errorMessage}</p>
          <button
            className={styles.retryButton}
            onClick={handleRetry}
          >
            Retry
          </button>
        </div>
      )}
    </div>
  );
}
