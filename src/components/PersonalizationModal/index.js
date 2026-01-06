/**
 * PersonalizationModal Component
 *
 * Modal form for users to update their experience levels.
 * Used by RAG chatbot to personalize responses.
 *
 * Features:
 * - 2-question form (software + hardware experience)
 * - Auto-saves to backend on submit
 * - Mobile-responsive button groups
 * - Keyboard navigation (Esc to close)
 *
 * Agent Approvals:
 * - hackathon-spec-guard: Architecture validated
 * - robotics-expert: Labels refined for clarity
 */

import React, { useState, useEffect } from 'react';
import { useAuth } from '@site/src/components/AuthProvider';
import { apiRequest, API_ENDPOINTS } from '@site/src/utils/api';
import styles from './PersonalizationModal.module.css';

const PersonalizationModal = ({ isOpen, onClose }) => {
  const { user, login } = useAuth();

  // Form state
  const [softwareExperience, setSoftwareExperience] = useState('');
  const [hardwareExperience, setHardwareExperience] = useState('');
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState(null);

  // Load current user preferences when modal opens
  useEffect(() => {
    if (isOpen && user) {
      setSoftwareExperience(user.software_experience || '');
      setHardwareExperience(user.hardware_experience || '');
      setError(null); // Clear any previous errors
    }
  }, [isOpen, user]);

  // Handle Escape key to close modal
  useEffect(() => {
    if (!isOpen) return;

    const handleKeyPress = (e) => {
      if (e.key === 'Escape') {
        onClose();
      }
    };

    document.addEventListener('keydown', handleKeyPress);
    return () => document.removeEventListener('keydown', handleKeyPress);
  }, [isOpen, onClose]);

  // Prevent body scroll when modal is open
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = 'unset';
    }

    return () => {
      document.body.style.overflow = 'unset';
    };
  }, [isOpen]);

  /**
   * Handle form submission - update user profile
   */
  const handleSubmit = async (e) => {
    e.preventDefault();
    setError(null);

    // Validation
    if (!softwareExperience || !hardwareExperience) {
      setError('Please answer both questions');
      return;
    }

    setIsSubmitting(true);

    try {
      const response = await apiRequest(API_ENDPOINTS.AUTH.UPDATE, {
        method: 'PUT',
        body: JSON.stringify({
          software_experience: softwareExperience,
          hardware_experience: hardwareExperience,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || 'Failed to update profile');
      }

      const updatedUser = await response.json();

      // FIX: Merge is_authenticated flag (spec-guard recommendation)
      // Backend may not return is_authenticated, so we preserve it from current context
      login({
        ...updatedUser,
        is_authenticated: true,
        name: user.name, // Preserve existing name
        id: user.id      // Preserve existing id
      });

      // Close modal on success
      onClose();
    } catch (err) {
      console.error('Personalization error:', err);
      setError(err.message || 'Failed to save preferences. Please try again.');
    } finally {
      setIsSubmitting(false);
    }
  };

  if (!isOpen) return null;

  return (
    <>
      {/* Backdrop */}
      <div
        className={styles.modalBackdrop}
        onClick={onClose}
        aria-label="Close modal"
      />

      {/* Modal Container */}
      <div
        className={styles.modalContainer}
        role="dialog"
        aria-modal="true"
        aria-labelledby="personalize-title"
      >
        {/* Header */}
        <div className={styles.modalHeader}>
          <h2 id="personalize-title" className={styles.modalTitle}>
            ✨ Personalize Your Learning
          </h2>
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close modal"
            title="Close (Esc)"
            disabled={isSubmitting}
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="18" y1="6" x2="6" y2="18"></line>
              <line x1="6" y1="6" x2="18" y2="18"></line>
            </svg>
          </button>
        </div>

        {/* Body */}
        <form onSubmit={handleSubmit} className={styles.modalBody}>
          <p className={styles.description}>
            Help us tailor the chatbot's explanations to your background
          </p>

          {/* Question 1: Software Experience */}
          <div className={styles.questionGroup}>
            <label className={styles.questionLabel}>
              What's your programming experience?
            </label>
            <div className={styles.optionGrid}>
              <button
                type="button"
                className={`${styles.optionButton} ${softwareExperience === 'beginner' ? styles.selected : ''}`}
                onClick={() => setSoftwareExperience('beginner')}
                disabled={isSubmitting}
              >
                <span className={styles.optionTitle}>Beginner</span>
                <span className={styles.optionDesc}>Basic Python/C++</span>
              </button>
              <button
                type="button"
                className={`${styles.optionButton} ${softwareExperience === 'intermediate' ? styles.selected : ''}`}
                onClick={() => setSoftwareExperience('intermediate')}
                disabled={isSubmitting}
              >
                <span className={styles.optionTitle}>Intermediate</span>
                <span className={styles.optionDesc}>Built apps, know OOP</span>
              </button>
              <button
                type="button"
                className={`${styles.optionButton} ${softwareExperience === 'pro' ? styles.selected : ''}`}
                onClick={() => setSoftwareExperience('pro')}
                disabled={isSubmitting}
              >
                <span className={styles.optionTitle}>Pro</span>
                <span className={styles.optionDesc}>Production systems</span>
              </button>
            </div>
          </div>

          {/* Question 2: Hardware Experience */}
          <div className={styles.questionGroup}>
            <label className={styles.questionLabel}>
              What's your robotics hardware background?
            </label>
            <div className={styles.optionGrid}>
              <button
                type="button"
                className={`${styles.optionButton} ${hardwareExperience === 'none' ? styles.selected : ''}`}
                onClick={() => setHardwareExperience('none')}
                disabled={isSubmitting}
              >
                <span className={styles.optionTitle}>None</span>
                <span className={styles.optionDesc}>Software only</span>
              </button>
              <button
                type="button"
                className={`${styles.optionButton} ${hardwareExperience === 'arduino' ? styles.selected : ''}`}
                onClick={() => setHardwareExperience('arduino')}
                disabled={isSubmitting}
              >
                <span className={styles.optionTitle}>Hobbyist</span>
                <span className={styles.optionDesc}>DIY robots (Arduino, RPi)</span>
              </button>
              <button
                type="button"
                className={`${styles.optionButton} ${hardwareExperience === 'ros' ? styles.selected : ''}`}
                onClick={() => setHardwareExperience('ros')}
                disabled={isSubmitting}
              >
                <span className={styles.optionTitle}>ROS Practitioner</span>
                <span className={styles.optionDesc}>Built ROS2 projects</span>
              </button>
              <button
                type="button"
                className={`${styles.optionButton} ${hardwareExperience === 'professional' ? styles.selected : ''}`}
                onClick={() => setHardwareExperience('professional')}
                disabled={isSubmitting}
              >
                <span className={styles.optionTitle}>Professional</span>
                <span className={styles.optionDesc}>Industry/research</span>
              </button>
            </div>
          </div>

          {/* Error Message */}
          {error && (
            <div className={styles.errorMessage}>
              {error}
            </div>
          )}

          {/* Footer Buttons */}
          <div className={styles.modalFooter}>
            <button
              type="button"
              className={styles.cancelButton}
              onClick={onClose}
              disabled={isSubmitting}
            >
              Cancel
            </button>
            <button
              type="submit"
              className={styles.submitButton}
              disabled={isSubmitting || !softwareExperience || !hardwareExperience}
            >
              {isSubmitting ? 'Saving...' : 'Save Preferences'}
            </button>
          </div>
        </form>
      </div>
    </>
  );
};

export default PersonalizationModal;
