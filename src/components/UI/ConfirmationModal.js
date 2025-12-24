/**
 * ConfirmationModal Component
 *
 * A modern, reusable confirmation dialog to replace browser-native window.confirm.
 * Features: Dark/Light mode support, keyboard navigation, danger variant, animations.
 *
 * Usage:
 *   const [showModal, setShowModal] = useState(false);
 *
 *   <ConfirmationModal
 *     isOpen={showModal}
 *     title="Confirm Delete"
 *     message="Are you sure you want to delete this item?"
 *     onConfirm={() => { handleDelete(); setShowModal(false); }}
 *     onCancel={() => setShowModal(false)}
 *     confirmText="Delete"
 *     cancelText="Cancel"
 *     isDanger={true}
 *   />
 */

import React, { useEffect } from 'react';
import styles from './modal.module.css';

const ConfirmationModal = ({
  isOpen,
  title,
  message,
  onConfirm,
  onCancel,
  confirmText = 'Confirm',
  cancelText = 'Cancel',
  isDanger = false
}) => {
  // Handle keyboard events (Escape to cancel, Enter to confirm)
  useEffect(() => {
    if (!isOpen) return;

    const handleKeyPress = (e) => {
      if (e.key === 'Escape') {
        onCancel();
      } else if (e.key === 'Enter' && !e.shiftKey) {
        e.preventDefault();
        onConfirm();
      }
    };

    document.addEventListener('keydown', handleKeyPress);
    return () => document.removeEventListener('keydown', handleKeyPress);
  }, [isOpen, onCancel, onConfirm]);

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

  if (!isOpen) return null;

  return (
    <>
      {/* Backdrop - Click to cancel */}
      <div
        className={styles.modalBackdrop}
        onClick={onCancel}
        aria-label="Close modal"
      ></div>

      {/* Modal Container */}
      <div
        className={styles.modalContainer}
        role="dialog"
        aria-modal="true"
        aria-labelledby="modal-title"
        aria-describedby="modal-message"
      >
        {/* Header */}
        <div className={styles.modalHeader}>
          <h2 id="modal-title" className={styles.modalTitle}>
            {title}
          </h2>
          <button
            className={styles.closeButton}
            onClick={onCancel}
            aria-label="Close modal"
            title="Close (Esc)"
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="18" y1="6" x2="6" y2="18"></line>
              <line x1="6" y1="6" x2="18" y2="18"></line>
            </svg>
          </button>
        </div>

        {/* Body */}
        <div className={styles.modalBody}>
          <p id="modal-message" className={styles.modalMessage}>
            {message}
          </p>
        </div>

        {/* Footer */}
        <div className={styles.modalFooter}>
          <button
            className={styles.cancelButton}
            onClick={onCancel}
            aria-label="Cancel action"
            title="Cancel (Esc)"
          >
            {cancelText}
          </button>
          <button
            className={isDanger ? styles.dangerButton : styles.confirmButton}
            onClick={onConfirm}
            aria-label={isDanger ? 'Confirm dangerous action' : 'Confirm action'}
            title="Confirm (Enter)"
          >
            {confirmText}
          </button>
        </div>
      </div>
    </>
  );
};

export default ConfirmationModal;
