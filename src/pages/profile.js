/**
 * User Profile Dashboard
 *
 * Allows authenticated users to:
 * - View their profile information (name, email, joined date)
 * - Edit their name (email is immutable)
 * - Delete their account (with confirmation)
 *
 * Auth Guard: Redirects to /signin if not authenticated
 */

import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import { useAuth } from '@site/src/components/AuthProvider';
import { apiRequest, API_ENDPOINTS } from '@site/src/utils/api';
import styles from './profile.module.css';

export default function ProfilePage() {
  // Auth context
  const { user, isAuthenticated, isLoading, logout } = useAuth();
  const history = useHistory();

  // Component state
  const [isEditing, setIsEditing] = useState(false);
  const [newName, setNewName] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');

  // Auth guard: Redirect to signin if not authenticated
  useEffect(() => {
    if (!isLoading && !isAuthenticated) {
      history.push('/signin');
    }
  }, [isAuthenticated, isLoading, history]);

  // Initialize newName when user data loads
  useEffect(() => {
    if (user && user.name) {
      setNewName(user.name);
    }
  }, [user]);

  /**
   * Handle name update
   */
  const handleUpdateName = async (e) => {
    e.preventDefault();
    setError('');
    setSuccess('');

    // Validation
    if (!newName || !newName.trim()) {
      setError('Name cannot be empty');
      return;
    }

    if (newName.trim().length > 100) {
      setError('Name must be 100 characters or less');
      return;
    }

    // Check if name actually changed
    if (newName.trim() === user.name) {
      setError('No changes detected');
      return;
    }

    setLoading(true);

    try {
      const response = await apiRequest(API_ENDPOINTS.AUTH.UPDATE, {
        method: 'PUT',
        body: JSON.stringify({ name: newName.trim() }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        setError(errorData.detail || 'Failed to update profile');
        setLoading(false);
        return;
      }

      const result = await response.json();
      setSuccess('Profile updated successfully!');
      setIsEditing(false);
      setLoading(false);

      // Update user data in auth context
      // Note: You may want to fetch fresh user data from /api/auth/me
      // For now, we'll just update the name locally
      if (user) {
        user.name = result.data.name;
      }

      // Clear success message after 3 seconds
      setTimeout(() => setSuccess(''), 3000);

    } catch (err) {
      console.error('Update error:', err);
      setError('Network error. Please try again.');
      setLoading(false);
    }
  };

  /**
   * Handle account deletion
   */
  const handleDeleteAccount = async () => {
    // Double confirmation
    const confirmed = window.confirm(
      '⚠️ WARNING: This action is IRREVERSIBLE!\n\n' +
      'Deleting your account will permanently remove:\n' +
      '- Your profile information\n' +
      '- All chat history\n' +
      '- All associated data\n\n' +
      'Are you absolutely sure you want to proceed?'
    );

    if (!confirmed) {
      return;
    }

    // Second confirmation
    const doubleConfirmed = window.confirm(
      'FINAL CONFIRMATION:\n\n' +
      'Type DELETE to confirm account deletion in the next prompt.'
    );

    if (!doubleConfirmed) {
      return;
    }

    const userInput = window.prompt('Type DELETE to confirm:');
    if (userInput !== 'DELETE') {
      alert('Account deletion cancelled.');
      return;
    }

    setLoading(true);
    setError('');

    try {
      const response = await apiRequest(API_ENDPOINTS.AUTH.DELETE_ACCOUNT, {
        method: 'DELETE',
      });

      if (!response.ok) {
        const errorData = await response.json();
        setError(errorData.detail || 'Failed to delete account');
        setLoading(false);
        return;
      }

      const result = await response.json();
      console.log('Account deleted:', result);

      // Logout and redirect to home
      logout();
      alert('Your account has been deleted successfully.');
      history.push('/');

    } catch (err) {
      console.error('Delete account error:', err);
      setError('Network error. Please try again.');
      setLoading(false);
    }
  };

  // Show loading state while checking authentication
  if (isLoading) {
    return (
      <Layout title="Profile" description="User Profile Dashboard">
        <main className={styles.profilePage}>
          <div className={styles.loadingContainer}>
            <p>Loading...</p>
          </div>
        </main>
      </Layout>
    );
  }

  // If not authenticated, show nothing (redirect will happen)
  if (!isAuthenticated || !user) {
    return null;
  }

  // Format joined date
  const joinedDate = user.created_at
    ? new Date(user.created_at).toLocaleDateString('en-US', {
        year: 'numeric',
        month: 'long',
        day: 'numeric',
      })
    : 'Unknown';

  return (
    <Layout
      title="Profile"
      description="Manage your Physical AI & Humanoid Robotics account"
    >
      <main className={styles.profilePage}>
        <div className={styles.profileContainer}>
          <h1>My Profile</h1>

          {/* Error/Success messages */}
          {error && (
            <div className={styles.errorBanner} role="alert">
              {error}
            </div>
          )}
          {success && (
            <div className={styles.successBanner} role="alert">
              {success}
            </div>
          )}

          {/* Profile Information Card */}
          <div className={styles.profileCard}>
            <h2>Profile Information</h2>

            <div className={styles.profileField}>
              <label>Name</label>
              {isEditing ? (
                <input
                  type="text"
                  value={newName}
                  onChange={(e) => setNewName(e.target.value)}
                  maxLength={100}
                  disabled={loading}
                  className={styles.editInput}
                  autoFocus
                />
              ) : (
                <p className={styles.fieldValue}>{user.name}</p>
              )}
            </div>

            <div className={styles.profileField}>
              <label>Email</label>
              <p className={styles.fieldValue}>
                {user.email}
                <span className={styles.immutableLabel}> (Cannot be changed)</span>
              </p>
            </div>

            <div className={styles.profileField}>
              <label>Software Experience</label>
              <p className={styles.fieldValue}>
                {user.software_experience?.charAt(0).toUpperCase() + user.software_experience?.slice(1) || 'N/A'}
              </p>
            </div>

            <div className={styles.profileField}>
              <label>Hardware Experience</label>
              <p className={styles.fieldValue}>
                {user.hardware_experience?.charAt(0).toUpperCase() + user.hardware_experience?.slice(1) || 'N/A'}
              </p>
            </div>

            <div className={styles.profileField}>
              <label>Member Since</label>
              <p className={styles.fieldValue}>{joinedDate}</p>
            </div>

            {/* Action Buttons */}
            <div className={styles.actionButtons}>
              {isEditing ? (
                <>
                  <button
                    onClick={handleUpdateName}
                    className={styles.saveButton}
                    disabled={loading}
                  >
                    {loading ? 'Saving...' : 'Save Changes'}
                  </button>
                  <button
                    onClick={() => {
                      setIsEditing(false);
                      setNewName(user.name);
                      setError('');
                    }}
                    className={styles.cancelButton}
                    disabled={loading}
                  >
                    Cancel
                  </button>
                </>
              ) : (
                <button
                  onClick={() => setIsEditing(true)}
                  className={styles.editButton}
                  disabled={loading}
                >
                  Edit Profile
                </button>
              )}
            </div>
          </div>

          {/* Danger Zone */}
          <div className={styles.dangerZone}>
            <h2>Danger Zone</h2>
            <p className={styles.dangerWarning}>
              Once you delete your account, there is no going back. This action
              cannot be undone.
            </p>
            <button
              onClick={handleDeleteAccount}
              className={styles.deleteButton}
              disabled={loading}
            >
              Delete Account
            </button>
          </div>
        </div>
      </main>
    </Layout>
  );
}
