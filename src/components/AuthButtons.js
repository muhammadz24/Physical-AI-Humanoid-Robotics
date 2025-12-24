/**
 * Authentication Buttons for Navbar
 *
 * Conditionally renders authentication UI based on user state:
 * - Guest: "Login" | "Signup" buttons
 * - Authenticated: "Logout" button (with optional user avatar)
 *
 * Integrates with AuthProvider context for auth state management.
 */

import React from 'react';
import Link from '@docusaurus/Link';
import { useHistory } from '@docusaurus/router';
import { useAuth } from '@site/src/components/AuthProvider';
import { apiRequest, API_ENDPOINTS } from '@site/src/utils/api';
import styles from './AuthButtons.module.css';

export default function AuthButtons() {
  const { user, isAuthenticated, logout } = useAuth();
  const history = useHistory();

  /**
   * Handle logout - clear auth state and redirect to home.
   */
  const handleLogout = async () => {
    try {
      // Call backend signout endpoint to clear httpOnly cookie
      // Note: This endpoint doesn't exist yet, but we'll clear client state anyway
      await apiRequest(API_ENDPOINTS.AUTH.SIGNOUT, {
        method: 'POST',
      });
    } catch (err) {
      console.error('Logout error:', err);
      // Continue with logout even if API call fails
    } finally {
      // Clear auth context (clears localStorage)
      logout();
      // Redirect to home (using router to preserve state)
      history.push('/');
    }
  };

  // Guest user - show Login and Signup buttons
  if (!isAuthenticated) {
    return (
      <div className={styles.authButtons}>
        <Link
          to="/signin"
          className={styles.loginButton}
        >
          Login
        </Link>
        <Link
          to="/signup"
          className={styles.signupButton}
        >
          Sign Up
        </Link>
      </div>
    );
  }

  // Authenticated user - show Logout button
  return (
    <div className={styles.authButtons}>
      {user && (
        <span className={styles.userName}>
          {user.name}
        </span>
      )}
      <button
        onClick={handleLogout}
        className={styles.logoutButton}
      >
        Logout
      </button>
    </div>
  );
}
