/**
 * User Signin Page
 *
 * Handles user authentication with email/password credentials.
 *
 * Features:
 * - Email/password login
 * - Client-side validation
 * - Error handling
 * - JWT token storage in httpOnly cookie
 * - Redirect to homepage on success
 */

import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import { apiRequest, API_ENDPOINTS } from '@site/src/utils/api';
import { useAuth } from '@site/src/components/AuthProvider';
import styles from './signin.module.css';

export default function SigninPage() {
  // Form state
  const [formData, setFormData] = useState({
    email: '',
    password: '',
  });

  // UI state
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');
  const [validationErrors, setValidationErrors] = useState({});

  // Auth context and router
  const { login } = useAuth();
  const history = useHistory();

  /**
   * Handle input changes for all form fields.
   */
  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: value,
    }));
    // Clear validation error for this field
    if (validationErrors[name]) {
      setValidationErrors((prev) => ({
        ...prev,
        [name]: '',
      }));
    }
  };

  /**
   * Client-side form validation.
   *
   * Returns: true if valid, false otherwise (sets validationErrors)
   */
  const validateForm = () => {
    const errors = {};

    // Email validation (basic)
    if (!formData.email.trim()) {
      errors.email = 'Email is required';
    } else if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(formData.email)) {
      errors.email = 'Invalid email format';
    }

    // Password validation
    if (!formData.password) {
      errors.password = 'Password is required';
    }

    setValidationErrors(errors);
    return Object.keys(errors).length === 0;
  };

  /**
   * Handle form submission - authenticate user.
   */
  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');

    // Client-side validation
    if (!validateForm()) {
      return;
    }

    setIsLoading(true);

    try {
      // Send signin request to backend
      const response = await apiRequest(API_ENDPOINTS.AUTH.SIGNIN, {
        method: 'POST',
        body: JSON.stringify(formData),
      });

      if (!response.ok) {
        const errorData = await response.json();

        // Handle specific error cases
        if (response.status === 401) {
          setError('Invalid email or password. Please try again.');
        } else if (response.status === 422) {
          setError(errorData.detail || 'Invalid input data. Please check your information.');
        } else {
          setError(errorData.detail || 'Failed to sign in. Please try again.');
        }
        setIsLoading(false);
        return;
      }

      // Success - user authenticated and JWT token set in httpOnly cookie
      const userData = await response.json();
      console.log('Signin successful:', userData);

      // Update auth context with user data (persists to localStorage)
      login(userData);

      // Redirect to homepage (using router to preserve state)
      history.push('/');

    } catch (err) {
      console.error('Signin error:', err);
      setError('Network error. Please check your connection and try again.');
      setIsLoading(false);
    }
  };

  return (
    <Layout
      title="Sign In"
      description="Sign in to access the Physical AI & Humanoid Robotics chatbot"
    >
      <main className={styles.signinPage}>
        <div className={styles.signinContainer}>
          <h1>Welcome Back</h1>
          <p className={styles.subtitle}>
            Sign in to continue using the Physical AI & Humanoid Robotics chatbot
          </p>

          {/* Error message display */}
          {error && (
            <div className={styles.errorBanner} role="alert">
              {error}
            </div>
          )}

          <form onSubmit={handleSubmit} className={styles.signinForm}>
            {/* Email field */}
            <div className={styles.formGroup}>
              <label htmlFor="email">Email Address *</label>
              <input
                type="email"
                id="email"
                name="email"
                value={formData.email}
                onChange={handleChange}
                className={validationErrors.email ? styles.inputError : ''}
                required
                disabled={isLoading}
                autoComplete="email"
              />
              {validationErrors.email && (
                <span className={styles.errorText}>{validationErrors.email}</span>
              )}
            </div>

            {/* Password field */}
            <div className={styles.formGroup}>
              <label htmlFor="password">Password *</label>
              <input
                type="password"
                id="password"
                name="password"
                value={formData.password}
                onChange={handleChange}
                className={validationErrors.password ? styles.inputError : ''}
                required
                disabled={isLoading}
                autoComplete="current-password"
              />
              {validationErrors.password && (
                <span className={styles.errorText}>{validationErrors.password}</span>
              )}
            </div>

            {/* Submit button */}
            <button
              type="submit"
              className={styles.submitButton}
              disabled={isLoading}
            >
              {isLoading ? 'Signing In...' : 'Sign In'}
            </button>

            {/* Sign up link */}
            <p className={styles.signupLink}>
              Don't have an account?{' '}
              <a href="/signup">Sign Up</a>
            </p>
          </form>
        </div>
      </main>
    </Layout>
  );
}
