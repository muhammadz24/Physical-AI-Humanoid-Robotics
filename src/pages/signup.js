/**
 * User Signup Page
 *
 * Handles user registration with email/password authentication and
 * experience level data collection (PDF Compliance - bonus points).
 *
 * Features:
 * - Email/password registration
 * - Software experience level selection (Beginner/Intermediate/Pro)
 * - Hardware experience level selection (None/Arduino/ROS/Professional)
 * - Client-side validation
 * - Error handling
 * - Redirect to homepage on success
 */

import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { apiRequest, API_ENDPOINTS } from '@site/src/utils/api';
import styles from './signup.module.css';

export default function SignupPage() {
  // Form state
  const [formData, setFormData] = useState({
    name: '',
    email: '',
    password: '',
    software_experience: 'beginner',
    hardware_experience: 'none',
  });

  // UI state
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');
  const [validationErrors, setValidationErrors] = useState({});

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

    // Name validation
    if (!formData.name.trim()) {
      errors.name = 'Name is required';
    } else if (formData.name.trim().length < 1 || formData.name.trim().length > 100) {
      errors.name = 'Name must be between 1 and 100 characters';
    }

    // Email validation (basic)
    if (!formData.email.trim()) {
      errors.email = 'Email is required';
    } else if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(formData.email)) {
      errors.email = 'Invalid email format';
    }

    // Password validation
    if (!formData.password) {
      errors.password = 'Password is required';
    } else if (formData.password.length < 8) {
      errors.password = 'Password must be at least 8 characters';
    }

    setValidationErrors(errors);
    return Object.keys(errors).length === 0;
  };

  /**
   * Handle form submission - create user account.
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
      // Send signup request to backend
      const response = await apiRequest(API_ENDPOINTS.AUTH.SIGNUP, {
        method: 'POST',
        body: JSON.stringify(formData),
      });

      if (!response.ok) {
        const errorData = await response.json();

        // Handle specific error cases
        if (response.status === 409) {
          setError('Email already registered. Please use a different email or sign in.');
        } else if (response.status === 422) {
          setError(errorData.detail || 'Invalid input data. Please check your information.');
        } else {
          setError(errorData.detail || 'Failed to create account. Please try again.');
        }
        setIsLoading(false);
        return;
      }

      // Success - user created and JWT token set in httpOnly cookie
      const userData = await response.json();
      console.log('Signup successful:', userData);

      // Redirect to homepage
      window.location.href = '/';

    } catch (err) {
      console.error('Signup error:', err);
      setError('Network error. Please check your connection and try again.');
      setIsLoading(false);
    }
  };

  return (
    <Layout
      title="Sign Up"
      description="Create your account to access the Physical AI & Humanoid Robotics chatbot"
    >
      <main className={styles.signupPage}>
        <div className={styles.signupContainer}>
          <h1>Create Account</h1>
          <p className={styles.subtitle}>
            Join to access the Physical AI & Humanoid Robotics textbook chatbot
          </p>

          {/* Error message display */}
          {error && (
            <div className={styles.errorBanner} role="alert">
              {error}
            </div>
          )}

          <form onSubmit={handleSubmit} className={styles.signupForm}>
            {/* Name field */}
            <div className={styles.formGroup}>
              <label htmlFor="name">Full Name *</label>
              <input
                type="text"
                id="name"
                name="name"
                value={formData.name}
                onChange={handleChange}
                className={validationErrors.name ? styles.inputError : ''}
                required
                maxLength={100}
                disabled={isLoading}
              />
              {validationErrors.name && (
                <span className={styles.errorText}>{validationErrors.name}</span>
              )}
            </div>

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
                minLength={8}
                disabled={isLoading}
              />
              {validationErrors.password && (
                <span className={styles.errorText}>{validationErrors.password}</span>
              )}
              <span className={styles.helpText}>Minimum 8 characters</span>
            </div>

            {/* Software Experience dropdown (PDF Compliance) */}
            <div className={styles.formGroup}>
              <label htmlFor="software_experience">Software Experience *</label>
              <select
                id="software_experience"
                name="software_experience"
                value={formData.software_experience}
                onChange={handleChange}
                required
                disabled={isLoading}
              >
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="pro">Pro</option>
              </select>
              <span className={styles.helpText}>
                Your software development experience level
              </span>
            </div>

            {/* Hardware Experience dropdown (PDF Compliance) */}
            <div className={styles.formGroup}>
              <label htmlFor="hardware_experience">Hardware Experience *</label>
              <select
                id="hardware_experience"
                name="hardware_experience"
                value={formData.hardware_experience}
                onChange={handleChange}
                required
                disabled={isLoading}
              >
                <option value="none">None</option>
                <option value="arduino">Arduino</option>
                <option value="ros">ROS</option>
                <option value="professional">Professional</option>
              </select>
              <span className={styles.helpText}>
                Your hardware/robotics experience level
              </span>
            </div>

            {/* Submit button */}
            <button
              type="submit"
              className={styles.submitButton}
              disabled={isLoading}
            >
              {isLoading ? 'Creating Account...' : 'Sign Up'}
            </button>

            {/* Sign in link */}
            <p className={styles.signinLink}>
              Already have an account?{' '}
              <a href="/signin">Sign In</a>
            </p>
          </form>
        </div>
      </main>
    </Layout>
  );
}
