/**
 * API Configuration Utility
 *
 * Provides environment-agnostic API URL configuration following Constitutional
 * Principle IX (Zero-Edit Deployment Configuration).
 *
 * Usage:
 *   import { API_BASE_URL } from '@site/src/utils/api';
 *   fetch(`${API_BASE_URL}/api/auth/signup`, { ... });
 */

/**
 * Base URL for backend API.
 *
 * Environment-driven configuration:
 * - Local Development: http://localhost:8000 (default)
 * - Production: Set REACT_APP_API_URL in Vercel environment variables
 *
 * IMPORTANT: NO HARDCODING - This allows the same build to work in
 * local development and production without code changes.
 *
 * Safe check for process.env to avoid ReferenceError in browser (Webpack 5/Docusaurus).
 */
let apiUrl = 'http://localhost:8000';
try {
  if (typeof process !== 'undefined' && process.env && process.env.REACT_APP_API_URL) {
    apiUrl = process.env.REACT_APP_API_URL;
  }
} catch (e) {
  // Ignore error, fallback to localhost
}
export const API_BASE_URL = apiUrl;

/**
 * API endpoint paths.
 *
 * Centralized endpoint definitions for type safety and maintainability.
 */
export const API_ENDPOINTS = {
  AUTH: {
    SIGNUP: '/api/auth/signup',
    SIGNIN: '/api/auth/signin',
    SIGNOUT: '/api/auth/signout',
  },
  CHAT: {
    SEND_MESSAGE: '/api/chat',
  },
  PERSONALIZE: {
    PERSONALIZE_CHAPTER: '/api/personalize',
  },
};

/**
 * Default fetch options for API requests.
 *
 * Includes credentials to send httpOnly cookies with requests.
 */
export const DEFAULT_FETCH_OPTIONS = {
  credentials: 'include', // Send cookies for JWT authentication
  headers: {
    'Content-Type': 'application/json',
  },
};

/**
 * Helper function to make authenticated API requests.
 *
 * @param {string} endpoint - API endpoint path (e.g., '/api/auth/signup')
 * @param {object} options - Fetch options (method, body, etc.)
 * @returns {Promise<Response>} Fetch response
 *
 * @example
 * const response = await apiRequest('/api/auth/signup', {
 *   method: 'POST',
 *   body: JSON.stringify({ email, password })
 * });
 */
export async function apiRequest(endpoint, options = {}) {
  const url = `${API_BASE_URL}${endpoint}`;

  const config = {
    ...DEFAULT_FETCH_OPTIONS,
    ...options,
    headers: {
      ...DEFAULT_FETCH_OPTIONS.headers,
      ...(options.headers || {}),
    },
  };

  return fetch(url, config);
}
