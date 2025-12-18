/**
 * API Configuration Utility
 *
 * Provides environment-agnostic API URL configuration following Constitutional
 * Principle IX (Zero-Edit Deployment Configuration).
 *
 * Feature: 012-production-stabilization
 * Fixes: Trailing slash bug, zombie localhost, ENV var issues
 *
 * Usage:
 *   import { API_BASE_URL } from '@site/src/utils/api';
 *   fetch(`${API_BASE_URL}/api/auth/signup`, { ... });
 */

/**
 * Detect if running on localhost
 * - window.location.hostname === 'localhost' (dev server)
 * - window.location.hostname === '127.0.0.1' (alternative localhost)
 *
 * CRITICAL: This detection ignores ENV vars entirely in production to bypass
 * trailing slash bugs and ensure relative paths work correctly.
 */
const isLocalhost = typeof window !== 'undefined' &&
  (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1');

/**
 * Base URL for backend API.
 *
 * Environment-driven configuration:
 * - Local Development: http://localhost:8000 (hardcoded for dev)
 * - Production: '' (empty string = relative URLs, same domain)
 *
 * IMPORTANT: In production, we FORCE relative paths by returning empty string.
 * This bypasses ENV var issues (trailing slashes, wrong domains, etc.)
 */
export const API_BASE_URL = isLocalhost ? 'http://localhost:8000' : '';

/**
 * Log API configuration for debugging
 * Safe to log: no secrets, just configuration info
 *
 * This helps diagnose "zombie localhost" issues where production
 * incorrectly tries to hit localhost:8000
 */
if (typeof console !== 'undefined') {
  console.log('[API Utils] ====== API CONFIGURATION ======');
  console.log(`[API Utils] Environment: ${isLocalhost ? 'LOCALHOST' : 'PRODUCTION'}`);
  console.log(`[API Utils] Hostname: ${typeof window !== 'undefined' ? window.location.hostname : 'N/A'}`);
  console.log(`[API Utils] API Base URL: ${API_BASE_URL || '(relative URLs)'}`);
  console.log(`[API Utils] Example Full URL: ${API_BASE_URL}/api/chat`);
  console.log('[API Utils] ================================');
}

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

  // Log the actual URL being called for debugging
  console.log(`[API Utils] Making request to: ${url}`);

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
