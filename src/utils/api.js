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
  console.log(`[API Utils] Example Full URL: ${API_BASE_URL}${API_PREFIX}/chat`);
  console.log('[API Utils] ================================');
}

/**
 * API endpoint paths.
 *
 * Centralized endpoint definitions for type safety and maintainability.
 * In local development, the /api prefix is handled by the backend directly,
 * while in production it's handled by Vercel rewrites.
 */
const API_PREFIX = isLocalhost ? '' : '/api';

export const API_ENDPOINTS = {
  AUTH: {
    SIGNUP: `${API_PREFIX}/auth/signup`,
    SIGNIN: `${API_PREFIX}/auth/signin`,
    SIGNOUT: `${API_PREFIX}/auth/signout`,
    ME: `${API_PREFIX}/auth/me`,
    UPDATE: `${API_PREFIX}/auth/update`,
    DELETE_ACCOUNT: `${API_PREFIX}/auth/account`,
  },
  CHAT: {
    SEND_MESSAGE: `${API_PREFIX}/chat`,
    HISTORY: `${API_PREFIX}/chat/history`, // GET endpoint for fetching chat history
    DELETE_HISTORY: `${API_PREFIX}/chat/history`, // DELETE endpoint for clearing history
    DELETE_MESSAGE: (messageId) => `${API_PREFIX}/chat/${messageId}`,
  },
  PERSONALIZE: {
    PERSONALIZE_CHAPTER: `${API_PREFIX}/personalize`,
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
 * @param {Function} onUnauthenticated - Optional callback for 401 responses
 * @returns {Promise<Response>} Fetch response
 *
 * @example
 * const response = await apiRequest('/api/auth/signup', {
 *   method: 'POST',
 *   body: JSON.stringify({ email, password })
 * });
 */
export async function apiRequest(endpoint, options = {}, onUnauthenticated = null) {
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

  const response = await fetch(url, config);

  // Handle 401 Unauthorized responses gracefully
  if (response.status === 401) {
    console.warn(`[API Utils] 401 Unauthorized for endpoint: ${endpoint}`);

    // Call the unauthenticated callback if provided
    if (onUnauthenticated && typeof onUnauthenticated === 'function') {
      onUnauthenticated();
    }
  }

  return response;
}

/**
 * Context-aware API request function that automatically handles 401 responses
 * by calling the provided logout function.
 *
 * @param {Function} logoutFn - Function to call when receiving 401 response
 * @returns {Function} API request function with automatic 401 handling
 */
export function createApiRequestWithAuth(logoutFn) {
  return async function(endpoint, options = {}) {
    return apiRequest(endpoint, options, logoutFn);
  };
}
