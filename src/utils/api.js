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
 * Helper function to detect if running on localhost
 * - Uses window.location.hostname === 'localhost' (dev server)
 * - Uses window.location.hostname === '127.0.0.1' (alternative localhost)
 *
 * CRITICAL: This detection is SSR-safe by checking typeof window first.
 * Returns false on the server (SSR) and proper value on the client.
 */
function getIsLocalhost() {
  if (typeof window === 'undefined') {
    // Server-side rendering: always return false
    return false;
  }
  return (
    window.location.hostname === 'localhost' ||
    window.location.hostname === '127.0.0.1'
  );
}

/**
 * Base URL for backend API.
 * Uses a getter function to be SSR-safe.
 *
 * Environment-driven configuration:
 * - Local Development: http://localhost:8000 (hardcoded for dev)
 * - Production: '' (empty string = relative URLs, same domain)
 *
 * IMPORTANT: In production, we FORCE relative paths by returning empty string.
 * This bypasses ENV var issues (trailing slashes, wrong domains, etc.)
 */
function getApiBaseUrl() {
  return getIsLocalhost() ? 'http://localhost:8000' : '';
}

/**
 * API prefix for endpoints.
 * Uses a getter function to be SSR-safe.
 * In local development, the /api prefix is handled by the backend directly,
 * while in production it's handled by Vercel rewrites.
 */
function getApiPrefix() {
  return getIsLocalhost() ? '' : '/api';
}

/**
 * Log API configuration for debugging
 * Safe to log: no secrets, just configuration info
 *
 * This helps diagnose "zombie localhost" issues where production
 * incorrectly tries to hit localhost:8000
 */
if (typeof window !== 'undefined' && typeof console !== 'undefined') {
  // Only log if we're in the browser
  const hostname = window.location.hostname;
  console.log('[API Utils] ====== API CONFIGURATION ======');
  console.log(`[API Utils] Environment: ${getIsLocalhost() ? 'LOCALHOST' : 'PRODUCTION'}`);
  console.log(`[API Utils] Hostname: ${hostname}`);
  console.log(`[API Utils] API Base URL: ${getApiBaseUrl() || '(relative URLs)'}`);
  console.log(`[API Utils] Example Full URL: ${getApiBaseUrl()}${getApiPrefix()}/chat`);
  console.log('[API Utils] ================================');
}

/**
 * SSR-safe API_BASE_URL export
 * Uses conditional check to avoid window access during SSR
 */
export const API_BASE_URL = typeof window !== 'undefined' ? getApiBaseUrl() : '';

/**
 * API endpoint paths.
 * These are defined as static strings that get calculated at runtime
 * to be SSR-safe. The values will be determined when accessed in the browser.
 * In local development, the /api prefix is handled by the backend directly,
 * while in production it's handled by Vercel rewrites.
 */
const getAuthEndpoints = () => ({
  SIGNUP: `${getApiPrefix()}/auth/signup`,
  SIGNIN: `${getApiPrefix()}/auth/signin`,
  SIGNOUT: `${getApiPrefix()}/auth/signout`,
  ME: `${getApiPrefix()}/auth/me`,
  UPDATE: `${getApiPrefix()}/auth/update`,
  DELETE_ACCOUNT: `${getApiPrefix()}/auth/account`,
});

const getChatEndpoints = () => ({
  SEND_MESSAGE: `${getApiPrefix()}/chat`,
  HISTORY: `${getApiPrefix()}/chat/history`, // GET endpoint for fetching chat history
  DELETE_HISTORY: `${getApiPrefix()}/chat/history`, // DELETE endpoint for clearing history
  DELETE_MESSAGE: (messageId) => `${getApiPrefix()}/chat/${messageId}`,
});

const getPersonalizeEndpoints = () => ({
  PERSONALIZE_CHAPTER: `${getApiPrefix()}/personalize`,
});

// Export endpoints as functions that return the correct values when called
export const API_ENDPOINTS = {
  get AUTH() {
    return getAuthEndpoints();
  },
  get CHAT() {
    return getChatEndpoints();
  },
  get PERSONALIZE() {
    return getPersonalizeEndpoints();
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
