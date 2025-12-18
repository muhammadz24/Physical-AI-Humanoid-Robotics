/**
 * API Configuration
 *
 * Automatically detects localhost vs. production and uses appropriate API URL.
 *
 * Constitution Compliance:
 * - Principle IX: Zero-Edit Deployment (auto-detection, no env vars needed)
 *
 * Feature: 009.1-hotfix-smart-routing
 * Fixes: Production API connection, dynamic environment detection
 */

/**
 * Detect if running on localhost
 * - window.location.hostname === 'localhost' (dev server)
 * - window.location.hostname === '127.0.0.1' (alternative localhost)
 */
const isLocalhost = typeof window !== 'undefined' &&
  (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1');

/**
 * Base URL for backend API
 * - Localhost: 'http://localhost:8000' (separate backend server)
 * - Production/Vercel: '' (relative URLs, same domain as frontend)
 *
 * Examples:
 * - Localhost: API_BASE_URL + '/api/chat' = 'http://localhost:8000/api/chat'
 * - Production: API_BASE_URL + '/api/chat' = '/api/chat' (relative, same origin)
 */
export const API_BASE_URL = isLocalhost ? 'http://localhost:8000' : '';

/**
 * Log API configuration for debugging
 * Safe to log: no secrets, just configuration info
 */
if (typeof console !== 'undefined') {
  console.log(`[API Config] Mode: ${isLocalhost ? 'LOCALHOST' : 'PRODUCTION'}`);
  console.log(`[API Config] API Base: ${API_BASE_URL || '(relative URLs)'}`);
  console.log(`[API Config] Example URL: ${API_BASE_URL}/api/chat`);
}

/**
 * Usage in components:
 *
 * import { API_BASE_URL } from '@site/src/config/api';
 *
 * fetch(`${API_BASE_URL}/api/chat`, {
 *   method: 'POST',
 *   body: JSON.stringify({ message: 'Hello' })
 * });
 */
