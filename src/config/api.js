/**
 * API Configuration
 *
 * Centralized API URL configuration for environment-agnostic deployment.
 *
 * Constitution Compliance:
 * - Principle IX: Zero-Edit Deployment (works in dev and prod automatically)
 *
 * Feature: 009-ultimate-fix (Phase 5 - Smart API Configuration)
 * Success Criteria: FR-007, FR-008, SC-005
 */

/**
 * Base URL for backend API.
 *
 * Environment-driven configuration:
 * - Local Development: http://localhost:8000 (default fallback)
 * - Production: Set REACT_APP_API_URL in Vercel environment variables
 *
 * Note: This project uses Docusaurus which requires REACT_APP_* prefix
 * (not NEXT_PUBLIC_* which is for Next.js)
 *
 * IMPORTANT: NO HARDCODING - This allows the same build to work in
 * local development and production without code changes.
 */
let API_BASE_URL = 'http://localhost:8000';

// Safe check for process.env to avoid ReferenceError in browser
try {
  if (typeof process !== 'undefined' && process.env && process.env.REACT_APP_API_URL) {
    API_BASE_URL = process.env.REACT_APP_API_URL;
  }
} catch (e) {
  // Ignore error, fallback to localhost
  console.warn('[API Config] Failed to read environment variable, using localhost fallback');
}

// Log which API URL is being used (helpful for debugging)
console.log(`[API Config] Using API URL: ${API_BASE_URL}`);
console.log(`[API Config] Environment: ${process.env.NODE_ENV || 'development'}`);

// Export as constant
export { API_BASE_URL };

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
