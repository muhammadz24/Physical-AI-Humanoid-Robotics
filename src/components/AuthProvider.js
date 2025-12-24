/**
 * Authentication Context Provider
 *
 * Provides authentication state and methods throughout the application.
 * Uses React Context API for global auth state management.
 *
 * Features:
 * - User session state (isAuthenticated, user data)
 * - localStorage persistence for session recovery
 * - Sign in/out methods
 * - Loading state to prevent auth flicker during hydration
 */

import React, { createContext, useContext, useState, useEffect } from 'react';

/**
 * Authentication Context.
 *
 * Provides:
 * - user: Current user data (null if not authenticated)
 * - isAuthenticated: Boolean indicating auth status
 * - isLoading: Boolean indicating auth check in progress
 * - login: Function to set user after successful authentication
 * - logout: Function to clear user and sign out
 */
const AuthContext = createContext({
  user: null,
  isAuthenticated: false,
  isLoading: true,
  login: () => {},
  logout: () => {},
});

/**
 * Custom hook to access authentication context.
 *
 * @returns {object} Authentication context value
 *
 * @example
 * const { user, isAuthenticated, login, logout } = useAuth();
 */
export function useAuth() {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}

/**
 * AuthProvider component.
 *
 * Wraps the application to provide authentication context.
 * Handles localStorage persistence and session recovery.
 *
 * @param {object} props
 * @param {React.ReactNode} props.children - Child components
 *
 * @example
 * <AuthProvider>
 *   <App />
 * </AuthProvider>
 */
export default function AuthProvider({ children }) {
  const [user, setUser] = useState(null);
  const [isLoading, setIsLoading] = useState(true); // Start as true to prevent flicker

  /**
   * Check authentication status on mount.
   *
   * Reads user data from localStorage if available.
   * This prevents logout button "flash" on page refresh.
   */
  useEffect(() => {
    // Guard: Only run in browser context (not during SSR)
    if (typeof window === 'undefined') {
      return;
    }

    try {
      // Check if user exists in localStorage
      const storedUser = localStorage.getItem('user');

      if (storedUser) {
        // Parse and restore user session
        const userData = JSON.parse(storedUser);
        setUser(userData);
      }
    } catch (error) {
      // Handle JSON parsing errors gracefully
      console.error('Failed to restore user session from localStorage:', error);
      // Clear corrupted data
      localStorage.removeItem('user');
    } finally {
      // Mark loading as complete (whether user found or not)
      setIsLoading(false);
    }
  }, []); // Empty deps - only run once on mount

  /**
   * Login function - set user data after successful authentication.
   *
   * Persists user data to localStorage for session recovery.
   *
   * @param {object} userData - User data from authentication response
   */
  const login = (userData) => {
    // Guard: Only run in browser context
    if (typeof window !== 'undefined') {
      try {
        // Persist user data to localStorage
        localStorage.setItem('user', JSON.stringify(userData));
      } catch (error) {
        console.error('Failed to persist user session to localStorage:', error);
      }
    }

    // Update state (works in both SSR and browser)
    setUser(userData);
  };

  /**
   * Logout function - clear user data and remove authentication.
   *
   * Removes user data from localStorage and clears state.
   */
  const logout = () => {
    // Guard: Only run in browser context
    if (typeof window !== 'undefined') {
      try {
        // Clear user data from localStorage
        localStorage.removeItem('user');
      } catch (error) {
        console.error('Failed to clear user session from localStorage:', error);
      }
    }

    // Clear state (works in both SSR and browser)
    setUser(null);
  };

  const value = {
    user,
    isAuthenticated: !!user,
    isLoading,
    login,
    logout,
  };

  // CRITICAL: Don't render children until we've checked localStorage
  // This prevents the "logout button flash" bug
  if (isLoading) {
    return null; // Or return a loading spinner if desired
  }

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}
