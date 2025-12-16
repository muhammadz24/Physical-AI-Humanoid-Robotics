/**
 * Authentication Context Provider
 *
 * Provides authentication state and methods throughout the application.
 * Uses React Context API for global auth state management.
 *
 * Future Features:
 * - User session state (isAuthenticated, user data)
 * - Sign in/out methods
 * - Token refresh handling
 * - Protected route logic
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
  const [isLoading, setIsLoading] = useState(true);

  /**
   * Check authentication status on mount.
   *
   * Future: Validate JWT token and fetch user data from backend.
   */
  useEffect(() => {
    // Basic setup - mark loading as complete
    // Future: Add token validation and user data fetch
    setIsLoading(false);
  }, []);

  /**
   * Login function - set user data after successful authentication.
   *
   * @param {object} userData - User data from authentication response
   */
  const login = (userData) => {
    setUser(userData);
  };

  /**
   * Logout function - clear user data and remove authentication.
   *
   * Future: Call backend signout endpoint and clear httpOnly cookie.
   */
  const logout = () => {
    setUser(null);
    // Future: Clear JWT token by calling /api/auth/signout
  };

  const value = {
    user,
    isAuthenticated: !!user,
    isLoading,
    login,
    logout,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}
