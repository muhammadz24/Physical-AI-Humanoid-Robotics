import React from 'react';
import AuthProvider from '@site/src/components/AuthProvider';
import ChatWidget from '@site/src/components/ChatWidget';

// Wrapper component to add global providers and ChatWidget to all pages
export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
      <ChatWidget />
    </AuthProvider>
  );
}
