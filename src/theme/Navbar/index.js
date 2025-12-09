import React from 'react';
import Navbar from '@theme-original/Navbar';
import GitHubAuthor from '@site/src/components/GitHubAuthor';

/**
 * Swizzled Navbar Component - LAYOUT STABILITY FIX
 * Renders original Navbar untouched to preserve sticky behavior
 * GitHubAuthor uses position: fixed to float independently
 * No wrapper interference with Docusaurus layout
 */
export default function NavbarWrapper(props) {
  return (
    <>
      {/* Original Navbar - NO WRAPPER to preserve sticky behavior */}
      <Navbar {...props} />

      {/* GitHub Author - FIXED positioning, independent of navbar */}
      <div
        style={{
          position: 'fixed',
          top: '0.7rem',
          right: '5rem',
          zIndex: 200,
          pointerEvents: 'none', // Allow clicks to pass through container
        }}
      >
        <div style={{ pointerEvents: 'auto' }}>
          {/* Re-enable pointer events for actual component */}
          <GitHubAuthor />
        </div>
      </div>

      {/* Mobile: Hide on small screens to prevent clutter */}
      <style>{`
        @media (max-width: 768px) {
          .github-author {
            display: none !important;
          }
        }
      `}</style>
    </>
  );
}
