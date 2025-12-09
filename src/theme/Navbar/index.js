import React from 'react';
import Navbar from '@theme-original/Navbar';
import GitHubAuthor from '@site/src/components/GitHubAuthor';

/**
 * Swizzled Navbar Component - COMPLETE IMPLEMENTATION
 * Extends default Docusaurus Navbar with GitHub Author integration
 * Properly positions author on the right side using Flexbox
 * Follows Docusaurus best practices: swizzle wrapper pattern
 */
export default function NavbarWrapper(props) {
  return (
    <div style={{ position: 'relative' }}>
      <Navbar {...props} />
      <div
        style={{
          position: 'absolute',
          top: '50%',
          right: '80px',
          transform: 'translateY(-50%)',
          zIndex: 1,
          display: 'flex',
          alignItems: 'center',
        }}
      >
        <GitHubAuthor />
      </div>
    </div>
  );
}
