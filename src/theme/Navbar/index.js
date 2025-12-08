import React from 'react';
import Navbar from '@theme-original/Navbar';
import GitHubAuthor from '@site/src/components/GitHubAuthor';

/**
 * Swizzled Navbar Component
 * Extends default Docusaurus Navbar with GitHub Author integration
 * Follows Docusaurus best practices: swizzle pattern for customization
 */
export default function NavbarWrapper(props) {
  return (
    <>
      <Navbar {...props}>
        <div style={{
          display: 'flex',
          alignItems: 'center',
          marginLeft: 'auto',
          marginRight: '16px',
        }}>
          <GitHubAuthor />
        </div>
      </Navbar>
    </>
  );
}
