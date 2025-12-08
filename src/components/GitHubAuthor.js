import React, { useEffect, useState } from 'react';

/**
 * GitHubAuthor Component
 * Fetches and displays author information from GitHub API
 * Implements functional requirement: Dynamic Navbar with GitHub API integration
 * Uses useEffect for efficient data fetching (as per constraints)
 */
export default function GitHubAuthor() {
  const [author, setAuthor] = useState({
    name: 'Muhammad Zeeshan', // Fallback static text
    avatar_url: null,
    loading: true,
    error: false,
  });

  useEffect(() => {
    // Fetch GitHub user data
    const fetchGitHubUser = async () => {
      try {
        const response = await fetch('https://api.github.com/users/muhammadz24', {
          headers: {
            'Accept': 'application/vnd.github.v3+json',
          },
        });

        if (!response.ok) {
          throw new Error('GitHub API request failed');
        }

        const data = await response.json();

        setAuthor({
          name: data.name || 'Muhammad Zeeshan',
          avatar_url: data.avatar_url,
          loading: false,
          error: false,
        });
      } catch (error) {
        console.error('Failed to fetch GitHub user data:', error);
        // Fallback to static text on API failure
        setAuthor(prev => ({
          ...prev,
          loading: false,
          error: true,
        }));
      }
    };

    fetchGitHubUser();
  }, []); // Empty dependency array - fetch once on mount

  if (author.loading) {
    return (
      <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
        <span style={{ fontSize: '14px', color: 'var(--ifm-navbar-link-color)' }}>
          Loading...
        </span>
      </div>
    );
  }

  return (
    <div
      style={{
        display: 'flex',
        alignItems: 'center',
        gap: '10px',
        padding: '4px 12px',
        borderRadius: '20px',
        background: 'var(--ifm-navbar-background-color, rgba(0, 0, 0, 0.05))',
        transition: 'all 0.3s ease',
      }}
      className="github-author"
    >
      {author.avatar_url && !author.error && (
        <img
          src={author.avatar_url}
          alt={author.name}
          style={{
            width: '32px',
            height: '32px',
            borderRadius: '50%',
            border: '2px solid var(--ifm-color-primary)',
            objectFit: 'cover',
          }}
        />
      )}
      <span
        style={{
          fontSize: '14px',
          fontWeight: '500',
          color: 'var(--ifm-navbar-link-color)',
        }}
      >
        {author.name}
      </span>
    </div>
  );
}
