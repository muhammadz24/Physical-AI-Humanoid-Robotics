import React, { useEffect, useState } from 'react';

/**
 * GitHubAuthor Component - COMPLETE IMPLEMENTATION
 * Fetches and displays author information from GitHub API
 * Uses useEffect for efficient data fetching per constraints
 * Cyber-Professional Dark Mode styling with clickable link
 */
export default function GitHubAuthor() {
  const [author, setAuthor] = useState({
    name: null,
    login: 'muhammadz24',
    avatar_url: null,
    html_url: 'https://github.com/muhammadz24',
    loading: true,
    error: false,
  });

  useEffect(() => {
    const fetchGitHubUser = async () => {
      try {
        const response = await fetch('https://api.github.com/users/muhammadz24', {
          headers: {
            'Accept': 'application/vnd.github.v3+json',
          },
        });

        if (!response.ok) {
          throw new Error(`GitHub API error: ${response.status}`);
        }

        const data = await response.json();

        setAuthor({
          name: data.name || data.login,
          login: data.login,
          avatar_url: data.avatar_url,
          html_url: data.html_url,
          loading: false,
          error: false,
        });
      } catch (error) {
        console.error('Failed to fetch GitHub user data:', error);
        setAuthor(prev => ({
          ...prev,
          loading: false,
          error: true,
        }));
      }
    };

    fetchGitHubUser();
  }, []);

  // If loading, show minimal loading state
  if (author.loading) {
    return (
      <div style={{
        display: 'flex',
        alignItems: 'center',
        padding: '6px 12px',
        fontSize: '13px',
        color: 'var(--ifm-navbar-link-color)',
      }}>
        <span>Loading...</span>
      </div>
    );
  }

  // If error, return null (minimal fallback)
  if (author.error && !author.avatar_url) {
    return null;
  }

  // Main render: Clickable link with avatar and name
  return (
    <a
      href={author.html_url}
      target="_blank"
      rel="noopener noreferrer"
      className="github-author"
      style={{
        display: 'flex',
        alignItems: 'center',
        gap: '10px',
        padding: '6px 14px',
        borderRadius: '24px',
        textDecoration: 'none',
        background: 'var(--ifm-navbar-background-color, rgba(0, 0, 0, 0.05))',
        border: '1px solid rgba(192, 192, 192, 0.2)',
        transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)',
        cursor: 'pointer',
      }}
    >
      {author.avatar_url && (
        <img
          src={author.avatar_url}
          alt={author.name || author.login}
          style={{
            width: '32px',
            height: '32px',
            borderRadius: '50%',
            border: '2px solid var(--ifm-color-primary, #00D9FF)',
            objectFit: 'cover',
            transition: 'all 0.3s ease',
          }}
        />
      )}
      <span
        style={{
          fontSize: '14px',
          fontWeight: '600',
          color: 'var(--ifm-navbar-link-color)',
          whiteSpace: 'nowrap',
        }}
      >
        {author.name || author.login}
      </span>
    </a>
  );
}
