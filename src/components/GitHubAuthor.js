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
    login: null,
    avatar_url: null,
    html_url: null,
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

        // VALIDATION: Check if response.ok
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
        // Set error state - will return null below
        setAuthor(prev => ({
          ...prev,
          loading: false,
          error: true,
        }));
      }
    };

    fetchGitHubUser();
  }, []); // Empty dependency array - fetch once on mount

  // Loading state - show minimal indicator
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

  // ERROR FALLBACK: Return null if API fails (do not render broken text)
  if (author.error || !author.avatar_url || !author.html_url) {
    return null;
  }

  // MAIN RENDER: Clickable <a> tag linking to GitHub profile
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
        background: 'rgba(10, 25, 47, 0.6)',
        border: '1px solid rgba(0, 217, 255, 0.3)',
        backdropFilter: 'blur(10px)',
        WebkitBackdropFilter: 'blur(10px)',
        transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)',
        cursor: 'pointer',
      }}
    >
      {/* AVATAR: 32px Circular image with Cyan border */}
      <img
        src={author.avatar_url}
        alt={author.name || author.login}
        style={{
          width: '32px',
          height: '32px',
          borderRadius: '50%',
          border: '2px solid #00D9FF',
          objectFit: 'cover',
          transition: 'all 0.3s ease',
        }}
      />
      {/* TEXT: Name in white/silver, font-weight 600 */}
      <span
        style={{
          fontSize: '14px',
          fontWeight: '600',
          color: '#E8E8E8',
          whiteSpace: 'nowrap',
        }}
      >
        {author.name || author.login}
      </span>
    </a>
  );
}
