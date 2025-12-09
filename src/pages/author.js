import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';

/**
 * Book Author Page - Cyber-Professional Dark Mode
 * Fetches GitHub profile data and displays with animations
 * Features: Glassmorphism, Fade-in/Slide-up animations, Cyber aesthetics
 */
export default function AuthorPage() {
  const [author, setAuthor] = useState({
    name: null,
    login: null,
    bio: null,
    avatar_url: null,
    html_url: null,
    public_repos: 0,
    followers: 0,
    location: null,
    blog: null,
    twitter_username: null,
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
          bio: data.bio,
          avatar_url: data.avatar_url,
          html_url: data.html_url,
          public_repos: data.public_repos,
          followers: data.followers,
          location: data.location,
          blog: data.blog,
          twitter_username: data.twitter_username,
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

  return (
    <Layout
      title="Book Author"
      description="Meet the author of Physical AI & Humanoid Robotics textbook">

      {/* Animation Keyframes */}
      <style>{`
        @keyframes fadeInUp {
          from {
            opacity: 0;
            transform: translateY(40px);
          }
          to {
            opacity: 1;
            transform: translateY(0);
          }
        }

        @keyframes pulse {
          0%, 100% {
            box-shadow: 0 0 20px rgba(0, 217, 255, 0.3);
          }
          50% {
            box-shadow: 0 0 40px rgba(0, 217, 255, 0.6);
          }
        }

        .author-container {
          animation: fadeInUp 1s cubic-bezier(0.4, 0, 0.2, 1);
        }

        .author-avatar {
          animation: pulse 3s ease-in-out infinite;
        }

        .stat-box:hover {
          transform: translateY(-4px);
          box-shadow: 0 8px 16px rgba(0, 217, 255, 0.3);
        }
      `}</style>

      {/* Full-Height Container with Cyber Gradient Background */}
      <div style={{
        minHeight: 'calc(100vh - var(--ifm-navbar-height) - 120px)',
        background: 'linear-gradient(135deg, #020617 0%, #0a192f 100%)',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        padding: '4rem 2rem',
      }}>

        {/* Loading State */}
        {author.loading && (
          <div style={{
            fontSize: '1.5rem',
            color: '#00D9FF',
            fontWeight: '300',
            letterSpacing: '2px',
            animation: 'fadeInUp 0.8s ease-out',
          }}>
            Establishing Neural Uplink...
          </div>
        )}

        {/* Error State */}
        {author.error && !author.loading && (
          <div style={{
            fontSize: '1.2rem',
            color: '#FF4757',
            textAlign: 'center',
          }}>
            Failed to establish connection. Please try again later.
          </div>
        )}

        {/* Main Content */}
        {!author.loading && !author.error && (
          <div className="author-container" style={{
            maxWidth: '800px',
            width: '100%',
          }}>

            {/* Glassmorphism Card */}
            <div style={{
              background: 'rgba(255, 255, 255, 0.05)',
              backdropFilter: 'blur(16px)',
              WebkitBackdropFilter: 'blur(16px)',
              border: '1px solid rgba(192, 192, 192, 0.2)',
              borderRadius: '24px',
              padding: '3rem',
              boxShadow: '0 8px 32px rgba(0, 0, 0, 0.3)',
            }}>

              {/* Header Section */}
              <div style={{
                textAlign: 'center',
                marginBottom: '2rem',
              }}>
                {/* Avatar with Cyan Glow */}
                {author.avatar_url && (
                  <img
                    src={author.avatar_url}
                    alt={author.name}
                    className="author-avatar"
                    style={{
                      width: '150px',
                      height: '150px',
                      borderRadius: '50%',
                      border: '3px solid #00D9FF',
                      objectFit: 'cover',
                      marginBottom: '1.5rem',
                    }}
                  />
                )}

                {/* Name */}
                <h1 style={{
                  fontSize: '2.5rem',
                  fontWeight: '700',
                  color: '#E2E8F0',
                  marginBottom: '0.5rem',
                  letterSpacing: '1px',
                }}>
                  {author.name}
                </h1>

                {/* Username */}
                <p style={{
                  fontSize: '1.1rem',
                  color: '#00D9FF',
                  marginBottom: '1rem',
                }}>
                  @{author.login}
                </p>

                {/* Bio */}
                {author.bio && (
                  <p style={{
                    fontSize: '1rem',
                    color: '#94A3B8',
                    lineHeight: '1.6',
                    maxWidth: '600px',
                    margin: '0 auto 1.5rem',
                  }}>
                    {author.bio}
                  </p>
                )}

                {/* Location & Blog */}
                <div style={{
                  display: 'flex',
                  gap: '2rem',
                  justifyContent: 'center',
                  flexWrap: 'wrap',
                  marginBottom: '2rem',
                  fontSize: '0.95rem',
                  color: '#C0C0C0',
                }}>
                  {author.location && (
                    <div>📍 {author.location}</div>
                  )}
                  {author.blog && (
                    <div>
                      🔗 <a href={author.blog} target="_blank" rel="noopener noreferrer" style={{ color: '#00D9FF' }}>
                        {author.blog}
                      </a>
                    </div>
                  )}
                </div>
              </div>

              {/* Stats Grid */}
              <div style={{
                display: 'grid',
                gridTemplateColumns: 'repeat(auto-fit, minmax(200px, 1fr))',
                gap: '1.5rem',
                marginBottom: '2rem',
              }}>
                {/* Repositories */}
                <div className="stat-box" style={{
                  background: 'rgba(0, 217, 255, 0.1)',
                  backdropFilter: 'blur(10px)',
                  border: '1px solid rgba(0, 217, 255, 0.3)',
                  borderRadius: '12px',
                  padding: '1.5rem',
                  textAlign: 'center',
                  transition: 'all 0.3s ease',
                }}>
                  <div style={{
                    fontSize: '2rem',
                    fontWeight: '700',
                    color: '#00D9FF',
                    marginBottom: '0.5rem',
                  }}>
                    {author.public_repos}
                  </div>
                  <div style={{
                    fontSize: '0.9rem',
                    color: '#C0C0C0',
                    textTransform: 'uppercase',
                    letterSpacing: '1px',
                  }}>
                    Public Repos
                  </div>
                </div>

                {/* Followers */}
                <div className="stat-box" style={{
                  background: 'rgba(192, 192, 192, 0.1)',
                  backdropFilter: 'blur(10px)',
                  border: '1px solid rgba(192, 192, 192, 0.3)',
                  borderRadius: '12px',
                  padding: '1.5rem',
                  textAlign: 'center',
                  transition: 'all 0.3s ease',
                }}>
                  <div style={{
                    fontSize: '2rem',
                    fontWeight: '700',
                    color: '#C0C0C0',
                    marginBottom: '0.5rem',
                  }}>
                    {author.followers}
                  </div>
                  <div style={{
                    fontSize: '0.9rem',
                    color: '#C0C0C0',
                    textTransform: 'uppercase',
                    letterSpacing: '1px',
                  }}>
                    Followers
                  </div>
                </div>
              </div>

              {/* GitHub Profile Button */}
              <div style={{ textAlign: 'center' }}>
                <a
                  href={author.html_url}
                  target="_blank"
                  rel="noopener noreferrer"
                  style={{
                    display: 'inline-block',
                    padding: '1rem 2.5rem',
                    background: 'linear-gradient(135deg, #00D9FF, #0097B3)',
                    color: '#000000',
                    fontWeight: '600',
                    fontSize: '1rem',
                    textDecoration: 'none',
                    borderRadius: '12px',
                    border: 'none',
                    cursor: 'pointer',
                    transition: 'all 0.3s ease',
                    boxShadow: '0 4px 12px rgba(0, 217, 255, 0.3)',
                  }}
                  onMouseEnter={(e) => {
                    e.currentTarget.style.transform = 'translateY(-2px)';
                    e.currentTarget.style.boxShadow = '0 8px 24px rgba(0, 217, 255, 0.5)';
                  }}
                  onMouseLeave={(e) => {
                    e.currentTarget.style.transform = 'translateY(0)';
                    e.currentTarget.style.boxShadow = '0 4px 12px rgba(0, 217, 255, 0.3)';
                  }}
                >
                  View GitHub Profile →
                </a>
              </div>

            </div>

            {/* Footer Text */}
            <div style={{
              textAlign: 'center',
              marginTop: '2rem',
              color: '#94A3B8',
              fontSize: '0.9rem',
            }}>
              Author of Physical AI & Humanoid Robotics Textbook
            </div>

          </div>
        )}
      </div>
    </Layout>
  );
}
