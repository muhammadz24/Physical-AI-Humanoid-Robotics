import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';
import { FaRobot, FaBrain, FaCode } from 'react-icons/fa';
import PageLoader from '../components/PageLoader';

function NeuralLoader({ onComplete }) {
  useEffect(() => {
    const timer = setTimeout(() => {
      onComplete();
    }, 3000);

    return () => clearTimeout(timer);
  }, []);

  return <PageLoader />;
}

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning →
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  const features = [
    {
      icon: FaRobot,
      title: '6 Comprehensive Chapters',
      description: 'From Physical AI fundamentals to advanced capstone project implementation',
    },
    {
      icon: FaCode,
      title: 'Interactive Code Examples',
      description: 'All code includes syntax highlighting, line numbers, and copy buttons',
    },
    {
      icon: FaBrain,
      title: 'AI-Native Learning',
      description: 'RAG chatbot for instant answers grounded in textbook content (coming soon)',
    },
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {features.map((feature, idx) => {
            const Icon = feature.icon;
            return (
              <div key={idx} className={clsx('col col--4')}>
                <div className="text--center padding-horiz--md">
                  <Icon style={{ fontSize: '4rem', color: '#00D9FF', marginBottom: '1rem' }} />
                  <h3>{feature.title}</h3>
                  <p>{feature.description}</p>
                </div>
              </div>
            );
          })}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  const [showLoader, setShowLoader] = useState(true);

  const handleLoaderComplete = () => {
    setShowLoader(false);
  };

  return (
    <Layout
      title={`Welcome`}
      description="AI-Native Interactive Textbook for Physical AI and Humanoid Robotics">
      {showLoader && <NeuralLoader onComplete={handleLoaderComplete} />}
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
