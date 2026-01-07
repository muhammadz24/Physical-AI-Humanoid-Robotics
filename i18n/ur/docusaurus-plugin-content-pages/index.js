import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from '../../../src/pages/index.module.css';
import { FaRobot, FaBrain, FaCode } from 'react-icons/fa';
import PageLoader from '../../../src/components/PageLoader';

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
        <h1 className="hero__title">{"جسمانی مصنوعی ذہانت اور ہیومنوائڈ روبوٹکس"}</h1>
        <p className="hero__subtitle">{"جسمانی مصنوعی ذہانت، ROS 2، اور ہیومنوائڈ روبوٹکس سیکھیں: بنیادیات سے اعلی درجے تک"}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/intro">
            سیکھنا شروع کریں →
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
      title: '6 جامع ابواب',
      description: 'جسمانی مصنوعی ذہانت کے بنیادیات سے لے کر جدید کیپسٹون پروجیکٹ کے نفاذ تک',
    },
    {
      icon: FaCode,
      title: 'تعاملی کوڈ کی مثالیں',
      description: 'تمام کوڈ میں سینٹیکس ہائی لائٹنگ، لائن نمبرز اور کاپی بٹن شامل ہیں',
    },
    {
      icon: FaBrain,
      title: 'مصنوعی ذہانت کے مطابق سیکھنا',
      description: 'متن کی مثالوں میں مبنی فوری جوابات کے لیے RAG چیٹ بور، جلد ہی آرہا ہے',
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
      title={`خوش آمدید`}
      description="مصنوعی ذہانت کے مطابق تعاملی متن کتاب برائے جسمانی مصنوعی ذہانت اور ہیومنوائڈ روبوٹکس">
      {showLoader && <NeuralLoader onComplete={handleLoaderComplete} />}
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}