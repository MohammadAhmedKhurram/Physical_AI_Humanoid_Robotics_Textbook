import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './index.module.css';

export default function FinalAcademicLanding() {
  const chapters = [
    'Foundations',
    'Perception & Sensing',
    'Control & Locomotion',
    'System Integration',
    'Simulation & Validation',
    'Implementation Patterns',
    'Evaluation Methodology',
    'Case Studies',
    'Safety & Ethics',
    'Advanced Topics',
  ];

  return (
    <Layout title="Physical AI & Humanoid Robotics" description="A systems‑engineered textbook organized by formal specifications and reproducible validation.">
      <main className={styles.pageRoot}>

        {/* HERO SECTION */}
        <section className={styles.heroSection}>
          <div className={styles.container}>
            <div className={styles.heroInner}>
              <div className={styles.heroContent}>
                <h1 className={styles.heroTitle}>Physical AI &amp; Humanoid Robotics<span className={styles.navEmoji}>🤖</span></h1>
                <p className={styles.heroSubtitle}>A systems‑engineered textbook emphasizing specification, verification, and reproducible artifacts.</p>
                <p className={styles.heroIntro}>This book presents methods and workflows for designing, validating, and reproducing research in physical artificial intelligence and humanoid robotics. Chapters pair formal specifications with executable artifacts so results are auditable.</p>
                <div className={styles.heroActions}>
                  <Link className={styles.buttonPrimary} to="/docs/chapters/chapter-01/concept">Start reading</Link>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* SYSTEM FLOW */}
        <section className={styles.flowSection}>
          <div className={styles.container}>
            <h2 className={styles.sectionHeading}>System flow</h2>
            <p className={styles.curriculumIntro}>A compact overview of the Specs‑Driven Development workflow.</p>
            <div className={styles.flowViz}>
              <svg className={styles.flowSvg} viewBox="0 0 1000 120" preserveAspectRatio="xMidYMid meet" xmlns="http://www.w3.org/2000/svg">
                <defs>
                  <marker id="mArrow" markerWidth="8" markerHeight="8" refX="6" refY="4" orient="auto">
                    <path d="M0 0 L8 4 L0 8 z" fill="rgba(11,17,20,0.06)" />
                  </marker>
                </defs>

                <line x1="80" y1="70" x2="920" y2="70" stroke="rgba(11,17,20,0.06)" strokeWidth="1.5" markerEnd="url(#mArrow)" />

                <g transform="translate(80,70)">
                  <text x="0" y="-10" className={styles.flowNodeTitle}>Constitution</text>
                  <text x="0" y="10" className={styles.flowNodeSub}>Principles</text>
                </g>

                <g transform="translate(260,70)">
                  <text x="0" y="-10" className={styles.flowNodeTitle}>Specifications</text>
                  <text x="0" y="10" className={styles.flowNodeSub}>Specs & contracts</text>
                </g>

                <g transform="translate(460,70)">
                  <text x="0" y="-10" className={styles.flowNodeTitle}>Plans &amp; Tasks</text>
                  <text x="0" y="10" className={styles.flowNodeSub}>Design → Tasks</text>
                </g>

                <g transform="translate(660,70)">
                  <text x="0" y="-10" className={styles.flowNodeTitle}>Implementation</text>
                  <text x="0" y="10" className={styles.flowNodeSub}>Code & artifacts</text>
                </g>

                <g transform="translate(860,70)">
                  <text x="0" y="-10" className={styles.flowNodeTitle}>Validation</text>
                  <text x="0" y="10" className={styles.flowNodeSub}>Tests & repro</text>
                </g>

              </svg>
            </div>
          </div>
        </section>

        {/* CURRICULUM OVERVIEW */}
        <section className={styles.chaptersSection}>
          <div className={styles.container}>
            <h2 className={styles.sectionHeading}>Curriculum overview</h2>
            <p className={styles.curriculumIntro}>The curriculum presents a canonical progression of modules with explicit objectives and reproducible artifacts that support validation and peer review.</p>

            <ol className={styles.chaptersGrid}>
              {chapters.map((label, idx) => (
                <li key={label} className={styles.chapterRow}>
                  <span className={styles.chapterNum}>{String(idx + 1).padStart(2, '0')}</span>
                  <span className={styles.chapterText}>{label}</span>
                </li>
              ))}
            </ol>

            <p className={styles.closingStatement}>The curriculum emphasizes rigorous specification and reproducible validation.</p>
          </div>
        </section>

      </main>
    </Layout>
  );
}
