import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

export default function Home(): JSX.Element {
  const {siteConfig = {}} = useDocusaurusContext();

  const chapters = Array.from({length: 13}).map((_, i) => {
    const n = String(i + 1).padStart(2, '0');
    return {
      label: `Chapter ${n}`,
      to: `/docs/chapters/chapter-${n}/concept`,
    };
  });

  return (
    <Layout
      title={siteConfig.title}
      description="A system-engineered textbook for physical AI and humanoid robotics">
      <div className={styles.root}>
        <header className={styles.masthead} role="banner">
          <div className={styles.brand}>
            <h1 className={styles.title}>{siteConfig.title}</h1>
            <p className={styles.tagline}>{siteConfig.tagline}</p>
            <div className={styles.ctaRow}>
              <Link className={`button button--secondary`} to="/docs/intro">Specs & Quickstart</Link>
              <Link className={`button button--primary`} to={chapters[0].to}>Start reading</Link>
            </div>
          </div>

          <aside className={styles.sideCards} aria-hidden>
            <div className={styles.sideCard}>
              <h4 className={styles.sideCardTitle}>What the book is</h4>
              <ul className={styles.sideList}>
                <li>Systems‑first textbook for physical AI & humanoid robotics</li>
                <li>Architecture, validation, reproducible artifacts</li>
                <li>Engineered examples over narrative prose</li>
              </ul>
            </div>

            <div className={styles.sideCard}>
              <h4 className={styles.sideCardTitle}>Agents & validation</h4>
              <ul className={styles.sideList}>
                <li>Agents run structured checks on specs</li>
                <li>Validators produce pass/fail artifacts</li>
                <li>Reviewer checklists close the loop</li>
              </ul>
            </div>
          </aside>
        </header>

        <main className={styles.main}>
          <section className={styles.sdd}>
            <h2 className={styles.sectionHeading}>Specs‑Driven Development</h2>
            <div className={styles.sddFlow}>
              <div className={styles.sddStep}>Constitution</div>
              <div className={styles.sddArrow}>›</div>
              <div className={styles.sddStep}>Specs</div>
              <div className={styles.sddArrow}>›</div>
              <div className={styles.sddStep}>Plan → Tasks</div>
              <div className={styles.sddArrow}>›</div>
              <div className={styles.sddStep}>Validation</div>
            </div>
            <p className={styles.sddNote}>Visual, reproducible, auditable — each step links to chapter artifacts and validators.</p>
          </section>

          <section className={styles.hierarchy}>
            <h2 className={styles.sectionHeading}>Constitution & specs hierarchy</h2>
            <div className={styles.tree}>
              <div className={styles.treeNode}>
                <div className={styles.nodeTitle}>Constitution</div>
                <div className={styles.nodeChildren}>
                  <div className={styles.nodeLeaf}>Chapter schemas</div>
                  <div className={styles.nodeLeaf}>Reviewer checklists</div>
                  <div className={styles.nodeLeaf}>Automated validators</div>
                </div>
              </div>
            </div>
          </section>

          <section className={styles.progression}>
            <h2 className={styles.sectionHeading}>Chapter progression</h2>
            <ol className={styles.progressBar} aria-label="Chapter progression">
              {chapters.map((c, idx) => (
                <li key={c.to} className={styles.progressStep}>
                  <Link to={c.to} className={styles.stepLink}>
                    <span className={styles.stepNumber}>{String(idx + 1).padStart(2, '0')}</span>
                    <span className={styles.stepLabel}>{c.label}</span>
                  </Link>
                </li>
              ))}
            </ol>
            <p className={styles.progressNote}>Follow the canonical progression starting at Chapter 01.</p>
          </section>
        </main>
      </div>
    </Layout>
  );
}
