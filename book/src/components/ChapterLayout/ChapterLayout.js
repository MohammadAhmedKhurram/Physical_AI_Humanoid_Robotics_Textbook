import React from 'react';

export default function ChapterLayout({ children, chapterNumber, title, subtitle, intent }) {
  return (
    <div className="chapter-layout">
      <header>
        <h1>{chapterNumber} — {title}</h1>
        {subtitle && <p className="subtitle">{subtitle}</p>}
        {intent && <p className="intent"><em>{intent}</em></p>}
      </header>
      <main>{children}</main>
    </div>
  );
}

export function Summary({ children }) {
  return <div className="chapter-summary">{children}</div>;
}

export function NextChapter({ children }) {
  return <div className="chapter-next">{children}</div>;
}
