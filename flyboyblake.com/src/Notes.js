import React from 'react';
import './index.scss';
import FadeIn from 'react-fade-in';

const data = require('./notes.json')

export default function Notes() {
  return (
    <FadeIn>
      <h2 style={{ marginTop: '3rem' }}>notes.</h2>
      {data.notes.map(note => (
        <div style={{ marginBottom: '3rem' }}>
          <p style={{ paddingBottom: 0, marginBottom: 0 }}>{ note.text }</p>
          <p style={{ fontSize: '12px', paddingTop: 0, marginTop: 0 }}>{ note.date }</p>
        </div>
      ))} 
    </FadeIn>
  );
}
