import * as React from 'react';
import ReactDOM from 'react-dom/client';
import App from './App';
import './globals.css';
import 'victormono';
import '@fontsource/raleway';

ReactDOM.createRoot(document.getElementById('root') as HTMLElement).render(
  // @ts-ignore
  <React.StrictMode>
    <App />
  </React.StrictMode>,
);
