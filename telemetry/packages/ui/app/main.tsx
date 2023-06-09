import * as React from 'react';
import ReactDOM from 'react-dom/client';
import App from './App';
import './globals.css';
import 'victormono';
import '@fontsource/raleway';
import { Toaster } from 'react-hot-toast';

ReactDOM.createRoot(document.getElementById('root') as HTMLElement).render(
  // @ts-ignore
  <React.StrictMode>
    <App />
    {/* @ts-ignore */}
    <Toaster position="bottom-center" reverseOrder={false} />
  </React.StrictMode>,
);
