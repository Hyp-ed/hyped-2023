/** @type {import('tailwindcss').Config} */
const plugin = require('tailwindcss/plugin');

module.exports = {
  content: ['./app/index.html', './app/**/*.{js,ts,jsx,tsx}'],
  theme: {
    extend: {},
  },
};
