/** @type {import('tailwindcss').Config} */

module.exports = {
  content: ['./app/index.html', './app/**/*.{js,ts,jsx,tsx}'],
  theme: {
    extend: {
      fontFamily: {
        sans: ['Victor Mono'],
        title: ['Raleway'],
      },
    },
  },
};
