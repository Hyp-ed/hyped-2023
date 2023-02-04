export interface ButtonProps {
  text: string;
  colour: 'green' | 'red';
  onClick: () => void;
}

const Button = ({ text, colour, onClick }: ButtonProps) => (
  <button
    className={`px-4 py-8 rounded-lg shadow-lg ${
      colour == 'red'
        ? 'bg-red-600 hover:bg-red-700'
        : colour == 'green'
        ? 'bg-green-600 hover:bg-green-700'
        : ''
    } transition text-white text-xl font-bold`}
    onClick={onClick}
  >
    {text.toUpperCase()}
  </button>
);

export default Button;
