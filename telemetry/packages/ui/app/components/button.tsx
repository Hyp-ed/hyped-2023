export interface ButtonProps {
  /**
   * The text to display on the button
   * @default 'Click me'
   * */
  text?: string;
  /**
   * The type of button
   * @default 'blue'
   * */
  colour?: 'blue' | 'green' | 'red';
  /**
   * The size of the button
   * @default 'medium'
   * */
  size?: 'small' | 'medium' | 'large';
  /**
   * The function to call when the button is clicked
   * */
  onClick: () => void;
}

const Button = ({ text, colour, size, onClick }: ButtonProps) => (
  <button
    className={`px-4 py-8 rounded-lg shadow-md bg-green-500 hover:bg-green-700 transition text-white text-${size}`}
    onClick={onClick}
  >
    {text}
  </button>
);

export default Button;

Button.defaultProps = {
  text: 'Click me',
  type: 'blue',
  size: 'medium',
};
