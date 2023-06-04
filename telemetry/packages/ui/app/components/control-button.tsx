import { Button } from './ui/button';

export interface ControlButtonProps {
  text: string;
  colour: 'green' | 'red';
  onClick: () => void;
}

export const ControlButton = ({
  text,
  colour,
  onClick,
}: ControlButtonProps) => (
  // @ts-ignore
  <Button
    className={`px-4 py-12 rounded-lg shadow-lg ${
      colour == 'red'
        ? 'bg-red-600 hover:bg-red-700'
        : colour == 'green'
        ? 'bg-green-600 hover:bg-green-700'
        : ''
    } transition text-white text-3xl font-bold`}
    onClick={onClick}
  >
    {text.toUpperCase()}
  </Button>
);
