import { cn } from '@/lib/utils';
import { Button } from './ui/button';

export interface ControlButtonProps {
  text: string;
  colour: 'green' | 'red';
  onClick: () => void;
  disabled: boolean;
}

export const ControlButton = ({
  text,
  colour,
  onClick,
  disabled,
}: ControlButtonProps) => (
  // @ts-ignore
  <Button
    // className={`px-4 py-12 rounded-lg shadow-lg ${
    //   colour == 'red'
    //     ? 'bg-red-600 hover:bg-red-700'
    //     : colour == 'green'
    //     ? 'bg-green-600 hover:bg-green-700'
    //     : ''
    // } transition text-white text-3xl font-bold`}
    className={cn(
      'px-4 py-12 rounded-lg shadow-lg transition text-white text-3xl font-bold',
      colour == 'red'
        ? 'bg-red-600 hover:bg-red-700'
        : colour == 'green'
        ? 'bg-green-600 hover:bg-green-700'
        : '',
      disabled ? 'opacity-50 cursor-not-allowed' : '',
    )}
    onClick={onClick}
  >
    {text.toUpperCase()}
  </Button>
);
