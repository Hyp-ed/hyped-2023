import { Switch } from '@headlessui/react';
import { Fragment, useEffect, useState } from 'react';

export interface ToggleProps {
  text: string;
  onChange: (active: boolean) => void;
}

export const Toggle = ({ text, onChange }: ToggleProps) => {
  const [enabled, setEnabled] = useState(false);

  useEffect(() => {
    onChange(enabled);
  }, [enabled]);

  return (
    <Switch.Group>
      <div className="flex justify-between">
        {/* @ts-ignorenext-line */}
        <Switch.Label className="mr-4">{text}</Switch.Label>
        <div className="min-w-min">
          <Switch checked={enabled} onChange={setEnabled} as={Fragment}>
            {({ checked }) => (
              <button
                className={`${
                  checked ? 'bg-blue-600' : 'bg-[#535353]'
                } relative inline-flex h-6 w-11 items-center rounded-full`}
              >
                <span className="sr-only">{text}</span>
                <span
                  className={`${
                    checked ? 'translate-x-6' : 'translate-x-1'
                  } inline-block h-4 w-4 transform rounded-full bg-white transition`}
                />
              </button>
            )}
          </Switch>
        </div>
      </div>
    </Switch.Group>
  );
};
