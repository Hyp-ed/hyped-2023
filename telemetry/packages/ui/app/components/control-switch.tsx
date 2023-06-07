import { Label } from './ui/label';
import { Switch } from './ui/switch';

interface ControlSwitchProps {
  onCheckedChange: (active: boolean) => void;
  label: string;
  id: string;
  disabled: boolean;
}

export const ControlSwitch = ({
  onCheckedChange,
  label,
  id,
  disabled,
}: ControlSwitchProps) => (
  <div className="flex justify-between items-center">
    {/* @ts-ignore */}
    <Label htmlFor={id}>{label}</Label>
    {/* @ts-ignore */}
    <Switch id={id} onCheckedChange={onCheckedChange} disabled={disabled} />
  </div>
);
