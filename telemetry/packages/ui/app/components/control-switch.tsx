import { Label } from './ui/label';
import { Switch } from './ui/switch';

interface ControlSwitchProps {
  onCheckedChange: (active: boolean) => void;
  label: string;
  id: string;
}

export const ControlSwitch = ({
  onCheckedChange,
  label,
  id,
}: ControlSwitchProps) => (
  <div className="flex justify-between items-center">
    {/* @ts-ignore */}
    <Label htmlFor={id}>{label}</Label>
    {/* @ts-ignore */}
    <Switch id={id} onCheckedChange={onCheckedChange} />
  </div>
);
