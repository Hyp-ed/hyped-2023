import { toast } from 'react-hot-toast';
import { useEffect, useState } from 'react';
import { PodState } from './pod-state';
import { Button } from '@/components/ui/button';
import { Label } from '@/components/ui/label';
import { Switch } from '@/components/ui/switch';
import { cn } from '@/lib/utils';
import {
  clamp,
  lower,
  raise,
  retract,
  startPod,
  stopPod,
  startHP,
  stopHP,
} from '@/controls/controls';
import { usePod } from '@/context/pods';

interface PodControlsProps {
  podId: string;
  show: boolean;
}

export const PodControls = ({ podId, show }: PodControlsProps) => {
  const { podState } = usePod(podId);

  const [motorCooling, setMotorCooling] = useState(false);
  const [activeSuspension, setActiveSuspension] = useState(false);
  const [clamped, setClamped] = useState(false);
  const [raised, setRaised] = useState(false);
  const [deadmanSwitch, setDeadmanSwitch] = useState(false);
  const [stopped, setStopped] = useState(true);

  const SWITCHES_DISABLED = false; //TODOLater: replace with logic to determine whether switches should be disabled

  /**
   * Toggles motor cooling
   * @param active Whether motor cooling is active
   */
  const toggleMotorCooling = (active: boolean) => {
    setMotorCooling(active);
    toast(active ? 'Motor cooling enabled' : 'Motor cooling disabled');
  };

  /**
   * Toggles active suspension
   * @param active Whether active suspension is active
   */
  const toggleActiveSuspension = (active: boolean) => {
    setActiveSuspension(active);
    toast(active ? 'Active suspension enabled' : 'Active suspension disabled');
  };

  // Display notification when the pod state changes
  useEffect(() => {
    toast(`Pod state changed: ${podState}`);
  }, [podState]);

  return (
    <div className={cn('my-8 space-y-8', show ? 'block' : 'hidden')}>
      <PodState state={podState} />
      <div className="space-y-6">
        <div className="flex flex-col gap-2">
          <div className="flex justify-between items-center">
            <Label htmlFor="motor-cooling">Motor Cooling</Label>
            <Switch
              id="motor-cooling"
              onCheckedChange={toggleMotorCooling}
              disabled={SWITCHES_DISABLED}
            />
          </div>
          <div className="flex justify-between items-center">
            <Label htmlFor="active-suspension">Active Suspension</Label>
            <Switch
              id="active-suspension"
              onCheckedChange={toggleActiveSuspension}
              disabled={SWITCHES_DISABLED}
            />
          </div>
        </div>
        <div className="flex flex-col gap-2">
          {stopped ? (
            <Button
              className={cn(
                'px-4 py-10 rounded-md shadow-lg transition text-white text-3xl font-bold',
                'bg-green-600 hover:bg-green-700',
              )}
              onClick={() => {
                startPod(podId, {
                  motorCooling,
                  activeSuspension,
                });
                setStopped(false);
              }}
            >
              START RUN
            </Button>
          ) : (
            <Button
              className={cn(
                'px-4 py-10 rounded-md shadow-lg transition text-white text-3xl font-bold',
                'bg-red-700 hover:bg-red-800',
              )}
              onClick={() => {
                stopPod(podId);
                setStopped(true);
              }}
            >
              STOP RUN
            </Button>
          )}
          <Button
            className={cn(
              'px-4 py-10 rounded-md shadow-lg transition text-white text-3xl font-bold',
              clamped && 'bg-blue-600 hover:bg-blue-700',
              !clamped && 'bg-gray-600 hover:bg-gray-700',
            )}
            onClick={() => {
              if (clamped) retract(podId);
              else clamp(podId);
              setClamped(!clamped);
            }}
          >
            {clamped ? 'Retract Brakes' : 'Clamp Brakes'}
          </Button>
          <Button
            className={cn(
              'px-4 py-10 rounded-md shadow-lg transition text-white text-3xl font-bold',
              raised && 'bg-blue-600 hover:bg-blue-700',
              !raised && 'bg-gray-600 hover:bg-gray-700',
            )}
            onClick={() => {
              if (raised) lower(podId);
              else raise(podId);
              setRaised(!raised);
            }}
          >
            {raised ? 'Lower Pod' : 'Raise Pod'}
          </Button>
          <Button
            className={cn(
              'px-4 py-10 rounded-md shadow-lg transition text-white text-3xl font-bold',
              deadmanSwitch && 'bg-red-600 hover:bg-red-700',
              !deadmanSwitch && 'bg-gray-600 hover:bg-gray-700',
            )}
            onClick={() => {
              if (deadmanSwitch) stopHP(podId);
              else startHP(podId);
              setDeadmanSwitch(!deadmanSwitch);
            }}
          >
            {deadmanSwitch ? 'HP Active' : 'HP Inactive'}
          </Button>
        </div>
      </div>
    </div>
  );
};
