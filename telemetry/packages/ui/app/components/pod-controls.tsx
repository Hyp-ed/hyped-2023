import { toast } from 'react-hot-toast';
import { useState } from 'react';
import { PodStateIndicator } from './pod-state';
import { PodState, failureStates, idleStates } from '@/types/PodState';
import { Button } from './ui/button';
import { Label } from './ui/label';
import { Switch } from './ui/switch';
import { cn } from '@/lib/utils';
import { MqttPublish } from '@/types/mqtt';
import {
  calibrate,
  raise,
  retract,
  startPod,
  stopPod,
} from '@/controls/controls';

interface PodControlsProps {
  podId: string;
  show: boolean;
  mqttPublish: MqttPublish;
}

export const PodControls = ({ podId, show, mqttPublish }: PodControlsProps) => {
  const POD_STATE: PodState = idleStates.idle; // TODOLater: replace with real value once we can read pod state from ROS

  const [motorCooling, setMotorCooling] = useState(false);
  const [activeSuspension, setActiveSuspension] = useState(false);

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

  return (
    <div className={cn('my-8 space-y-8', show ? 'block' : 'hidden')}>
      <PodStateIndicator state={POD_STATE} />
      <div className="space-y-6">
        <div className="flex flex-col gap-2">
          {/* <p className="text-3xl font-title font-bold underline">Options</p> */}
          <div className="flex justify-between items-center">
            {/* @ts-ignore */}
            <Label htmlFor="motor-cooling">Motor Cooling</Label>
            {/* @ts-ignore */}
            <Switch
              id="motor-cooling"
              onCheckedChange={toggleMotorCooling}
              disabled={SWITCHES_DISABLED}
            />
          </div>
          <div className="flex justify-between items-center">
            {/* @ts-ignore */}
            <Label htmlFor="active-suspension">Active Suspension</Label>
            {/* @ts-ignore */}
            <Switch
              id="active-suspension"
              onCheckedChange={toggleActiveSuspension}
              disabled={SWITCHES_DISABLED}
            />
          </div>
        </div>
        <div className="flex flex-col gap-2">
          {/* @ts-ignore */}
          <Button
            className={cn(
              'px-4 py-10 rounded-md shadow-lg transition text-white text-3xl font-bold',
              'bg-yellow-600 hover:bg-yellow-700',
            )}
            onClick={() => calibrate(podId)}
          >
            CALIBRATE
          </Button>
          {/* @ts-ignore */}
          <Button
            className={cn(
              'px-4 py-10 rounded-md shadow-lg transition text-white text-3xl font-bold',
              'bg-green-600 hover:bg-green-700',
            )}
            onClick={() =>
              startPod(podId, {
                motorCooling,
                activeSuspension,
              })
            }
          >
            START RUN
          </Button>
          {/* @ts-ignore */}
          <Button
            className={cn(
              'px-4 py-10 rounded-md shadow-lg transition text-white text-3xl font-bold',
              'bg-red-700 hover:bg-red-800',
            )}
            onClick={() => stopPod(podId)}
          >
            STOP RUN
          </Button>
          {/* @ts-ignore */}
          <Button
            className={cn(
              'px-4 py-10 rounded-md shadow-lg transition text-white text-3xl font-bold',
              'bg-gray-600 hover:bg-gray-700',
            )}
            onClick={() => retract(podId, mqttPublish)}
          >
            Retract Brakes
          </Button>
          {/* @ts-ignore */}
          <Button
            className={cn(
              'px-4 py-10 rounded-md shadow-lg transition text-white text-3xl font-bold',
              'bg-gray-600 hover:bg-gray-700',
            )}
            // @ts-ignore
            onClick={() => raise(podId, mqttPublish)}
          >
            Raise Pod
          </Button>
        </div>
      </div>
    </div>
  );
};