import { toast } from 'react-hot-toast';
import { useState } from 'react';
import { PodStateIndicator } from './pod-state';
import { PodState, podStates } from '@/types/PodState';
import { Button } from './ui/button';
import { Label } from './ui/label';
import { Switch } from './ui/switch';
import { cn } from '@/lib/utils';
import { MqttPublish } from '@/types/mqtt';
import {
  calibrate,
  clamp,
  lower,
  raise,
  retract,
  startPod,
  stopPod,
  startHP,
  stopHP,
} from '@/controls/controls';

interface PodControlsProps {
  podId: string;
  show: boolean;
  mqttPublish: MqttPublish;
}

export const PodControls = ({ podId, show, mqttPublish }: PodControlsProps) => {
  const POD_STATE: PodState = podStates.failureBraking; // TODOLater: replace with real value once we can read pod state from ROS

  const [motorCooling, setMotorCooling] = useState(false);
  const [activeSuspension, setActiveSuspension] = useState(false);

  const [clamped, setClamped] = useState(false);
  const [raised, setRaised] = useState(false);
  const [deadman_switch, setDeadmanSwitch] = useState(false);

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
                   {/* <Button
            className={cn(
              'px-4 py-10 rounded-md shadow-lg transition text-white text-3xl font-bold',
              'bg-yellow-600 hover:bg-yellow-700',
            )}
            onClick={() => calibrate(podId)}
          >
            CALIBRATE
          </Button> */}
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
                   <Button
            className={cn(
              'px-4 py-10 rounded-md shadow-lg transition text-white text-3xl font-bold',
              'bg-red-700 hover:bg-red-800',
            )}
            onClick={() => stopPod(podId)}
          >
            STOP RUN
          </Button>
                   <Button
            className={cn(
              'px-4 py-10 rounded-md shadow-lg transition text-white text-3xl font-bold',
              clamped && 'bg-blue-600 hover:bg-blue-700',
              !clamped && 'bg-gray-600 hover:bg-gray-700',
            )}
            onClick={() => {
              if (clamped) retract(podId, mqttPublish);
              else clamp(podId, mqttPublish);
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
            // @ts-ignore
            onClick={() => {
              if (raised) lower(podId, mqttPublish);
              else raise(podId, mqttPublish);
              setRaised(!raised);
            }}
          >
            {raised ? 'Lower Pod' : 'Raise Pod'}
          </Button>
                   <Button
            className={cn(
              'px-4 py-10 rounded-md shadow-lg transition text-white text-3xl font-bold',
              deadman_switch && 'bg-red-600 hover:bg-red-700',
              !deadman_switch && 'bg-gray-600 hover:bg-gray-700',
            )}
            // @ts-ignore
            onClick={() => {
              if (deadman_switch) stopHP(podId, mqttPublish);
              else startHP(podId, mqttPublish);
              setDeadmanSwitch(!deadman_switch);
            }}
          >
            {deadman_switch ? 'HP Active' : 'HP Inactive'}
          </Button>
        </div>
      </div>
    </div>
  );
};
