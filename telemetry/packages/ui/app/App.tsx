import {
  Logo,
  StatusIndicator,
  ControlButton,
  ControlSwitch,
} from '@/components';
import { StatusType } from '@/types/StatusType';
import { Button } from './components/ui/button';
import { cn } from './lib/utils';
import { useState } from 'react';
import toast from 'react-hot-toast';

const App = () => {
  const LATENCY = 11; // temp
  const STATUS: StatusType = 'connected';
  const STATIONARY = false; // temp

  const [motorCooling, setMotorCooling] = useState(false);
  const [activeSuspension, setActiveSuspension] = useState(false);
  const [launched, setLaunched] = useState(false);

  /**
   * Starts pod
   */
  const go = () => {
    if (motorCooling && activeSuspension) {
      console.log('GO! (with motor cooling and active suspension)');
    } else if (motorCooling) {
      console.log('GO! (with motor cooling)');
    } else {
      console.log('GO! (with active suspension)');
    }
    setTimeout(() => {
      setLaunched(true);
      toast.success('Pod launched!');
    }, 1000);

    setTimeout(() => {
      setLaunched(false);
      toast('Pod stopped!', { icon: '🛑' });
    }, 10000);
  };

  /**
   * Stops pod
   */
  const stop = () => {
    console.log('STOP!');
    toast('Pod stopped!', { icon: '🛑' });
  };

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
    <main className="px-4 py-8 flex flex-col justify-between h-full bg-[#393939] select-none text-gray-100">
      <div className="flex flex-col justify-between h-full">
        <div className="space-y-2">
          <StatusIndicator status={STATUS} />
          <p>
            <span className="italic">Latency: </span>
            <span className="text-sm">{LATENCY} ms</span>
          </p>
          <h1 className="text-5xl font-title font-black">Controls</h1>
        </div>
        <div>
          <div className="flex flex-col gap-4 mb-16">
            <p className="text-3xl font-title font-bold underline">Options</p>
            <ControlSwitch
              id="motor-cooling"
              label="Motor Cooling"
              onCheckedChange={toggleMotorCooling}
              disabled={launched}
            />
            <ControlSwitch
              id="active-suspension"
              label="Active Suspension"
              onCheckedChange={toggleActiveSuspension}
              disabled={launched}
            />
          </div>
          <div className="flex flex-col gap-4">
            <ControlButton
              onClick={go}
              colour="green"
              text="GO"
              disabled={launched}
            />
            <ControlButton
              onClick={stop}
              colour="red"
              text="STOP"
              disabled={false}
            />
            {/* @ts-ignore */}
            <Button
              className={cn(
                'px-4 py-12 rounded-lg shadow-lg transition text-white text-3xl font-bold',
                STATIONARY
                  ? 'bg-blue-600 hover:bg-blue-700'
                  : 'opacity-50 cursor-not-allowed bg-gray-400',
              )}
              onClick={go}
            >
              Retract Brakes
            </Button>
          </div>
        </div>
        <Logo />
      </div>
    </main>
  );
};

export default App;
