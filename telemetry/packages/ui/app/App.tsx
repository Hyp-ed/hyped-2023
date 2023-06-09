import { Logo, StatusIndicator } from './components';
import { StatusType } from '@/types/StatusType';
import { cn } from './lib/utils';
import { useState } from 'react';
import toast from 'react-hot-toast';
import { Button } from '@/components/ui/button';
import { Label } from './components/ui/label';
import { Switch } from './components/ui/switch';

const App = () => {
  const LATENCY = 11; // temp
  const CONN_STATUS: StatusType = 'connected';

  const [motorCooling, setMotorCooling] = useState(false);
  const [activeSuspension, setActiveSuspension] = useState(false);

  const calibrate = () => {
    console.log('Calibrating');
    toast('Calibrating!');
  };

  const go = () => {
    if (motorCooling && activeSuspension) {
      console.log('GO! (with motor cooling and active suspension)');
      toast.success(
        'Pod launched (with motor cooling and active suspension)!',
        { icon: '🚀' },
      );
    } else if (motorCooling) {
      console.log('GO! (with motor cooling)');
      toast.success('Pod launched (with motor cooling)!', { icon: '🚀' });
    } else if (activeSuspension) {
      console.log('GO! (with active suspension)');
      toast.success('Pod launched with (active suspension)!', { icon: '🚀' });
    } else {
      console.log('GO!');
      toast.success('Pod launched!', { icon: '🚀' });
    }
  };

  const stop = () => {
    console.log('STOP!');
    toast('Pod stopped!', { icon: '🛑' });
  };

  const retractBrakes = () => {
    console.log('Retracting brakes');
    toast('Brakes retracted!');
  };

  const clampBrakes = () => {
    console.log('Clamping brakes');
    toast('Brakes clamped!');
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
          <StatusIndicator status={CONN_STATUS} />
          <p>
            <span className="italic">Latency: </span>
            <span className="text-sm">{LATENCY} ms</span>
          </p>
          <h1 className="text-5xl font-title font-black">Controls</h1>
        </div>
        <div>
          <div className="flex flex-col gap-4 mb-16">
            <p className="text-3xl font-title font-bold underline">Options</p>
            <div className="flex justify-between items-center">
              {/* @ts-ignore */}
              <Label htmlFor="motor-cooling">Motor Cooling</Label>
              {/* @ts-ignore */}
              <Switch
                id="motor-cooling"
                onCheckedChange={toggleMotorCooling}
                disabled={false}
              />
            </div>
            <div className="flex justify-between items-center">
              {/* @ts-ignore */}
              <Label htmlFor="active-suspension">Active Suspension</Label>
              {/* @ts-ignore */}
              <Switch
                id="active-suspension"
                onCheckedChange={toggleActiveSuspension}
                disabled={false}
              />
            </div>
          </div>
          <div className="flex flex-col gap-4">
            {/* @ts-ignore */}
            <Button
              className={cn(
                'px-4 py-12 rounded-lg shadow-lg transition text-white text-3xl font-bold',
                'bg-yellow-600 hover:bg-yellow-700',
              )}
              onClick={calibrate}
            >
              CALIBRATE
            </Button>
            {/* @ts-ignore */}
            <Button
              className={cn(
                'px-4 py-12 rounded-lg shadow-lg transition text-white text-3xl font-bold',
                'bg-green-600 hover:bg-green-700',
              )}
              onClick={go}
            >
              START RUN
            </Button>
            {/* @ts-ignore */}
            <Button
              className={cn(
                'px-4 py-12 rounded-lg shadow-lg transition text-white text-3xl font-bold',
                'bg-red-600 hover:bg-red-700',
              )}
              onClick={stop}
            >
              STOP RUN
            </Button>
            {/* @ts-ignore */}
            <Button
              className={cn(
                'px-4 py-12 rounded-lg shadow-lg transition text-white text-3xl font-bold',
                'bg-blue-600 hover:bg-blue-700',
              )}
              onClick={retractBrakes}
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
