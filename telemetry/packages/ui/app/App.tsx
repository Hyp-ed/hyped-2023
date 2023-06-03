import {
  Logo,
  StatusIndicator,
  ControlButton,
  ControlSwitch,
} from '@/components';
import { StatusType } from '@/types/StatusType';

const App = () => {
  const LATENCY = 11; // temp
  const STATUS: StatusType = 'connected';

  /**
   * Starts pod
   */
  const go = () => {
    console.log('GO!');
  };

  /**
   * Stops pod
   */
  const stop = () => {
    console.log('STOP!');
  };

  /**
   * Toggles motor cooling
   * @param active Whether motor cooling is active
   */
  const toggleMotorCooling = (active: boolean) => {
    console.log(`Toggle motor cooling: ${active ? 'active' : 'inactive'}`);
  };

  /**
   * Toggles active suspension
   * @param active Whether active suspension is active
   */
  const toggleActiveSuspension = (active: boolean) => {
    console.log(`Toggle active suspension: ${active ? 'active' : 'inactive'}`);
  };

  /**
   * Toggles active suspension
   * @param active Whether active suspension is active
   */
  const toggleActiveBraking = (active: boolean) => {
    console.log(`Toggle active braking: ${active ? 'active' : 'inactive'}`);
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
            />
            <ControlSwitch
              id="active-suspension"
              label="Active Suspension"
              onCheckedChange={toggleActiveSuspension}
            />
            <ControlSwitch
              id="active-braking"
              label="Active Braking"
              onCheckedChange={toggleActiveBraking}
            />
          </div>
          <div className="flex flex-col gap-4">
            <ControlButton onClick={go} colour="green" text="GO" />
            <ControlButton onClick={stop} colour="red" text="STOP" />
          </div>
        </div>
        <Logo />
      </div>
    </main>
  );
};

export default App;
