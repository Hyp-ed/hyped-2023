import { Button, Checkbox, StatusIndicator } from './components';

const App = () => {
  const LATENCY = 10; // temp
  const STATUS: 'connected' | 'disconnected' | 'connecting' = 'connecting';

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

  return (
    <main
      className={`px-4 py-8 flex flex-col justify-between h-full bg-[#393939] text-white select-none text-gray-100`}
    >
      <div className="flex flex-col justify-between h-full">
        <div>
          <StatusIndicator status={STATUS} />
          <p>
            <span className="italic">Latency: </span>
            <span className="text-sm">{LATENCY} ms</span>
          </p>
          <h1 className="text-4xl font-black">Controls</h1>
        </div>
        <div>
          <div className="flex flex-col gap-4 mb-16">
            <p className="text-xl font-bold">Options:</p>
            <Checkbox onChange={toggleMotorCooling} text="Motor Cooling" />
            <Checkbox
              onChange={toggleActiveSuspension}
              text="Active Suspension"
            />
          </div>
          <div className="flex flex-col gap-4">
            <Button onClick={go} colour="green" text="GO" />
            <Button onClick={stop} colour="red" text="STOP" />
          </div>
        </div>
        <div className="mx-auto">
          <img src="/hyped.png" />
        </div>
      </div>
    </main>
  );
};

export default App;