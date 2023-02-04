import { Button } from './components';
import Checkbox from './components/checkbox';

const App = () => {
  const go = () => {
    console.log('GO!');
  };

  const stop = () => {
    console.log('STOP!');
  };

  const toggleMotorCooling = () => {
    console.log('Toggle motor cooling');
  };

  const toggleActiveSuspension = () => {
    console.log('Toggle active suspension');
  };

  return (
    <main
      className={`p-4 flex flex-col justify-between h-full bg-[#333] text-white`}
    >
      <div className="space-y-8">
        <h1 className="text-xl font-bold">Controls</h1>
        {/* Stats */}
        <div>
          <h2 className="text-lg font-bold">Stats</h2>
          <div className="grid grid-cols-2 gap-4">
            <span className="text-sm">Latency: 0 ms</span>
          </div>
        </div>
        {/* Controls */}
        <div className="flex flex-col gap-4">
          <Checkbox onChange={toggleMotorCooling} text="Motor Cooling" />
          <Checkbox
            onChange={toggleActiveSuspension}
            text="Active Suspension"
          />
          <Button onClick={go} colour="green" text="GO" />
          <Button onClick={stop} colour="red" text="STOP" />
        </div>
      </div>
      <div className="mx-auto">
        <img src="hyped.png" />
      </div>
    </main>
  );
};

export default App;
