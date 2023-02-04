import { Button } from './components';

/**
 * The background colour to match openmct
 */
const BG_COLOUR = '#393939';

const App = () => {
  return (
    <main
      className={`p-4 flex flex-col justify-between h-full bg-black text-white`}
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
        {/* Buttons */}
        <div className="grid grid-rows-10 gap-4">
          <Button onClick={() => {}} colour="blue" />
          <Button onClick={() => {}} colour="green" />
          <Button onClick={() => {}} colour="red" />
        </div>
      </div>
      <div className="mx-auto"><img src="hyped.svg" /></div>
    </main>
  );
};

export default App;
