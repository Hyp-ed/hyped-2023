import { Logo, StatusIndicator } from './components';
import { PodControls } from './components/pod-controls';
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from './components/ui/select';
import { useState } from 'react';
import { useMQTT } from './hooks/useMQTT';

const App = () => {
  const { connectionStatus, publish, latency } = useMQTT();

  const podIds = ['pod_1'];
  const [pod, setPod] = useState(podIds[0]);

  return (
    <main className="px-4 py-8 flex flex-col gap-2 justify-between h-full bg-[#393939] select-none text-gray-100">
      <div className="flex flex-col justify-between h-full">
        {/* Status, Latency, State, Title */}
        <div className="flex flex-col gap-2">
          <div className="flex flex-col gap-1">
            <StatusIndicator status={connectionStatus} />
            <p>
              <span className="">Latency: </span>
              <span className="text-sm">{latency} ms</span>
            </p>
          </div>
          <h1 className="text-5xl font-title font-black my-2">Controls</h1>
        </div>
        <div className="flex flex-col justify-start h-full mt-4">
          {/* Select component to decide which pod to show the controls for */}
          <Select
            onValueChange={(podId) => setPod(podId)}
            defaultValue={pod}
            style={{ width: 'full' }}
          >
            <SelectTrigger>
              <SelectValue placeholder="Pod" />
            </SelectTrigger>
            <SelectContent>
              {podIds.map((podId) => (
                <SelectItem key={podId} value={podId}>
                  {podId}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
          {podIds.map((podId) => (
            <PodControls
              key={podId}
              podId={podId}
              show={pod === podId}
              publish={publish}
            />
          ))}
        </div>
        <Logo />
      </div>
    </main>
  );
};

export default App;
