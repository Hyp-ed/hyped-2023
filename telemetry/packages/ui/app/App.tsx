import { Logo, StatusIndicator } from './components';
import { StatusType } from '@/types/StatusType';
import { PodControls } from './components/pod-controls';
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from './components/ui/select';
import { useState } from 'react';

const App = () => {
  const LATENCY = 11; // TODOLater: replace with real value once latency is implemented
  const CONN_STATUS: StatusType = 'connected'; // TODOLater: replace with real value representing whether we are connected to ROS

  const podIds = ['pod_1', 'pod_2'];
  const [pod, setPod] = useState(podIds[0]);

  return (
    <main className="px-4 py-8 flex flex-col gap-2 justify-between h-full bg-[#393939] select-none text-gray-100">
      <div className="flex flex-col justify-between h-full">
        {/* Status, Latency, State, Title */}
        <div className="flex flex-col gap-4">
          <div className="flex flex-col gap-2">
            <StatusIndicator status={CONN_STATUS} />
            <p>
              <span className="">Latency: </span>
              <span className="text-sm">{LATENCY} ms</span>
            </p>
          </div>
          <h1 className="text-5xl font-title font-black my-4">Controls</h1>
        </div>
        <div className="flex flex-col justify-start h-full mt-4">
          {/* Select component to decide which pod to show the controls for */}
          {/* @ts-ignore */}
          <Select
            onValueChange={(podId) => setPod(podId)}
            defaultValue={pod}
            style={{ width: 'full' }}
          >
            {/* @ts-ignore */}
            <SelectTrigger>
              {/* @ts-ignore */}
              <SelectValue placeholder="Pod" />
            </SelectTrigger>
            {/* @ts-ignore */}
            <SelectContent>
              {podIds.map((podId) => (
                // @ts-ignore
                <SelectItem value={podId}>{podId}</SelectItem>
              ))}
            </SelectContent>
          </Select>
          {podIds.map((podId) => (
            <PodControls podId={podId} show={pod === podId} />
          ))}
        </div>
        <Logo />
      </div>
    </main>
  );
};

export default App;
