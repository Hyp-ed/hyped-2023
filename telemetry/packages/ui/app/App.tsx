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
import { useEffect, useState } from 'react';
import mqtt from 'mqtt/dist/mqtt';

const App = () => {
  const LATENCY = 11; // TODOLater: replace with real value once latency is implemented

  const podIds = ['pod_1'];
  const [pod, setPod] = useState(podIds[0]);

  const [client, setClient] = useState<mqtt.MqttClient | null>(null);
  const [connectStatus, setConnectStatus] =
    useState<StatusType>('disconnected');

  const MQTT_BROKER = 'ws://localhost:8080';

  const mqttPublish = ({
    topic,
    qos,
    payload,
  }: {
    topic: string;
    qos: mqtt.QoS | undefined;
    payload: string;
  }) => {
    if (client) {
      client.publish(`hyped/pod_1/${topic}`, payload, { qos }, (error: any) => {
        if (error) {
          console.log('Publish error: ', error);
        }
      });
    }
  };

  // Connect to MQTT broker on mount
  useEffect(() => {
    const mqttConnect = (
      host: string,
      mqttOption: mqtt.IClientOptions | undefined,
    ) => {
      setConnectStatus('connecting');
      setClient(mqtt.connect(host, mqttOption));
    };
    mqttConnect(MQTT_BROKER, undefined);
  }, []);

  useEffect(() => {
    if (client) {
      console.log(client);
      client.on('connect', () => {
        setConnectStatus('connected');
      });
      client.on('error', (err: any) => {
        console.error('Connection error: ', err);
        client.end();
      });
      client.on('reconnect', () => {
        setConnectStatus('reconnecting');
      });
    }
  }, [client]);

  return (
    <main className="px-4 py-8 flex flex-col gap-2 justify-between h-full bg-[#393939] select-none text-gray-100">
      <div className="flex flex-col justify-between h-full">
        {/* Status, Latency, State, Title */}
        <div className="flex flex-col gap-2">
          <div className="flex flex-col gap-1">
            <StatusIndicator status={connectStatus} />
            <p>
              <span className="">Latency: </span>
              <span className="text-sm">{LATENCY} ms</span>
            </p>
          </div>
          <h1 className="text-5xl font-title font-black my-2">Controls</h1>
        </div>
        <div className="flex flex-col justify-start h-full mt-4">
          {/* Select component to decide which pod to show the controls for */}
          <Select
            onValueChange={(podId) => setPod(podId)}
            defaultValue={pod}
            // @ts-ignore
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
              mqttPublish={mqttPublish}
            />
          ))}
        </div>
        <Logo />
      </div>
    </main>
  );
};

export default App;
