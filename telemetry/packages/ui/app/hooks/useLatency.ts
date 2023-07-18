import { MqttPublish, MqttSubscribe } from '@hyped/telemetry-types';
import { MqttClient } from 'mqtt/types/lib/client';
import { useEffect, useState } from 'react';

export const useLatency = (
  client: MqttClient | null,
  subscribe: MqttSubscribe,
  publish: MqttPublish,
) => {
  const [latency, setLatency] = useState<number>();

  // send latency messages on interval
  useEffect(() => {
    const interval = setInterval(() => {
      publish({
        topic: 'latency',
        payload: new Date().getTime().toString(),
      });
    }, 100);
    return () => clearInterval(interval);
  }, []);

  // subscribe to latency messages and calculate latency
  useEffect(() => {
    if (!client) return;
    subscribe({ topic: 'latency' });
    client.on('message', (topic, message) => {
      if (topic === 'latency') {
        const latency = new Date().getTime() - parseInt(message.toString());
        setLatency(latency);
      }
    });
  }, [client]);

  return { latency };
};
