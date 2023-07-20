import { useMQTT } from '@/context/mqtt';
import { useState, useEffect } from 'react';

/**
 * Hook to get the latency between the base station and a pod in milliseconds
 * @param podId The podId of the pod to get latency for
 * @returns The latency between the base station and the pod in milliseconds
 */
export const useLatency = (podId: string) => {
  const { client, publish, subscribe, unsubscribe } = useMQTT();

  const [latency, setLatency] = useState<number>();

  // send latency messages on interval
  useEffect(() => {
    const interval = setInterval(() => {
      console.log('Sending latency request');
      publish(
        'latency/request',
        JSON.stringify({
          latency: new Date().getTime().toString(),
        }),
        podId,
      );
    }, 100);
    return () => clearInterval(interval);
  }, [client]);

  // subscribe to latency messages and calculate latency
  useEffect(() => {
    if (!client) return;
    subscribe('latency/response', podId);
    const getLatency = (topic: string, message: Buffer) => {
      if (topic === 'latency/response') {
        const latency = new Date().getTime() - parseInt(message.toString());
        setLatency(latency);
      }
    };
    client.on('message', getLatency);
    return () => {
      client.off('message', getLatency);
      unsubscribe('latency/response', podId);
    };
  }, [client]);

  return { latency };
};
