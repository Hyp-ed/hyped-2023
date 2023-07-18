import { PodState, podStates } from '@hyped/telemetry-constants';
import { MqttSubscribe } from '@hyped/telemetry-types';
import { MqttClient } from 'mqtt/types/lib/client';
import { useEffect, useState } from 'react';

export const usePodState = (
  client: MqttClient | null,
  subscribe: MqttSubscribe,
  podId: string,
) => {
  const [podState, setPodState] = useState<PodState>(podStates.UNKNOWN);

  useEffect(() => {
    subscribe({
      topic: `state`,
    });
    if (!client) return;
    client.on('message', (topic, message) => {
      if (topic === `hyped/${podId}/state`) {
        const newPodState = message.toString();
        const allowedStates = Object.values(podStates);
        if (allowedStates.includes(newPodState as PodState)) {
          setPodState(newPodState as PodState);
        }
      }
    });
  }, [client]);

  return { podState };
};
