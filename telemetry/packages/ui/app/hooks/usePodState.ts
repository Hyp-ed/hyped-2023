import { useMQTT } from '@/context/mqtt';
import { getTopic } from '@/lib/utils';
import { PodStateType, podStates } from '@hyped/telemetry-constants';
import { useEffect, useState } from 'react';

export const usePodState = (podId: string) => {
  const { client, subscribe, unsubscribe } = useMQTT();
  const [podState, setPodState] = useState<PodStateType>(podStates.UNKNOWN);

  useEffect(() => {
    if (!client) return;
    subscribe('state', podId);
    const getPodState = (topic: string, message: Buffer) => {
      if (topic === getTopic('state', podId)) {
        const newPodState = message.toString();
        const allowedStates = Object.values(podStates);
        if (allowedStates.includes(newPodState as PodStateType)) {
          setPodState(newPodState as PodStateType);
        }
      }
    };
    client.on('message', getPodState);
    return () => {
      client.off('message', getPodState);
      unsubscribe('state', podId);
    };
  }, [client]);

  return { podState };
};
