import {
  POD_CONNECTION_STATUS,
  PodConnectionStatusType,
} from '@/types/PodConnectionStatus';
import { createContext, useContext, useEffect, useState } from 'react';
import { useMQTT } from './mqtt';
import { MQTT_CONNECTION_STATUS } from '@/types/MQTTConnectionStatus';
import { getTopic } from '@/lib/utils';

/**
 * The maximum latency before a pod is considered disconnected, in milliseconds
 */
const POD_MAX_LATENCY = 300;

/**
 * The interval between latency messages, in milliseconds
 */
const LATENCY_REQUEST_INTERVAL = 100;

type PodsContextType = {
  [podId: string]: {
    connectionStatus: PodConnectionStatusType;
    previousLatencies?: number[];
    latency?: number;
  };
};

const PodsContext = createContext<PodsContextType | null>(null);

function createPodsContextFromIds(podIds: string[]): PodsContextType {
  const podsContext: PodsContextType = {};
  for (const podId of podIds) {
    podsContext[podId] = {
      connectionStatus: POD_CONNECTION_STATUS.UNKNOWN,
    };
  }
  return podsContext;
}

/**
 * MQTT Context Provider.
 * Provides an MQTT client and functions to publish, subscribe and unsubscribe to MQTT topics.
 * Also provides the connection status of the MQTT client.
 */
export const PodsProvider = ({
  podIds,
  children,
}: {
  podIds: string[];
  children: React.ReactNode;
}) => {
  const [podsState, setPodsState] = useState<PodsContextType>(
    createPodsContextFromIds(podIds),
  );
  const [lastLatencyResponse, setLastLatencyResponse] = useState<number>();

  const { client, publish, subscribe, unsubscribe, mqttConnectionStatus } =
    useMQTT();

  useEffect(() => {
    // If we don't have an MQTT connection, set all pod connection statuses to disconnected
    if (mqttConnectionStatus !== MQTT_CONNECTION_STATUS.CONNECTED) {
      setPodsState((prevState) =>
        Object.fromEntries(
          Object.entries(prevState).map(([podId]) => [
            podId,
            {
              connectionStatus: POD_CONNECTION_STATUS.DISCONNECTED,
            },
          ]),
        ),
      );
    }
  }, [mqttConnectionStatus]);

  useEffect(() => {
    // send latency messages every LATENCY_INTERVAL milliseconds
    const interval = setInterval(() => {
      podIds.map((podId) => {
        console.log(`Sending latency request to ${podId}`);
        publish(
          'latency/request',
          JSON.stringify({
            latency: new Date().getTime().toString(),
          }),
          podId,
        );
      });
    }, LATENCY_REQUEST_INTERVAL);
    return () => clearInterval(interval);
  }, [client]);

  useEffect(() => {
    // check if we have received a latency response within the last POD_MAX_LATENCY milliseconds
    const interval = setTimeout(() => {
      podIds.map((podId) => {
        if (!lastLatencyResponse) return;
        if (new Date().getTime() - lastLatencyResponse > POD_MAX_LATENCY) {
          setPodsState((prevState) => ({
            ...prevState,
            [podId]: {
              ...prevState[podId],
              connectionStatus: POD_CONNECTION_STATUS.DISCONNECTED,
            },
          }));
        }
      });
    }, POD_MAX_LATENCY);
    return () => clearInterval(interval);
  }, [lastLatencyResponse]);

  // subscribe to latency messages and calculate latency
  useEffect(() => {
    if (!client) return;

    const getLatency = (podId: string, topic: string, message: Buffer) => {
      if (topic !== getTopic('latency/response', podId)) return;
      const latency =
        new Date().getTime() -
        parseInt(JSON.parse(message.toString())['latency']);
      console.log(`Received latency response from ${podId}: ${latency}ms`);
      setLastLatencyResponse(new Date().getTime());
      setPodsState((prevState) => ({
        ...prevState,
        [podId]: {
          connectionStatus: POD_CONNECTION_STATUS.CONNECTED,
          // maintain a list of the previous 10 latencies
          previousLatencies: [
            ...(prevState[podId].previousLatencies || []).slice(-10),
            latency,
          ],
          // calculate the average latency
          latency: Math.round(
            (prevState[podId].previousLatencies?.reduce((a, b) => a + b, 0) ||
              0) / (prevState[podId].previousLatencies?.length || 1),
          ),
        },
      }));
    };

    podIds.map((podId) => {
      subscribe('latency/response', podId);
      client.on('message', (topic, message) =>
        getLatency(podId, topic, message),
      );
    });

    return () => {
      podIds.map((podId) => {
        client.off('message', (topic, message) =>
          getLatency(podId, topic, message),
        );
        unsubscribe('latency/response', podId);
      });
    };
  }, [client]);

  return (
    <PodsContext.Provider value={podsState}>{children}</PodsContext.Provider>
  );
};

export const usePods = (podId: string) => {
  const context = useContext(PodsContext);
  if (!context) {
    throw new Error('usePods must be used within PodsProvider');
  }
  // Get the pod connection status for the given podId
  return context[podId];
};
