import { useEffect, useState } from 'react';
import mqtt from 'mqtt/dist/mqtt';
import {
  ClientSubscribeCallback,
  IClientOptions,
  MqttClient,
} from 'mqtt/types/lib/client';
import { StatusType } from '@/types/StatusType';
import { QoS } from '@hyped/telemetry-types';

const MQTT_BROKER = 'ws://localhost:8080';

export const useMQTT = () => {
  const latency = 11; // TODOLater: replace with real value once latency is implemented

  const [client, setClient] = useState<MqttClient | null>(null);

  const [connectionStatus, setConnectionStatus] =
    useState<StatusType>('waiting');

  // Connect to MQTT broker on mount
  useEffect(() => {
    const mqttConnect = (host: string, mqttOption?: IClientOptions) => {
      setConnectionStatus('connecting');
      setClient(mqtt.connect(host, mqttOption));
    };
    mqttConnect(MQTT_BROKER);
  }, []);

  // Handle client changes
  useEffect(() => {
    if (client) {
      client.on('connect', () => {
        setConnectionStatus('connected');
      });
      client.on('error', (err: any) => {
        console.error('Connection error: ', err);
        setConnectionStatus('error');
        client.end();
      });
      client.on('reconnect', () => {
        setConnectionStatus('reconnecting');
      });
    } else {
      setConnectionStatus('disconnected');
    }
  }, [client]);

  /**
   * Publish an MQTT message
   */
  const publish = ({
    topic,
    qos,
    payload,
    podId = 'pod_1',
  }: {
    topic: string;
    qos?: QoS;
    payload: string;
    podId?: string;
  }) => {
    if (!client) return;
    // if (!client) throw new Error('MQTT client not connected');
    client.publish(`hyped/${podId}/${topic}`, payload, { qos }, (error) => {
      if (error) {
        console.error('Publish error: ', error);
      }
    });
  };

  /**
   * Subscribe to an MQTT topic
   */
  const subscribe = ({
    topic,
    qos = 0,
    podId = 'pod_1',
  }: {
    topic: string;
    qos?: QoS;
    podId?: string;
  }) => {
    if (!client) return;
    // if (!client) throw new Error('MQTT client not connected');
    client.subscribe(`hyped/${podId}/${topic}`, { qos });
  };

  return { connectionStatus, publish, latency, subscribe, client };
};
