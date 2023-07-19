// create a contet for mqtt

import { createContext, useContext, useEffect, useState } from 'react';
import { IClientOptions, MqttClient } from 'mqtt/types/lib/client';
import { MqttPublish, MqttSubscribe, QoS } from '@hyped/telemetry-types';
import { StatusType } from '@/types/StatusType';
import mqtt from 'mqtt/dist/mqtt';
import { MqttUnsubscribe } from '@hyped/telemetry-types';

const MQTT_BROKER = 'ws://localhost:8080';

type MQTTContextType = {
  client: MqttClient | null;
  latency: number | undefined;
  publish: MqttPublish;
  subscribe: MqttSubscribe;
  unsubscribe: MqttUnsubscribe;
  connectionStatus: StatusType;
};

const MQTTContext = createContext<MQTTContextType | null>(null);

export const MQTTProvider = ({ children }: { children: React.ReactNode }) => {
  const [client, setClient] = useState<MqttClient | null>(null);

  const [connectionStatus, setConnectionStatus] =
    useState<StatusType>('waiting');

  const mqttConnect = (host: string, mqttOption?: IClientOptions) => {
    console.log('Connecting to MQTT broker: ', host);
    setConnectionStatus('connecting');
    const mqttClient = mqtt.connect(host, mqttOption);
    console.log('MQTT client: ', mqttClient);
    setClient(mqttClient);
  };

  // Connect to MQTT broker on mount
  useEffect(() => {
    mqttConnect(MQTT_BROKER);
  }, []);

  // Handle client changes
  useEffect(() => {
    console.log('CLIENT CHANGED: ', client);
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
      console.log("Client doesn't exist, reconnecting");
      mqttConnect(MQTT_BROKER);
    }
  }, [client]);

  /**
   * Publish an MQTT message
   */
  const publish = ({
    topic,
    qos = 0,
    payload,
    podId = 'pod_1',
  }: {
    topic: string;
    qos?: QoS;
    payload: string;
    podId?: string;
  }) => {
    const fullTopic = getFullTopic(topic, podId);
    if (!client) {
      console.log(`Couldn't publish to ${fullTopic} because client is null`);
      return;
    }
    client.publish(fullTopic, payload, { qos }, (error) => {
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
    const fullTopic = getFullTopic(topic, podId);
    if (!client) {
      console.log(`Couldn't subscribe to ${fullTopic} because client is null`);
      return;
    }
    client.subscribe(fullTopic, { qos });
  };

  const unsubscribe = (topic: string, podId: string) => {
    const fullTopic = getFullTopic(topic, podId);
    if (!client) {
      console.log(
        `Couldn't unsubscribe from ${fullTopic} because client is null`,
      );
      return;
    }
    client.unsubscribe(fullTopic);
  };

  const [latency, setLatency] = useState<number>();
  console.log(client);

  // send latency messages on interval
  useEffect(() => {
    const interval = setInterval(() => {
      console.log('Sending latency request');
      publish({
        topic: 'latency/request',
        qos: 0,
        payload: 'pls bro',
      });
    }, 1000);
    return () => clearInterval(interval);
  }, []);

  // subscribe to latency messages and calculate latency
  useEffect(() => {
    if (!client) return;
    subscribe({ topic: 'latency/response', qos: 0 });
    const getLatency = (topic: string, message: Buffer) => {
      if (topic === 'latency/response') {
        const latency = new Date().getTime() - parseInt(message.toString());
        setLatency(latency);
      }
    };
    client.on('message', getLatency);
    return () => {
      client.off('message', getLatency);
      unsubscribe('latency/response', 'pod_1');
    };
  }, [client]);

  return (
    <MQTTContext.Provider
      value={{
        client,
        publish,
        subscribe,
        unsubscribe,
        latency,
        connectionStatus,
      }}
    >
      {children}
    </MQTTContext.Provider>
  );
};

export const useMQTT = () => {
  const context = useContext(MQTTContext);
  if (!context) {
    throw new Error('useMQTT must be used within MQTTProvider');
  }
  return context;
};

const getFullTopic = (topic: string, podId: string) => {
  return `hyped/${podId}/${topic}`;
};
