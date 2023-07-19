import { ClientSubscribeCallback } from 'mqtt/types/lib/client';

export type QoS = 0 | 1 | 2;

export type MqttPublish = ({
  topic,
  qos,
  payload,
}: {
  topic: string;
  qos?: QoS;
  payload: string;
  podId?: string;
}) => void;

export type MqttSubscribe = ({
  topic,
  qos = 0,
  podId = 'pod_1',
}: {
  topic: string;
  qos?: QoS;
  podId?: string;
}) => void;

export type MqttUnsubscribe = (topic: string, podId: string) => void;
