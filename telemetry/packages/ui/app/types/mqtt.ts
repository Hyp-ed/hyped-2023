import mqtt from 'mqtt/dist/mqtt';

export type MqttPublish = ({
  topic,
  qos,
  payload,
}: {
  topic: string;
  qos: mqtt.QoS | undefined;
  payload: string;
}) => void;
