import { MqttModuleOptions } from 'nest-mqtt';
import { MQTT_BROKER_HOST } from 'src/modules/core/config';

export const mqttClientOptions: MqttModuleOptions = {
  host: MQTT_BROKER_HOST,
};
