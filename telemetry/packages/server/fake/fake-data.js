/* eslint-disable no-console */
import mqtt from 'mqtt';
import random from 'random';

const SENSORS = await require('./fake-sensors.json');

const USE_PUBLIC_BROKER = false;

const client = mqtt.connect(
  USE_PUBLIC_BROKER ? 'ws://broker.emqx.io:8083/mqtt' : 'ws://localhost:9001',
);

function generateValueForSensor(sensor) {
  if (sensor.type === 'boolean') {
    if (sensor.default === undefined) {
      throw new Error('Sensor of type "boolean" must have a default value');
    }
    return String(sensor.default);
  }

  if (sensor.type === 'float') {
    if (sensor.min === undefined || sensor.max === undefined) {
      throw new Error('Sensor of type "float" must have a min and max value');
    }
    return String(random.float(sensor.min, sensor.max).toFixed(2));
  }

  if (sensor.type === 'integer') {
    if (sensor.min === undefined || sensor.max === undefined) {
      throw new Error('Sensor of type "integer" must have a min and max value');
    }
    return String(random.int(sensor.min, sensor.max));
  }

  throw new Error('Sensor type not supported');
}

client.on('connect', () => {
  console.log('CLIENT CONNECTED');

  SENSORS.forEach((sensor) => {
    setInterval(() => {
      const value = generateValueForSensor(sensor);
      client.publish(`hyped.${sensor.name}`, value, (err) => {
        if (err) {
          console.error('ERROR PUBLISHING', err);
        } else {
          console.log(`Published ${sensor.name} = ${value}`);
        }
      });
    }, sensor.update_interval);
  });
});
