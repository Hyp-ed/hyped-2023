/* eslint-disable no-console */
import mqtt from 'mqtt';
import random from 'random';
import pods from '@hyped/telemetry-constants/sensors.json';
import { Sensor } from 'types/sensor';

const client = mqtt.connect('ws://localhost:9001');

const SENSOR_UPDATE_INTERVAL = 1000;

/**
 * Generates a value for a sensor based on it's previous value.
 * Uses the sensor's type to generate more sensible test data.
 * @param previousValue The previous value of the sensor
 * @param sensor The sensor to generate the value for
 * @returns A value for the sensor
 */
const generateValueForSensor = (previousValue: string, sensor: Sensor) => {
  // Use the sensor's type (e.g. acceleration) to generate a sensible value
  switch (sensor.type) {
    case 'temperature':
      if (sensor.range === undefined) {
        throw new Error('Temperature sensor must have a min and max value');
      }
      // Generate a value that is within the sensor's range, and is within 2 degrees of the previous value
      const proposedValue = parseFloat(previousValue) + random.float(-2, 2);
      if (proposedValue < sensor.range.min) {
        return String(sensor.range.min);
      }
      if (proposedValue > sensor.range.max) {
        return String(sensor.range.max);
      }
      return String(proposedValue);
    case 'acceleration':
      if (sensor.range === undefined) {
        throw new Error('Acceleration sensor must have a min and max value');
      }
      return String(0);
    // case 'keyence':
    //   if (sensor.range === undefined) {
    //     throw new Error('Keyence sensor must have a min and max value');
    //   }
    //   const proposedValue = parseFloat(previousValue) + random.;
    //   return String();
    case 'brake_feedback':
      if (sensor.enumerations === undefined) {
        throw new Error('Brake feedback sensor must have enumerations');
      }
      return String(sensor.enumerations[0].value);
    default:
      // Fall back to using the sensor's format (e.g. float) if the type is not specified above
      return generateValueByFormat(sensor);
  }
};

/**
 * Generates the value for a sensor based on it's format (e.g. float), within the sensor's range.
 * @param sensor The sensor to generate the value for
 * @returns A value for the sensor
 */
const generateValueByFormat = (sensor: Sensor) => {
  switch (sensor.format) {
    case 'float':
      if (sensor.range === undefined) {
        throw new Error('Sensor of type "float" must have a min and max value');
      }
      return String(
        random.float(sensor.range.min, sensor.range.max).toFixed(2),
      );
    case 'integer':
      if (sensor.range === undefined) {
        throw new Error(
          'Sensor of type "integer" must have a min and max value',
        );
      }
      return String(random.int(sensor.range.min, sensor.range.max));
    default:
      throw new Error('Sensor format not supported');
  }
};

const initialValues: {
  [key: string]: string;
} = {
  'temperature.shell_front': '40',
  'temperature.shell_middle': '45',
  'temperature.shell_back': '35',
  accelerometer: '0',
  keyence: '0',
  brake_feedback: '1',
};

client.on('connect', () => {
  console.log('CLIENT CONNECTED');

  Object.entries(pods).forEach(([, pod]) => {
    Object.entries(pod.measurements).forEach(([, sensor]) => {
      const previousValues = initialValues;
      setInterval(() => {
        const previousValue =
          initialValues[sensor.key] || generateValueByFormat(sensor);
        const newValue = generateValueForSensor(previousValue, sensor);
        previousValues[sensor.key] = newValue;
        client.publish(`hyped/${pod.key}/${sensor.key}`, newValue, (err) => {
          if (err) {
            console.error('ERROR PUBLISHING', err);
          } else {
            console.log(`Published ${sensor.name} = ${newValue}`);
          }
        });
      }, SENSOR_UPDATE_INTERVAL);
    });
  });
});
