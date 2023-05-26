import type { Pods } from '@hyped/telemetry-types';

export const POD_IDS = ['1', '2'] as const;

export const pods: Pods = {
  '1': {
    name: 'Pod 1',
    id: 1,
    measurements: {
      'temperature.shell_front': {
        name: 'Temperature - Shell Front',
        key: 'temperature.shell_front',
        format: 'float',
        type: 'temperature',
        unit: '°C',
        range: {
          min: -20,
          max: 50,
        },
      },
      'temperature.shell_middle': {
        name: 'Temperature - Shell Middle',
        key: 'temperature.shell_middle',
        format: 'float',
        type: 'temperature',
        unit: '°C',
        range: {
          min: -20,
          max: 50,
        },
      },
      'temperature.shell_back': {
        name: 'Temperature - Shell Back',
        key: 'temperature.shell_back',
        format: 'float',
        type: 'temperature',
        unit: '°C',
        range: {
          min: -20,
          max: 50,
        },
      },
      accelerometer: {
        name: 'Accelerometer',
        key: 'accelerometer',
        format: 'float',
        type: 'acceleration',
        unit: 'ms^-2',
        range: {
          min: -10,
          max: 10,
        },
      },
      keyence: {
        name: 'Keyence',
        key: 'keyence',
        format: 'integer',
        type: 'keyence',
        unit: 'number of stripes',
        range: {
          min: 0,
          max: 100,
        },
      },
      brake_feedback: {
        name: 'Brake Feedback',
        key: 'brake_feedback',
        format: 'enum',
        type: 'brake_feedback',
        unit: 'brakes engaged',
        enumerations: [
          {
            key: 'ON',
            value: '1',
          },
          {
            key: 'OFF',
            value: '0',
          },
        ],
      },
    },
  },
  '2': {
    name: 'Pod 2',
    id: 2,
    measurements: {
      'temperature.shell_front': {
        name: 'Temperature - Shell Front',
        key: 'temperature.shell_front',
        format: 'float',
        type: 'temperature',
        unit: '°C',
        range: {
          min: -20,
          max: 50,
        },
      },
      'temperature.shell_middle': {
        name: 'Temperature - Shell Middle',
        key: 'temperature.shell_middle',
        format: 'float',
        type: 'temperature',
        unit: '°C',
        range: {
          min: -20,
          max: 50,
        },
      },
      'temperature.shell_back': {
        name: 'Temperature - Shell Back',
        key: 'temperature.shell_back',
        format: 'float',
        type: 'temperature',
        unit: '°C',
        range: {
          min: -20,
          max: 50,
        },
      },
      accelerometer: {
        name: 'Accelerometer',
        key: 'accelerometer',
        format: 'float',
        type: 'acceleration',
        unit: 'ms^-2',
        range: {
          min: -10,
          max: 10,
        },
      },
      keyence: {
        name: 'Keyence',
        key: 'keyence',
        format: 'integer',
        type: 'keyence',
        unit: 'number of stripes',
        range: {
          min: 0,
          max: 100,
        },
      },
      brake_feedback: {
        name: 'Brake Feedback',
        key: 'brake_feedback',
        format: 'enum',
        type: 'brake_feedback',
        unit: 'brakes engaged',
        enumerations: [
          {
            key: 'ON',
            value: '1',
          },
          {
            key: 'OFF',
            value: '0',
          },
        ],
      },
    },
  },
};
