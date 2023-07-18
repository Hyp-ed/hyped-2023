import type { Pods } from '@hyped/telemetry-types';

export const POD_IDS = ['pod_1'] as const;

// ************************************ COMMON ************************************ //
const accelerometerCommon = {
  format: 'float',
  type: 'acceleration',
  unit: 'ms^-2',
  limits: {
    critical: {
      low: -150,
      high: 150,
    },
  },
} as const;

const thermistorCommon = {
  format: 'float',
  type: 'thermistor',
  unit: '°C',
  limits: {
    critical: {
      low: 0,
      high: 120,
    },
  },
} as const;

const pressureCommon = {
  format: 'float',
  type: 'pressure',
  unit: 'bar',
  limits: {
    critical: {
      low: 3.26,
      high: 4.6,
    },
  },
} as const;

const hallEffectCommon = {
  format: 'float',
  type: 'hall_effect',
  unit: 'A',
  limits: {
    critical: {
      low: 0,
      high: 500,
    },
  },
} as const;

// ************************************ MEASUREMENTS ************************************ //
export const pods: Pods = {
  pod_1: {
    id: 'pod_1',
    name: 'Pod Ness',
    measurements: {
      accelerometer_1: {
        name: 'Accelerometer 1',
        key: 'accelerometer_1',
        ...accelerometerCommon,
      },
      accelerometer_2: {
        name: 'Accelerometer 2',
        key: 'accelerometer_2',
        ...accelerometerCommon,
      },
      accelerometer_3: {
        name: 'Accelerometer 3',
        key: 'accelerometer_3',
        ...accelerometerCommon,
      },
      accelerometer_4: {
        name: 'Accelerometer 4',
        key: 'accelerometer_4',
        ...accelerometerCommon,
      },
      accelerometer_avg: {
        name: 'Accelerometer Average',
        key: 'accelerometer_avg',
        ...accelerometerCommon,
      },
      displacement: {
        name: 'Displacement',
        key: 'displacement',
        format: 'float',
        type: 'displacement',
        unit: 'm',
        limits: {
          critical: {
            low: 0,
            high: 10000000000000,
          },
        },
      },
      velocity: {
        name: 'Velocity',
        key: 'velocity',
        format: 'float',
        type: 'velocity',
        unit: 'm/s',
        limits: {
          critical: {
            low: 0,
            high: 10000000000000,
          },
        },
      },
      acceleration: {
        name: 'Acceleration',
        key: 'acceleration',
        format: 'float',
        type: 'acceleration',
        unit: 'm/s^2',
        limits: {
          critical: {
            low: 0,
            high: 10000000000000,
          },
        },
      },
      pressure_1: {
        name: 'Pressure 1',
        key: 'pressure_1',
        ...pressureCommon,
      },
      pressure_2: {
        name: 'Pressure 2',
        key: 'pressure_2',
        ...pressureCommon,
      },
      pressure_3: {
        name: 'Pressure 3',
        key: 'pressure_3',
        ...pressureCommon,
      },
      pressure_4: {
        name: 'Pressure 4',
        key: 'pressure_4',
        ...pressureCommon,
      },
      pressure_5: {
        name: 'Pressure 5',
        key: 'pressure_5',
        ...pressureCommon,
      },
      pressure_6: {
        name: 'Pressure 6',
        key: 'pressure_6',
        ...pressureCommon,
      },
      pressure_7: {
        name: 'Pressure 7',
        key: 'pressure_7',
        ...pressureCommon,
      },
      pressure_8: {
        name: 'Pressure 8',
        key: 'pressure_8',
        ...pressureCommon,
      },
      hall_effect_1: {
        name: 'Hall Effect 1',
        key: 'hall_effect_1',
        ...hallEffectCommon,
      },
      hall_effect_2: {
        name: 'Hall Effect 2',
        key: 'hall_effect_2',
        ...hallEffectCommon,
      },
      thermistor_1: {
        name: 'Thermistor 1',
        key: 'thermistor_1',
        ...thermistorCommon,
      },
      thermistor_2: {
        name: 'Thermistor 2',
        key: 'thermistor_2',
        ...thermistorCommon,
      },
      thermistor_3: {
        name: 'Thermistor 3',
        key: 'thermistor_3',
        ...thermistorCommon,
      },
      thermistor_4: {
        name: 'Thermistor 4',
        key: 'thermistor_4',
        ...thermistorCommon,
      },
      thermistor_5: {
        name: 'Thermistor 5',
        key: 'thermistor_5',
        ...thermistorCommon,
      },
      thermistor_6: {
        name: 'Thermistor 6',
        key: 'thermistor_6',
        ...thermistorCommon,
      },
      thermistor_7: {
        name: 'Thermistor 7',
        key: 'thermistor_7',
        ...thermistorCommon,
      },
      thermistor_8: {
        name: 'Thermistor 8',
        key: 'thermistor_8',
        ...thermistorCommon,
      },
      thermistor_9: {
        name: 'Thermistor 9',
        key: 'thermistor_9',
        ...thermistorCommon,
      },
      thermistor_10: {
        name: 'Thermistor 10',
        key: 'thermistor_10',
        ...thermistorCommon,
      },
      thermistor_11: {
        name: 'Thermistor 11',
        key: 'thermistor_11',
        ...thermistorCommon,
      },
      thermistor_12: {
        name: 'Thermistor 12',
        key: 'thermistor_12',
        ...thermistorCommon,
      },
      thermistor_13: {
        name: 'Thermistor 13',
        key: 'thermistor_13',
        ...thermistorCommon,
      },
      thermistor_14: {
        name: 'Thermistor 14',
        key: 'thermistor_14',
        ...thermistorCommon,
      },
      thermistor_15: {
        name: 'Thermistor 15',
        key: 'thermistor_15',
        ...thermistorCommon,
      },
      thermistor_16: {
        name: 'Thermistor 16',
        key: 'thermistor_16',
        ...thermistorCommon,
      },
      brake_clamp_status: {
        name: 'Brake Clamp Status',
        key: 'brake_clamp_status',
        format: 'enum',
        type: 'status',
        unit: 'state',
        enumerations: [
          {
            value: 1,
            string: 'ON',
          },
          {
            value: 0,
            string: 'OFF',
          },
        ],
      },
      pod_raised_status: {
        name: 'Pod Raised Status',
        key: 'pod_raised_status',
        format: 'enum',
        type: 'status',
        unit: 'state',
        enumerations: [
          {
            value: 1,
            string: 'ON',
          },
          {
            value: 0,
            string: 'OFF',
          },
        ],
      },
    },
  },
};
