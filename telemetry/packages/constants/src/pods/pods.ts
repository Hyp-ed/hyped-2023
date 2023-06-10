import type { Pods } from '@hyped/telemetry-types';

export const POD_IDS = ['pod_1'] as const;

const MIN_PRESSURE = 3.26;
const MAX_PRESSURE = 4.6;

export const pods: Pods = {
  pod_1: {
    id: 'pod_1',
    name: 'Pod Ness',
    measurements: {
      pressure_1: {
        name: 'Pressure 1',
        key: 'pressure_1',
        format: 'float',
        type: 'pressure',
        unit: 'bar',
        range: {
          min: MIN_PRESSURE,
          max: MAX_PRESSURE,
        },
      },
      pressure_2: {
        name: 'Pressure 2',
        key: 'pressure_2',
        format: 'float',
        type: 'pressure',
        unit: 'bar',
        range: {
          min: MIN_PRESSURE,
          max: MAX_PRESSURE,
        },
      },
      pressure_3: {
        name: 'Pressure 3',
        key: 'pressure_3',
        format: 'float',
        type: 'pressure',
        unit: 'bar',
        range: {
          min: MIN_PRESSURE,
          max: MAX_PRESSURE,
        },
      },
      pressure_4: {
        name: 'Pressure 4',
        key: 'pressure_4',
        format: 'float',
        type: 'pressure',
        unit: 'bar',
        range: {
          min: MIN_PRESSURE,
          max: MAX_PRESSURE,
        },
      },
      // pressure_5: {
      //   name: 'Pressure 5',
      //   key: 'pressure_5',
      //   format: 'float',
      //   type: 'pressure',
      //   unit: 'bar',
      //   range: {
      //     min: MIN_PRESSURE,
      //     max: MAX_PRESSURE,
      //   },
      // },
      // pressure_6: {
      //   name: 'Pressure 6',
      //   key: 'pressure_6',
      //   format: 'float',
      //   type: 'pressure',
      //   unit: 'bar',
      //   range: {
      //     min: MIN_PRESSURE,
      //     max: MAX_PRESSURE,
      //   },
      // },
      // pressure_7: {
      //   name: 'Pressure 7',
      //   key: 'pressure_7',
      //   format: 'float',
      //   type: 'pressure',
      //   unit: 'bar',
      //   range: {
      //     min: MIN_PRESSURE,
      //     max: MAX_PRESSURE,
      //   },
      // },
      // pressure_8: {
      //   name: 'Pressure 8',
      //   key: 'pressure_8',
      //   format: 'float',
      //   type: 'pressure',
      //   unit: 'bar',
      //   range: {
      //     min: MIN_PRESSURE,
      //     max: MAX_PRESSURE,
      //   },
      // },
    },
  },
};
//     'temperature.cell': {
//       name: 'Temperature - Cell',
//       key: 'temperature.cell',
//       format: 'float',
//       type: 'temperature',
//       unit: '°C',
//       range: {
//         min: -20,
//         max: 60,
//       },
//     },
//     current: {
//       name: 'Current',
//       key: 'current',
//       format: 'float',
//       type: 'current',
//       unit: 'A',
//       range: {
//         min: 0,
//         max: 500,
//       },
//     },
//     'voltage.cell': {
//       name: 'Voltage - Cell',
//       key: 'voltage.cell',
//       format: 'float',
//       type: 'voltage',
//       unit: 'V',
//       range: {
//         min: 3,
//         max: 4.2,
//       },
//     },
//     // imd: {
//     //   name: 'Insulation Monitoring Device',
//     //   key: 'imd',
//     //   format: 'enum',
//     //   type: 'imd',
//     //   unit: 'state',
//     //   enumerations: [
//     //     {
//     //       value: 1,
//     //       string: 'TRUE',
//     //     },
//     //     {
//     //       value: 0,
//     //       string: 'FALSE',
//     //     },
//     //   ],
//     // },
//     'pressure.brakes_tank': {
//       name: 'Pressure - Brakes Tank',
//       key: 'pressure.brakes_tank',
//       format: 'float',
//       type: 'pressure',
//       unit: 'bar',
//       range: {
//         min: 0, // TEMP
//         max: 10, // TEMP
//       },
//     },
//     'pressure.brakes_front': {
//       name: 'Pressure - Brakes Front',
//       key: 'pressure.brakes_front',
//       format: 'float',
//       type: 'pressure',
//       unit: 'bar',
//       range: {
//         min: 0, // TEMP
//         max: 10, // TEMP
//       },
//     },
//     'pressure.brakes_rear': {
//       name: 'Pressure - Brakes Rear',
//       key: 'pressure.brakes_rear',
//       format: 'float',
//       type: 'pressure',
//       unit: 'bar',
//       range: {
//         min: 0, // TEMP
//         max: 10, // TEMP
//       },
//     },
//     'pressure.brakes': {
//       name: 'Pressure - Brakes',
//       key: 'pressure.brakes',
//       format: 'float',
//       type: 'pressure',
//       unit: 'bar',
//       range: {
//         min: 0, // TEMP
//         max: 10, // TEMP
//       },
//     },
//     'temperature.brakes_forward_left_pcb': {
//       name: 'Temperature - Brakes Forward Left PCB',
//       key: 'temperature.brakes_forward_left_pcb',
//       format: 'float',
//       type: 'temperature',
//       unit: '°C',
//       range: {
//         min: 0,
//         max: 100,
//       },
//     },
//     'temperature.brakes_forward_right_pcb': {
//       name: 'Temperature - Brakes Forward Right PCB',
//       key: 'temperature.brakes_forward_right_pcb',
//       format: 'float',
//       type: 'temperature',
//       unit: '°C',
//       range: {
//         min: 0,
//         max: 100,
//       },
//     },
//     'temperature.brakes_rear_left_pcb': {
//       name: 'Temperature - Brakes Rear Left PCB',
//       key: 'temperature.brakes_rear_left_pcb',
//       format: 'float',
//       type: 'temperature',
//       unit: '°C',
//       range: {
//         min: 0,
//         max: 100,
//       },
//     },
//     'temperature.brakes_rear_right_pcb': {
//       name: 'Temperature - Brakes Rear Right PCB',
//       key: 'temperature.brakes_rear_right_pcb',
//       format: 'float',
//       type: 'temperature',
//       unit: '°C',
//       range: {
//         min: 0,
//         max: 100,
//       },
//     },
//     'pressure.vertical_suspension': {
//       name: 'Pressure - Vertical Suspension',
//       key: 'pressure.vertical_suspension',
//       format: 'float',
//       type: 'pressure',
//       unit: 'bar',
//       range: {
//         min: 0, // TEMP
//         max: 10, // TEMP
//       },
//     }
//   },
// },
// pod_2: {
//   id: 'pod_2',
//   name: 'Pod 2',
//   measurements: {
//     'temperature.shell_front': {
//       name: 'Temperature - Shell Front',
//       key: 'temperature.shell_front',
//       format: 'float',
//       type: 'temperature',
//       unit: '°C',
//       range: {
//         min: -20,
//         max: 50,
//       },
//     },
//     'temperature.shell_middle': {
//       name: 'Temperature - Shell Middle',
//       key: 'temperature.shell_middle',
//       format: 'float',
//       type: 'temperature',
//       unit: '°C',
//       range: {
//         min: -20,
//         max: 50,
//       },
//     },
//     'temperature.shell_back': {
//       name: 'Temperature - Shell Back',
//       key: 'temperature.shell_back',
//       format: 'float',
//       type: 'temperature',
//       unit: '°C',
//       range: {
//         min: -20,
//         max: 50,
//       },
//     },
//     accelerometer: {
//       name: 'Accelerometer',
//       key: 'accelerometer',
//       format: 'float',
//       type: 'acceleration',
//       unit: 'ms^-2',
//       range: {
//         min: -10,
//         max: 10,
//       },
//     },
//     keyence: {
//       name: 'Keyence',
//       key: 'keyence',
//       format: 'integer',
//       type: 'keyence',
//       unit: 'number of stripes',
//       range: {
//         min: 0,
//         max: 100,
//       },
//     },
//     brake_feedback: {
//       name: 'Brake Feedback',
//       key: 'brake_feedback',
//       format: 'enum',
//       type: 'brake_feedback',
//       unit: 'brakes engaged',
//       enumerations: [
//         {
//           value: 1,
//           string: 'ON',
//         },
//         {
//           value: 0,
//           string: 'OFF',
//         },
//       ],
//     },
//   },
// },
// };
