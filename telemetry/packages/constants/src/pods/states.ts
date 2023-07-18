export type PodState = keyof typeof podStates;

export const failureStates = {
  FAILURE_BRAKING: 'FAILURE_BRAKING',
  FAILURE_STOPPED: 'FAILURE_STOPPED',
  FAILURE_CALIBRATING: 'FAILURE_CALIBRATING',
} as const;

export const staticStates = {
  IDLE: 'IDLE',
  CALIBRATING: 'CALIBRATING',
  READY: 'READY',
  STOPPED: 'STOPPED',
  OFF: 'OFF',
} as const;

export const okayStates = {
  ACCELERATING: 'ACCELERATING',
  NOMINAL_BRAKING: 'NOMINAL_BRAKING',
} as const;

export const nullStates = {
  UNKNOWN: 'UNKNOWN',
} as const;

export const podStates = {
  ...failureStates,
  ...staticStates,
  ...okayStates,
  ...nullStates,
};
