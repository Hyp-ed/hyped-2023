/**
 * States taken from ```hyped-2023/lib/state_machine/state.hpp``` 09/06/2023
 * Added ```unknown``` state to represent the state of the pod when the state is not known
 */
export type PodState = keyof typeof podStates;

export const failureStates = {
  preFrictionBrakingFail: 'preFrictionBrakingFail',
  frictionBrakingFail: 'frictionBrakingFail',
  failureBraking: 'failureBraking',
  failureStopped: 'failureStopped',
  failureCalibrating: 'failureCalibrating',
} as const;

export const staticStates = {
  idle: 'idle',
  calibrating: 'calibrating',
  ready: 'ready',
  stopped: 'stopped',
  off: 'off',
} as const;

export const okayStates = {
  accelerating: 'accelerating',
  cruising: 'cruising',
  motorBraking: 'motorBraking',
  preFrictionBraking: 'preFrictionBraking',
  frictionBraking: 'frictionBraking',
} as const;

export const nullStates = {
  unknown: 'unknown',
} as const;

export const podStates = {
  ...failureStates,
  ...staticStates,
  ...okayStates,
  ...nullStates,
};
