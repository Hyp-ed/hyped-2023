import { MqttPublish } from '@hyped/telemetry-types';
import { toast } from 'react-hot-toast';

export const calibrate = (podId: string) => {
  console.log('Calibrating');
  toast(`[${podId}] Calibrating!`);
};

export const startPod = (
  podId: string,
  options: {
    motorCooling: boolean;
    activeSuspension: boolean;
  },
) => {
  const { motorCooling, activeSuspension } = options;
  if (motorCooling && activeSuspension) {
    console.log('GO! (with motor cooling and active suspension)');
    toast.success(
      `[${podId}] Pod launched (with motor cooling and active suspension)!`,
      {
        icon: '🚀',
      },
    );
  } else if (motorCooling) {
    console.log('GO! (with motor cooling)');
    toast.success(`[${podId}] Pod launched (with motor cooling)!`, {
      icon: '🚀',
    });
  } else if (activeSuspension) {
    console.log('GO! (with active suspension)');
    toast.success(`[${podId}] Pod launched with (active suspension)!`, {
      icon: '🚀',
    });
  } else {
    console.log('GO!');
    toast.success(`[${podId}] Pod launched!`, { icon: '🚀' });
  }
};

export const stopPod = (podId: string) => {
  console.log('STOP!');
  toast(`[${podId}] Pod stopped!`, { icon: '🛑' });
};

export const clampBrakes = (podId: string) => {
  console.log('Clamping brakes');
  toast(`[${podId}] Brakes clamped!`);
};

// raise, lower, clamp, retract

export const clamp = (podId: string, publish: MqttPublish) => {
  console.log('Clamping');
  toast(`[${podId}] Clamped!`);
  publish({
    topic: 'controls/clamp',
    qos: 0,
    payload: 'clamp',
  });
};

export const retract = (podId: string, publish: MqttPublish) => {
  console.log('Retracting');
  toast(`[${podId}] Retracted!`);
  publish({
    topic: 'controls/retract',
    qos: 0,
    payload: 'retract',
  });
};

export const raise = (podId: string, publish: MqttPublish) => {
  console.log('Raising');
  toast(`[${podId}] Raised!`);
  publish({
    topic: 'controls/raise',
    qos: 0,
    payload: 'raise',
  });
};

export const lower = (podId: string, publish: MqttPublish) => {
  console.log('Lowering');
  toast(`[${podId}] Lowered!`);
  publish({
    topic: 'controls/lower',
    qos: 0,
    payload: 'lower',
  });
};

export const startHP = (podId: string, publish: MqttPublish) => {
  console.log('Starting HP');
  toast(`[${podId}] HP started!`);
  publish({
    topic: 'controls/start-hp',
    qos: 0,
    payload: 'start-hp',
  });
};

export const stopHP = (podId: string, publish: MqttPublish) => {
  console.log('Stopping HP');
  toast(`[${podId}] HP stopped!`);
  publish({
    topic: 'controls/stop-hp',
    qos: 0,
    payload: 'stop-hp',
  });
};
