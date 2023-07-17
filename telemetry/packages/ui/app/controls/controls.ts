import { MqttPublish } from '@/types/mqtt';
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

export const clamp = (podId: string, mqttPublish: MqttPublish) => {
  console.log('Clamping');
  toast(`[${podId}] Clamped!`);
  mqttPublish({
    topic: 'controls/clamp',
    qos: 0,
    payload: 'clamp',
  });
};

export const retract = (podId: string, mqttPublish: MqttPublish) => {
  console.log('Retracting');
  toast(`[${podId}] Retracted!`);
  mqttPublish({
    topic: 'controls/retract',
    qos: 0,
    payload: 'retract',
  });
};

export const raise = (podId: string, mqttPublish: MqttPublish) => {
  console.log('Raising');
  toast(`[${podId}] Raised!`);
  mqttPublish({
    topic: 'controls/raise',
    qos: 0,
    payload: 'raise',
  });
};

export const lower = (podId: string, mqttPublish: MqttPublish) => {
  console.log('Lowering');
  toast(`[${podId}] Lowered!`);
  mqttPublish({
    topic: 'controls/lower',
    qos: 0,
    payload: 'lower',
  });
};

export const startHP = (podId: string, mqttPublish: MqttPublish) => {
  console.log('Starting HP');
  toast(`[${podId}] HP started!`);
  mqttPublish({
    topic: 'controls/start-hp',
    qos: 0,
    payload: 'start-hp',
  });
}

export const stopHP = (podId: string, mqttPublish: MqttPublish) => {
  console.log('Stopping HP');
  toast(`[${podId}] HP stopped!`);
  mqttPublish({
    topic: 'controls/stop-hp',
    qos: 0,
    payload: 'stop-hp',
  });
}