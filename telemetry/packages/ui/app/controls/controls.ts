import { MqttPublish } from '@/types/mqtt';
import { toast } from 'react-hot-toast';

export const startPod = (
  podId: string,
  publish: MqttPublish,
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

  publish('controls/go', 'go', podId);
};

export const stopPod = (podId: string, publish: MqttPublish) => {
  console.log('STOP!');
  toast(`[${podId}] Pod stopped!`, { icon: '🛑' });
  publish('controls/stop', 'stop', podId);
};

export const clamp = (podId: string, publish: MqttPublish) => {
  console.log('Clamping');
  toast(`[${podId}] Clamped!`);
  publish('controls/clamp', 'clamp', podId);
};

export const retract = (podId: string, publish: MqttPublish) => {
  console.log('Retracting');
  toast(`[${podId}] Retracted!`);
  publish('controls/retract', 'retract', podId);
};

export const raise = (podId: string, publish: MqttPublish) => {
  console.log('Raising');
  toast(`[${podId}] Raised!`);
  publish('controls/raise', 'raise', podId);
};

export const lower = (podId: string, publish: MqttPublish) => {
  console.log('Lowering');
  toast(`[${podId}] Lowered!`);
  publish('controls/lower', 'lower', podId);
};

export const startHP = (podId: string, publish: MqttPublish) => {
  console.log('Starting HP');
  toast(`[${podId}] HP started!`);
  publish('controls/start-hp', 'start-hp', podId);
};

export const stopHP = (podId: string, publish: MqttPublish) => {
  console.log('Stopping HP');
  toast(`[${podId}] HP stopped!`);
  publish('controls/stop-hp', 'stop-hp', podId);
};
