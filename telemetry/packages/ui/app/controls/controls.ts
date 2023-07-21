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
    toast.success(
      `[${podId}] Pod launched (with motor cooling and active suspension)!`,
      {
        icon: '🚀',
      },
    );
  } else if (motorCooling) {
    toast.success(`[${podId}] Pod launched (with motor cooling)!`, {
      icon: '🚀',
    });
  } else if (activeSuspension) {
    toast.success(`[${podId}] Pod launched with (active suspension)!`, {
      icon: '🚀',
    });
  } else {
    toast.success(`[${podId}] Pod launched!`, { icon: '🚀' });
  }

  publish('controls/go', 'go', podId);
};

export const stopPod = (podId: string, publish: MqttPublish) => {
  toast(`[${podId}] Pod stopped!`, { icon: '🛑' });
  publish('controls/stop', 'stop', podId);
};

export const clamp = (podId: string, publish: MqttPublish) => {
  toast(`[${podId}] Clamped!`);
  publish('controls/clamp', 'clamp', podId);
};

export const retract = (podId: string, publish: MqttPublish) => {
  toast(`[${podId}] Retracted!`);
  publish('controls/retract', 'retract', podId);
};

export const raise = (podId: string, publish: MqttPublish) => {
  toast(`[${podId}] Raised!`);
  publish('controls/raise', 'raise', podId);
};

export const lower = (podId: string, publish: MqttPublish) => {
  toast(`[${podId}] Lowered!`);
  publish('controls/lower', 'lower', podId);
};

export const startHP = (podId: string, publish: MqttPublish) => {
  toast(`[${podId}] HP started!`);
  publish('controls/start-hp', 'start-hp', podId);
};

export const stopHP = (podId: string, publish: MqttPublish) => {
  toast(`[${podId}] HP stopped!`);
  publish('controls/stop-hp', 'stop-hp', podId);
};
