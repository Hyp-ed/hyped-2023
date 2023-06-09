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

export const retractBrakes = (podId: string) => {
  console.log('Retracting brakes');
  toast(`[${podId}] Brakes retracted!`);
};

export const clampBrakes = (podId: string) => {
  console.log('Clamping brakes');
  toast(`[${podId}] Brakes clamped!`);
};
