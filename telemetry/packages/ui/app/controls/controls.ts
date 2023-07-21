import { SERVER_ENDPOINT } from '@/config';
import ky from 'ky';
import { toast } from 'react-hot-toast';

export const startPod = async (
  podId: string,
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

  const res = await ky.post(getURL(podId, 'start'));
  return res.status === 200;
};

export const stopPod = async (podId: string) => {
  toast(`[${podId}] Pod stopped!`, { icon: '🛑' });
  const res = await ky.post(getURL(podId, 'stop'));
  return res.status === 200;
};

export const clamp = async (podId: string) => {
  toast(`[${podId}] Clamped!`);
  const res = await ky.post(getURL(podId, 'clamp'));
  return res.status === 200;
};

export const retract = async (podId: string) => {
  toast(`[${podId}] Retracted!`);
  const res = await ky.post(getURL(podId, 'retract'));
  return res.status === 200;
};

export const raise = async (podId: string) => {
  toast(`[${podId}] Raised!`);
  const res = await ky.post(getURL(podId, 'raise'));
  return res.status === 200;
};

export const lower = async (podId: string) => {
  toast(`[${podId}] Lowered!`);
  const res = await ky.post(getURL(podId, 'lower'));
  return res.status === 200;
};

export const startHP = async (podId: string) => {
  toast(`[${podId}] HP started!`);
  const res = await ky.post(getURL(podId, 'start-hp'));
  return res.status === 200;
};

export const stopHP = async (podId: string) => {
  toast(`[${podId}] HP stopped!`);
  const res = await ky.post(getURL(podId, 'stop-hp'));
  return res.status === 200;
};

export const getURL = (podId: string, control: string) =>
  `${SERVER_ENDPOINT}/pods/${podId}/controls/${control}`;
