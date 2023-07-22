import { http } from 'openmct/core/http';
import { toast } from 'react-hot-toast';

export const startPod = async (podId: string) => {
  toast.success(`[${podId}] Pod launched!`, { icon: '🚀' });
  const res = await http.post(`pods/${podId}/controls/start`);
  return res.status === 200;
};

export const stopPod = async (podId: string) => {
  toast(`[${podId}] Pod stopped!`, { icon: '🛑' });
  const res = await http.post(`pods/${podId}/controls/stop`);
  return res.status === 200;
};

export const clamp = async (podId: string) => {
  toast(`[${podId}] Clamped!`);
  const res = await http.post(`pods/${podId}/controls/clamp`);
  return res.status === 200;
};

export const retract = async (podId: string) => {
  toast(`[${podId}] Retracted!`);
  const res = await http.post(`pods/${podId}/controls/retract`);
  return res.status === 200;
};

export const raise = async (podId: string) => {
  toast(`[${podId}] Raised!`);
  const res = await http.post(`pods/${podId}/controls/raise`);
  return res.status === 200;
};

export const lower = async (podId: string) => {
  toast(`[${podId}] Lowered!`);
  const res = await http.post(`pods/${podId}/controls/lower`);
  return res.status === 200;
};

export const startHP = async (podId: string) => {
  toast(`[${podId}] HP started!`);
  const res = await http.post(`pods/${podId}/controls/start-hp`);
  return res.status === 200;
};

export const stopHP = async (podId: string) => {
  toast(`[${podId}] HP stopped!`);
  const res = await http.post(`pods/${podId}/controls/stop-hp`);
  return res.status === 200;
};
