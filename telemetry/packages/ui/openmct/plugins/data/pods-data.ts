import { OpenMctPod } from '@hyped/telemetry-types';
import { http } from '../../core/http';

export function fetchPodIds() {
  return http
    .get('openmct/dictionary/pods')
    .json<{ ids: string[] }>()
    .then((data) => {
      return data;
    });
}

export function fetchPod(id: string) {
  return http
    .get(`openmct/dictionary/pod/${id}`)
    .json<OpenMctPod>()
    .then((data) => {
      return data;
    });
}
