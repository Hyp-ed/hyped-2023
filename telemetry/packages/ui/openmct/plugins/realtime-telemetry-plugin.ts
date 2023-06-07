import { EVENTS } from '@hyped/telemetry-constants';
import { OpenMCT } from 'openmct/dist/openmct';
import { io } from 'socket.io-client';
import { SERVER_ENDPOINT } from '../core/config';
import { AugmentedDomainObject } from '../types/AugmentedDomainObject';

const getMeasurementRoomName = (podId: string, measurementKey: string) => {
  return `${podId}/measurement/${measurementKey}`;
};

export function RealtimeTelemetryPlugin() {
  return function install(openmct: OpenMCT) {
    const socket = io(SERVER_ENDPOINT, { path: '/openmct/data/realtime' });
    // handle socket disconnects

    const listenerCallbacks: any = {};

    socket.on('measurement', (data) => {
      const { podId, measurementKey, value, timestamp } = data;
      const roomName = getMeasurementRoomName(podId, measurementKey);

      if (listenerCallbacks[roomName]) {
        listenerCallbacks[roomName]({ id: measurementKey, value, timestamp });
      }
    });

    const provider = {
      supportsSubscribe: (domainObject: AugmentedDomainObject) => {
        return domainObject.podId !== undefined;
      },
      subscribe: (domainObject: AugmentedDomainObject, callback: any) => {
        const { podId, identifier } = domainObject;
        const roomName = getMeasurementRoomName(podId, identifier.key);

        listenerCallbacks[roomName] = callback;
        socket.emit(EVENTS.SUBSCRIBE_TO_MEASUREMENT, roomName);

        return function unsubscribe() {
          delete listenerCallbacks[roomName];
          socket.emit(EVENTS.UNSUBSCRIBE_FROM_MEASUREMENT, roomName);
        }
      },
    };

    openmct.telemetry.addProvider(provider);
  }
}
