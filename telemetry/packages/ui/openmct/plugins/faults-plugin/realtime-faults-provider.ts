import { socket as socketConstants } from '@hyped/telemetry-constants';
import { SERVER_ENDPOINT } from 'openmct/core/config';
import { AugmentedDomainObject } from 'openmct/types/AugmentedDomainObject';
import { io } from 'socket.io-client';
import { FAULT_MANAGEMENT_DOMAIN_TYPE } from './constants';

export function RealtimeFaultsProvider() {
  const socket = io(SERVER_ENDPOINT, { path: '/openmct/faults/realtime' });
  // handle socket disconnects

  var faultCallback: any = null;

  socket.on(socketConstants.FAULT_EVENT, ({ fault }) => {
    console.log(fault);
    faultCallback(fault);
  });

  return {
    supportsSubscribe(domainObject: AugmentedDomainObject) {
      return domainObject.type === FAULT_MANAGEMENT_DOMAIN_TYPE;
    },
    subscribe: (domainObject: AugmentedDomainObject, callback: any) => {
      socket.emit(socketConstants.EVENTS.SUBSCRIBE_TO_FAULTS);
      faultCallback = callback;

      return function unsubscribe() {
        faultCallback = null;
        socket.emit(socketConstants.EVENTS.UNSUBSCRIBE_FROM_FAULTS);
      };
    },
  };
}
