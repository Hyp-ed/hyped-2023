import { OpenMCT } from 'openmct/dist/openmct';
import { io } from 'socket.io-client';
import { socket as socketConstants } from '@hyped/telemetry-constants'
import { SERVER_ENDPOINT } from 'openmct/core/config';
import { AugmentedDomainObject } from 'openmct/types/AugmentedDomainObject';

export function RealtimeFaultsProvider() {
    const socket = io(SERVER_ENDPOINT, { path: '/openmct/faults/realtime' });
    // handle socket disconnects

    var faultCallback: any = null;

    socket.on(socketConstants.FAULT_EVENT, ({fault}) => {
      console.log(fault)
      faultCallback(fault);
    });

    return {
      subscribe: (callback: any) => {
        socket.emit(socketConstants.EVENTS.SUBSCRIBE_TO_FAULTS);
        faultCallback = callback;
      },
      unsubscribe: () => {
        socket.emit(socketConstants.EVENTS.UNSUBSCRIBE_FROM_FAULTS);
      }
    };
}
