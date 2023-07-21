import { OpenMCT } from 'openmct/dist/openmct';
import utils from './fault-source'
import { AugmentedDomainObject } from 'openmct/types/AugmentedDomainObject';
import { Fault } from 'openmct/dist/src/api/faultmanagement/FaultManagementAPI';
import { FAULT_MANAGEMENT_DOMAIN_TYPE } from './constants';
import { DomainObject } from 'openmct/dist/src/api/objects/ObjectAPI';
import { RealtimeFaultsProvider } from './realtime-faults-provider';

export function FaultsPlugin() {
  return function install(openmct: OpenMCT) {
    openmct.install(openmct.plugins.FaultManagement());

    // const faultsData = utils.randomFaults(false);

    const realtimeProvider = RealtimeFaultsProvider();

    openmct.faults.addProvider({
      request(domainObject: AugmentedDomainObject, options: any) {
        return Promise.resolve();
      },
      subscribe(domainObject: DomainObject, callback: any) {
        realtimeProvider.subscribe(callback);
        return realtimeProvider.unsubscribe
      },
      // acknowledgeFault(fault: Fault, { comment = '' }) {
      //   utils.acknowledgeFault(fault);
        
      //   return Promise.resolve({
      //     success: true
      //   });
      // },
      // shelveFault(fault: Fault, duration: any) {
      //   utils.shelveFault(fault, duration);
        
      //   return Promise.resolve({
      //     success: true
      //   });
      // },
      supportsRequest(domainObject: AugmentedDomainObject) {
        return domainObject.type === FAULT_MANAGEMENT_DOMAIN_TYPE;
      },
      supportsSubscribe(domainObject: AugmentedDomainObject) {
        return domainObject.type === FAULT_MANAGEMENT_DOMAIN_TYPE;
      },
    });
  };
}
