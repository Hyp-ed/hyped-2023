import { OpenMCT } from 'openmct/dist/openmct';
import { http } from '../core/http';
import { getPodIdFromKey } from './utils/getPodIdFromKey';

export function HistoricalTelemetryPlugin() {
  return function install(openmct: OpenMCT) {
    const provider = {
      supportsRequest: function (domainObject: any) {
        return domainObject.type.startsWith('hyped.');
      },
      request: function (domainObject: any, options: any) {
        const url = `openmct/data/historical/pods/${getPodIdFromKey(domainObject.identifier.namespace)}/measurements/${domainObject.identifier.key}?start=${options.start}&end=${options.end}`

        return http
          .get(url)
          .json()
          .then((data) => data);
      },
    };

    openmct.telemetry.addProvider(provider);
  };
}
