import { OpenMCT } from 'openmct/dist/openmct';
import { http } from '../core/http';
import { parsePodId } from './utils/parsePodId';
import { DomainObject } from 'openmct/dist/src/api/objects/ObjectAPI';
import { TelemetryRequest } from 'openmct/dist/src/api/telemetry/TelemetryAPI';

export function HistoricalTelemetryPlugin() {
  return function install(openmct: OpenMCT) {
    const provider = {
      supportsRequest: function (domainObject: DomainObject) {
        return domainObject.type.startsWith('hyped.');
      },
      request: function (domainObject: DomainObject, options: TelemetryRequest) {
        const { start, end } = options;
        const podId = parsePodId(domainObject.identifier.namespace)
        const measurementKey = domainObject.identifier.key;
        const url = `openmct/data/historical/pods/${podId}/measurements/${measurementKey}?start=${start}&end=${end}`

        return http
          .get(url)
          .json()
          .then((data) => data);
      },
    };

    openmct.telemetry.addProvider(provider);
  };
}
