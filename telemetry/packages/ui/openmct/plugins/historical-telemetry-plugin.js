// @ts-nocheck
/* eslint-disable */

export function HistoricalTelemetryPlugin(namespaces) {
  return function install(openmct) {
    const provider = {
      supportsRequest(domainObject) {
        if (namespaces.indexOf(domainObject.type) === -1) {
          return false;
        }
        return true;
      },
      request(domainObject, options) {
        const url = `/telemetry/${domainObject.identifier.key}?start=${options.start}&end=${options.end}`;

        return http.get(url).then(function (resp) {
          return resp.data;
        });
      },
    };

    openmct.telemetry.addProvider(provider);
  };
}
