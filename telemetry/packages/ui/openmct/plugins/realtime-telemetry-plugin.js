// @ts-nocheck
/* eslint-disable */

function RealtimeTelemetryPlugin(namespaces) {
  return function (openmct) {
    const socket = new WebSocket(`ws://${window.location.hostname}:8082`);
    const listeners = {};

    socket.onmessage = function (event) {
      point = JSON.parse(event.data);
      if (listeners[point.id]) {
        listeners[point.id].forEach(function (l) {
          l(point);
        });
      }
    };

    const provider = {
      supportsSubscribe(domainObject) {
        if (namespaces.indexOf(domainObject.type) === -1) {
          return false;
        }
        return true;
      },
      subscribe(domainObject, callback, options) {
        if (!listeners[domainObject.identifier.key]) {
          listeners[domainObject.identifier.key] = [];
        }
        if (!listeners[domainObject.identifier.key].length) {
          socket.send(`subscribe ${domainObject.identifier.key}`);
        }
        listeners[domainObject.identifier.key].push(callback);
        return function () {
          listeners[domainObject.identifier.key] = listeners[domainObject.identifier.key].filter(function (c) {
            return c !== callback;
          });

          if (!listeners[domainObject.identifier.key].length) {
            socket.send(`unsubscribe ${domainObject.identifier.key}`);
          }
        };
      },
    };
    openmct.telemetry.addProvider(provider);
  };
}
