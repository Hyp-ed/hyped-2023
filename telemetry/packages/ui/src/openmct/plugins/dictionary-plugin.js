// @ts-nocheck
/* eslint-disable */

function getDictionary(name) {
  return http.get(`/dictionary/${name}.json`).then(function (result) {
    return result.data;
  });
}

const objectProvider = function (config) {
  return {
    get(identifier) {
      return getDictionary(config.key).then(function (dictionary) {
        if (identifier.key === config.key) {
          return {
            identifier,
            name: dictionary.name,
            type: "folder",
            location: "ROOT",
          };
        }
        const measurement = dictionary.measurements.filter(function (m) {
          return m.key === identifier.key;
        })[0];
        return {
          identifier,
          name: measurement.name,
          type: config.type,
          telemetry: {
            values: measurement.values,
          },
          location: `${config.namespace}:${config.key}`,
        };
      });
    },
  };
};

const compositionProvider = function (config) {
  return {
    appliesTo(domainObject) {
      return domainObject.identifier.namespace === config.namespace && domainObject.type === "folder";
    },
    load(domainObject) {
      return getDictionary(config.key).then(function (dictionary) {
        return dictionary.measurements.map(function (m) {
          return {
            namespace: config.namespace,
            key: m.key,
          };
        });
      });
    },
  };
};

const DictionaryPlugin = function (dictionary) {
  return function install(openmct) {
    openmct.objects.addRoot({
      namespace: dictionary.namespace,
      key: dictionary.key,
    });

    openmct.objects.addProvider(dictionary.namespace, objectProvider(dictionary));

    openmct.composition.addProvider(compositionProvider(dictionary));

    openmct.types.addType(dictionary.type, {
      name: dictionary.name,
      description: dictionary.description,
      cssClass: "icon-telemetry",
    });
  };
};
