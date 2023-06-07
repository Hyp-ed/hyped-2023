import { OpenMctMeasurement } from '@hyped/telemetry-types';
import { OpenMCT } from 'openmct/dist/openmct';
import { DomainObject } from 'openmct/dist/src/api/objects/ObjectAPI';
import { ObjectIdentitifer } from '../types/ObjectIdentifier';
import { fetchObjectTypes } from './data/object-types-data';
import { fetchMeasurement, fetchPod, fetchPodIds } from './data/pods-data';
import { convertNamespaceToPodId } from './utils/convertNamespaceToPodId';

const podObjectProvider = {
  get: (identifier: ObjectIdentitifer) => {
    return fetchPod(identifier.key).then((pod) => {
      return {
        identifier,
        name: pod.name,
        type: 'folder',
        location: 'ROOT',
      };
    });
  },
};

const measurementsObjectProvider = {
  get: (identifier: ObjectIdentitifer) => {
    const podId = convertNamespaceToPodId(identifier.namespace);
    return fetchMeasurement(podId, identifier.key).then((measurement) => {
      if (!measurement) {
        throw new Error('Measurement not found');
      }

      return {
        identifier,
        name: measurement.name,
        type: `hyped.${measurement.type}`,
        telemetry: {
          values: measurement.values,
        },
        location: `hyped.taxonomy:${podId}`,
        podId,
      };
    });
  },
};

const compositionProvider = {
  appliesTo: (domainObject: DomainObject) => {
    return (
      domainObject.identifier.namespace === 'hyped.taxonomy' &&
      domainObject.type === 'folder'
    );
  },
  load: (domainObject: DomainObject) => {
    const podId = domainObject.identifier.key;
    return fetchPod(podId).then((pod) =>
      pod.measurements.map((measurement: OpenMctMeasurement) => {
        return {
          namespace: `hyped.${podId}`,
          key: measurement.key,
        };
      }),
    );
  },
};

export function DictionaryPlugin() {
  return function install(openmct: OpenMCT) {
    fetchPodIds()
      .then(({ ids }) => {
        ids.forEach((id) => {
          openmct.objects.addRoot(
            {
              namespace: `hyped.taxonomy`,
              key: id,
            },
            openmct.priority.HIGH,
          );
          openmct.objects.addProvider(
            `hyped.${id}`,
            measurementsObjectProvider,
          );
        });

        openmct.objects.addProvider(`hyped.taxonomy`, podObjectProvider);
      })
      .catch((err) => {
        console.error(err);
        throw new Error('Failed to load dictionary');
      });

    fetchObjectTypes().then((objectTypes) => {
      objectTypes.forEach((objectType) => {
        openmct.types.addType(`hyped.${objectType.id}`, {
          name: objectType.name,
          cssClass: objectType.icon,
        });
      });
    });

    openmct.composition.addProvider(compositionProvider as any);
  };
}
