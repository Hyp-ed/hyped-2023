import { OpenMCT } from 'openmct/dist/openmct';
import { ObjectIdentitifer } from '../types/ObjectIdentifier';
import { fetchPod, fetchPodIds } from './data/pods-data';
import { fetchObjectTypes } from './data/object-types-data';
import { parsePodId } from './utils/parsePodId';

const podObjectProvider = {
  get: (identifier: ObjectIdentitifer) => {
    if (!identifier.key.startsWith('pod_')) {
      throw new Error('Invalid identifier');
    }

    const podId = parsePodId(identifier.key);
    return fetchPod(podId).then((pod) => {
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
    if (!identifier.namespace.startsWith('hyped.pod_')) {
      throw new Error('Invalid identifier');
    }

    const podId = parsePodId(identifier.namespace);
    return fetchPod(podId).then((pod) => {
      const measurement = pod.measurements.find(
        (measurement) => measurement.key === identifier.key,
      );

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
        location: `hyped.taxonomy:pod_${podId}`,
      };
    });
  },
};

const compositionProvider = {
  appliesTo: (domainObject: any) => {
    return (
      domainObject.identifier.namespace === 'hyped.taxonomy' &&
      domainObject.type === 'folder'
    );
  },
  load: (domainObject: any) => {
    const podId = parsePodId(domainObject.identifier.key);
    return fetchPod(podId).then((pod) =>
      pod.measurements.map((measurement: any) => {
        return {
          namespace: `hyped.pod_${podId}`,
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
              key: `pod_${id}`,
            },
            openmct.priority.HIGH,
          );
          openmct.objects.addProvider(
            `hyped.pod_${id}`,
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
