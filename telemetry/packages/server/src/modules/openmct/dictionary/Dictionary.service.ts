import { POD_IDS, pods } from '@hyped/telemetry-constants';
import { OpenMctDictionary, OpenMctPod } from '@hyped/telemetry-types';
import { Injectable } from '@nestjs/common';

@Injectable()
export class DictionaryService {
  getDictionary(): OpenMctDictionary {
    const dictionary: OpenMctDictionary = {};
    POD_IDS.forEach((podId) => {
      dictionary[podId] = this.getPod(podId);
    });

    return dictionary;
  }

  getPodIds() {
    return POD_IDS;
  }

  getPod(podId: string): OpenMctPod {
    const pod = pods[podId as keyof typeof pods];

    if (!pod) {
      throw new Error(`Pod ${podId} not found`);
    }

    const measurements = Object.entries(pod.measurements).map(
      ([key, measurement]) => {
        return {
          name: measurement.name,
          key: key,
          type: measurement.type,
          values: [
            {
              key: 'value',
              name: 'Value',
              unit: measurement.unit,
              format: measurement.format,
              ...('range' in measurement && {
                min: measurement.range?.min,
                max: measurement.range?.max,
              }),
              ...('enumerations' in measurement && {
                enumerations: measurement.enumerations,
              }),
              hints: {
                range: 1,
              },
            },
            {
              key: 'utc',
              source: 'timestamp',
              name: 'Timestamp',
              format: 'utc',
              hints: {
                domain: 1,
              },
            },
          ],
        };
      },
    );

    return {
      name: pod.name,
      id: pod.id,
      measurements,
    };
  }
}
