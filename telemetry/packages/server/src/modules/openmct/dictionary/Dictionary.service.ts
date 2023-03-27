import { pods } from '@hyped/telemetry-constants';
import { Injectable } from '@nestjs/common';

@Injectable()
export class DictionaryService {
  getDictionary(podId = '1') {
    const pod = pods[podId as keyof typeof pods];

    if (!pod) {
      throw new Error(`Pod ${podId} not found`);
    }

    const dictionary = Object.entries(pod.measurements).map(
      ([key, measurement]) => {
        return {
          name: measurement.name,
          key: key,
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
              units: {
                domain: 'time',
              },
            },
          ],
        };
      },
    );

    return {
      name: pod.name,
      key: pod.key,
      measurements: dictionary,
    };
  }
}
