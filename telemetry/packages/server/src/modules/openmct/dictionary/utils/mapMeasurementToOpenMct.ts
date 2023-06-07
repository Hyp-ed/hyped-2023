import { Measurement, OpenMctMeasurement } from '@hyped/telemetry-types';

export function mapMeasurementToOpenMct(
  measurement: Measurement,
): OpenMctMeasurement {
  return {
    name: measurement.name,
    key: measurement.key,
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
}
