import { pods } from '@hyped/telemetry-constants';
import { Point } from '@influxdata/influxdb-client';
import { Injectable, LoggerService } from '@nestjs/common';
import { InfluxService } from '../influx/Influx.service';
import { Logger } from '../logger/Logger.decorator';
import { RealtimeDataGateway } from '../openmct/data/realtime/RealtimeData.gateway';
import {
  MeasurementReading,
  MeasurementReadingSchema,
} from './MeasurementReading.types';
import { MeasurementReadingValidationError } from './errors/MeasurementReadingValidationError';

@Injectable()
export class MeasurementService {
  constructor(
    @Logger()
    private readonly logger: LoggerService,
    private influxService: InfluxService,
    private realtimeDataGateway: RealtimeDataGateway,
  ) {}

  public addMeasurementReading(props: MeasurementReading) {
    const currentTime = new Date();

    const validatedMeasurement = this.validateMeasurementReading(props);

    if (!validatedMeasurement) {
      throw new MeasurementReadingValidationError('Invalid measurement');
    }

    const {
      measurement,
      reading: { podId, measurementKey, value },
    } = validatedMeasurement;

    this.realtimeDataGateway.sendMeasurementReading({
      podId,
      measurementKey,
      value,
    });

    const point = new Point('measurement')
      .timestamp(currentTime)
      .tag('podId', podId.toString())
      .tag('measurementKey', measurementKey)
      .tag('format', measurement.format)
      .floatField('value', value);

    try {
      this.influxService.write.writePoint(point);

      this.logger.debug(
        `Added measurement {${props.podId}/${props.measurementKey}}: ${props.value}`,
        MeasurementService.name,
      );
    } catch (e) {
      this.logger.error(
        `Failed to add measurement {${props.podId}/${props.measurementKey}}: ${props.value}`,
        e,
        MeasurementService.name,
      );
    }
  }

  private validateMeasurementReading(props: MeasurementReading) {
    const result = MeasurementReadingSchema.safeParse(props);
    if (!result.success) {
      throw new MeasurementReadingValidationError(result.error.message);
    }

    const { podId, measurementKey, value } = result.data;

    const pod = pods[podId];
    if (!pod) {
      throw new MeasurementReadingValidationError('Pod not found');
    }

    const measurement = pods[podId]['measurements'][measurementKey];
    if (!measurement) {
      throw new MeasurementReadingValidationError('Measurement not found');
    }

    if (measurement.format === 'enum') {
      const enumValue = measurement.enumerations.find((e) => e.value === value);

      if (!enumValue) {
        throw new MeasurementReadingValidationError('Invalid enum value');
      }
    }

    return {
      reading: result.data,
      measurement,
    };
  }
}