import { pods } from '@hyped/telemetry-constants';
import { Point } from '@influxdata/influxdb-client';
import { Injectable, LoggerService } from '@nestjs/common';
import { z } from 'zod';
import { InfluxService } from '../influx/Influx.service';
import { Logger } from '../logger/Logger.decorator';
import { RealtimeDataGateway } from '../openmct/data/realtime/RealtimeData.gateway';

const BaseMeasurementReading = z.object({
  podId: z.string(),
  measurementKey: z.string(),
  value: z.number(),
});

type MeasurementReading = z.infer<typeof BaseMeasurementReading>;

@Injectable()
export class MeasurementService {
  constructor(
    @Logger()
    private readonly logger: LoggerService,
    private influxService: InfluxService,
    private realtimeDataGateway: RealtimeDataGateway,
  ) {}

  public addMeasurement(props: MeasurementReading) {
    const currentTime = new Date();

    const validatedMeasurement = this.validateMeasurementReading(props);

    if (!validatedMeasurement) {
      // do something more here...maybe
      return;
    }

    const {
      measurement,
      reading: { podId, measurementKey, value },
    } = validatedMeasurement;

    this.realtimeDataGateway.sendMeasurement(podId, measurementKey, value);

    const point = new Point('measurement')
      .timestamp(currentTime)
      .tag('podId', podId)
      .tag('measurementKey', measurementKey)
      .tag('format', measurement.format)
      .floatField('value', value);

    try {
      this.influxService.write.writePoint(point);

      this.logger.verbose(
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

  private logValidationError(message: string, props: MeasurementReading) {
    this.logger.error(
      `${message} {${props.podId ?? 'unknownPod'}/${
        props.measurementKey ?? 'unknownMeasurement'
      }}: ${props.value ?? 'no value'}`,
      null,
      MeasurementService.name,
    );
  }

  private validateMeasurementReading(props: MeasurementReading) {
    const result = BaseMeasurementReading.safeParse(props);
    if (!result.success) {
      this.logValidationError('Invalid measurement reading', props);
      return;
    }

    const { podId, measurementKey, value } = result.data;

    const pod = pods[podId];
    if (!pod) {
      this.logValidationError('Pod not found', props);
      return;
    }

    const measurement = pods[podId]['measurements'][measurementKey];
    if (!measurement) {
      this.logValidationError('Measurement not found', props);
      return;
    }

    if (measurement.format === 'enum') {
      const enumValue = measurement.enumerations.find((e) => e.value === value);

      if (!enumValue) {
        this.logValidationError('Invalid enum value', props);
        return;
      }
    }

    return {
      reading: result.data,
      measurement,
    };
  }
}
