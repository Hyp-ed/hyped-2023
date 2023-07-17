import { Injectable } from '@nestjs/common';
import { Params, Payload, Subscribe } from 'nest-mqtt';
import { MeasurementService } from '@/modules/measurement/Measurement.service';

@Injectable()
export class MqttIngestionService {
  constructor(private measurementService: MeasurementService) {}

  @Subscribe('hyped/+/measurement/+')
  getMeasurementReading(
    @Params() rawParams: string[],
    @Payload() rawValue: any,
  ) {
    const podId = rawParams[0];
    const measurementKey = rawParams[1];
    const value = rawValue;

    if (
      !podId ||
      !measurementKey ||
      value === undefined ||
      value === null ||
      isNaN(value)
    ) {
      throw new Error('Invalid MQTT message');
    }

    this.measurementService.addMeasurementReading({
      podId,
      measurementKey,
      value,
    });
  }
}
