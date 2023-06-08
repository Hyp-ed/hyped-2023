import { Injectable } from '@nestjs/common';
import { Params, Payload, Subscribe } from 'nest-mqtt';
import { MeasurementService } from 'src/modules/measurement/Measurement.service';

@Injectable()
export class MqttIngestionService {
  constructor(private measurementService: MeasurementService) {}

  @Subscribe('hyped/+/+')
  getNotifications(@Params() rawParams: string[], @Payload() rawValue: any) {
    const podId = rawParams[0];
    const measurementKey = rawParams[1];
    const value = rawValue;

    this.measurementService.addMeasurement({ podId, measurementKey, value });
  }
}
