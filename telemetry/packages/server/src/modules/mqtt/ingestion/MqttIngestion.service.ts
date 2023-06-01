import { Injectable, LoggerService } from '@nestjs/common';
import { Params, Payload, Subscribe } from 'nest-mqtt';
import { Logger } from 'src/modules/logger/Logger.decorator';
import { MeasurementService } from 'src/modules/measurement/Measurement.service';

@Injectable()
export class MqttIngestionService {
  constructor(
    @Logger()
    private readonly logger: LoggerService,
    private measurementService: MeasurementService,
  ) {}

  @Subscribe('hyped/+/+')
  getNotifications(@Params() rawParams: string[], @Payload() rawValue: any) {
    const podId = rawParams[0];
    const measurementKey = rawParams[1];
    const value = rawValue;

    this.measurementService.addMeasurement({ podId, measurementKey, value });
  }
}
