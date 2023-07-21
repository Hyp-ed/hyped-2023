import { Injectable, LoggerService, Inject } from '@nestjs/common';
import { MqttService } from 'nest-mqtt';
import { Logger } from '../logger/Logger.decorator';

@Injectable()
export class PodControlsService {
  constructor(
    @Inject(MqttService) private readonly mqttService: MqttService,
    @Logger()
    private readonly logger: LoggerService,
  ) {}

  async sendControlMessage(control: string, podId: string) {
    this.mqttService.publish(`hyped/${podId}/controls/${control}`, control);
    this.logger.log(
      `Control message "${control}" sent to pod "${podId}"`,
      PodControlsService.name,
    );
    return true;
  }
}
