import { pods, POD_IDS } from '@hyped/telemetry-constants';
import { Injectable, LoggerService } from '@nestjs/common';
import { Params, Payload, Subscribe, Topic } from 'nest-mqtt';
import { Logger } from 'src/modules/logger/Logger.decorator';
import { z } from 'zod';

const MqttMessage = z.object({
  podId: z.enum(POD_IDS),
  measurementName: z.string(),
  value: z.coerce.string(),
});

@Injectable()
export class MqttIngestionService {
  constructor(
    @Logger()
    private readonly logger: LoggerService,
  ) {}

  @Subscribe('hyped/+/+')
  getNotifications(
    @Topic() rawTopic: string,
    @Params() rawParams: string[],
    @Payload() rawValue: any,
  ) {
    const result = MqttMessage.safeParse({
      podId: rawParams[0],
      measurementName: rawParams[1],
      value: rawValue,
    });

    if (!result.success) {
      this.logger.error(
        `Invalid MQTT message {${rawTopic}}: ${result.error.message}`,
        null,
        MqttIngestionService.name,
      );
      return;
    }

    const { podId, measurementName, value } = result.data;

    const measurement = pods[podId]['measurements'][measurementName];

    if (!measurement) {
      this.logger.error(
        `Invalid MQTT message {${rawTopic}}: Measurement not found`,
        null,
        MqttIngestionService.name,
      );
      return;
    }

    this.logger.debug(
      `Incoming measurement reading {${measurement.name}}: ${value}}`,
      MqttIngestionService.name,
    );
  }
}
