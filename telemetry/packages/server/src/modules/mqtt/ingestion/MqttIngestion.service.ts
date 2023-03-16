import { Injectable } from '@nestjs/common';
import { Payload, Subscribe } from 'nest-mqtt';

@Injectable()
export class MqttIngestionService {
  @Subscribe('hyped.temperature')
  getNotifications(@Payload() data: any) {
    console.log(data);
  }
}
