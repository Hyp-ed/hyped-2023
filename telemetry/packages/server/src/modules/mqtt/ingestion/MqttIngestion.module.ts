import { Module } from '@nestjs/common';
import { MqttIngestionService } from './MqttIngestion.service';

@Module({
  providers: [MqttIngestionService],
})
export class MqttIngestionModule {}
