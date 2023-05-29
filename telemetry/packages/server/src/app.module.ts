import { Module } from '@nestjs/common';
import { AppController } from './app.controller';
import { AppService } from './app.service';
import { InfluxModule } from './modules/influx/Influx.module';
import { LoggerModule } from './modules/logger/Logger.module';
import { MqttClientModule } from './modules/mqtt/client/MqttClientModule';
import { MqttIngestionModule } from './modules/mqtt/ingestion/MqttIngestion.module';
import { OpenMCTModule } from './modules/openmct/OpenMCT.module';

@Module({
  imports: [
    LoggerModule,
    MqttClientModule,
    InfluxModule,
    MqttIngestionModule,
    OpenMCTModule,
  ],
  controllers: [AppController],
  providers: [AppService],
})
export class AppModule {}
