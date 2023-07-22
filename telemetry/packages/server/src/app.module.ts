import { Module } from '@nestjs/common';
import { AppController } from './app.controller';
import { AppService } from './app.service';
import { InfluxModule } from './modules/influx/Influx.module';
import { LoggerModule } from './modules/logger/Logger.module';
import { MqttClientModule } from './modules/mqtt/client/MqttClientModule';
import { MqttIngestionModule } from './modules/mqtt/ingestion/MqttIngestion.module';
import { OpenMCTModule } from './modules/openmct/OpenMCT.module';
import { MeasurementModule } from './modules/measurement/Measurement.module';
import { FaultModule } from './modules/openmct/faults/Fault.module';
import { PodControlsModule } from './modules/controls/PodControls.module';
import { RemoteLogsModule } from './modules/remote-logs/RemoteLogs.module';

@Module({
  imports: [
    LoggerModule,
    MqttClientModule,
    InfluxModule,
    MqttIngestionModule,
    OpenMCTModule,
    MeasurementModule,
    FaultModule,
    PodControlsModule,
    RemoteLogsModule,
  ],
  controllers: [AppController],
  providers: [AppService],
})
export class AppModule {}
