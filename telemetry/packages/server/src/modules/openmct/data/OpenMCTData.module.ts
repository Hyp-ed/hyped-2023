import { Module } from '@nestjs/common';
import { HistoricalDataController } from './historical/HistoricalData.controller';
import { HistoricalDataService } from './historical/HistoricalData.service';
import { InfluxModule } from 'src/modules/influx/Influx.module';
import { RealtimeDataGateway } from './realtime/RealtimeData.gateway';

@Module({
  imports: [InfluxModule],
  controllers: [HistoricalDataController],
  providers: [HistoricalDataService, RealtimeDataGateway],
  exports: [RealtimeDataGateway],
})
export class OpenMCTDataModule {}
