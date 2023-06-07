import { Module } from '@nestjs/common';
import { HistoricalDataController } from './historical/HistoricalData.controller';
import { HistoricalDataService } from './historical/HistoricalData.service';
import { InfluxModule } from 'src/modules/influx/Influx.module';

@Module({
  imports: [InfluxModule],
  controllers: [HistoricalDataController],
  providers: [HistoricalDataService],
})
export class OpenMCTDataModule {}
