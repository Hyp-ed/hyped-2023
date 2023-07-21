import { InfluxModule } from '@/modules/influx/Influx.module';
import { Module } from '@nestjs/common';
import { FaultService } from './Fault.service';
import { HistoricalFaultDataService } from './data/historical/HistoricalFaultData.service';

@Module({
  imports: [InfluxModule],
  providers: [FaultService, HistoricalFaultDataService],
  exports: [FaultService],
})
export class FaultModule {}
