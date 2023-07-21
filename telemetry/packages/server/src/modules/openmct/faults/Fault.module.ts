import { InfluxModule } from '@/modules/influx/Influx.module';
import { Module } from '@nestjs/common';
import { FaultService } from './Fault.service';
import { HistoricalFaultDataService } from './data/historical/HistoricalFaultData.service';
import { RealtimeFaultDataGateway } from './data/realtime/RealtimeFaultData.gateway';

@Module({
  imports: [InfluxModule],
  providers: [FaultService, HistoricalFaultDataService, RealtimeFaultDataGateway],
  exports: [FaultService],
})
export class FaultModule {}
