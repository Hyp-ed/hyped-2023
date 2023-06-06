import { Module } from '@nestjs/common';
import { MeasurementService } from './Measurement.service';
import { InfluxModule } from '../influx/Influx.module';
import { OpenMCTDataModule } from '../openmct/data/OpenMCTData.module';

@Module({
  imports: [InfluxModule, OpenMCTDataModule],
  providers: [MeasurementService],
  exports: [MeasurementService],
})
export class MeasurementModule {}
