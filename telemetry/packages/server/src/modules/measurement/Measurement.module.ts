import { Module } from '@nestjs/common';
import { MeasurementService } from './Measurement.service';
import { InfluxModule } from '../influx/Influx.module';

@Module({
  imports: [InfluxModule],
  providers: [MeasurementService],
  exports: [MeasurementService],
})
export class MeasurementModule {}
