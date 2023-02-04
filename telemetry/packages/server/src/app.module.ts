import { Module } from '@nestjs/common';
import { AppController } from './app.controller';
import { AppService } from './app.service';
// import { InfluxModule } from './modules/influx/Influx.module';
import { SensorsModule } from './modules/sensors/Sensors.module';

@Module({
  imports: [SensorsModule],
  controllers: [AppController],
  providers: [AppService],
})
export class AppModule {}
