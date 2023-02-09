import { Module } from '@nestjs/common';
import { AppController } from './app.controller';
import { AppService } from './app.service';
// import { InfluxModule } from './modules/influx/Influx.module';
import { OpenMCTModule } from './modules/openmct/OpenMCT.module';

@Module({
  imports: [OpenMCTModule],
  controllers: [AppController],
  providers: [AppService],
})
export class AppModule {}
