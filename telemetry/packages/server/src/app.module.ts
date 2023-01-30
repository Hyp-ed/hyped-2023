import { Module } from '@nestjs/common';
import { AppController } from './app.controller';
import { AppService } from './app.service';
import { InfluxModule } from './modules/influx/Influx.module';

@Module({
  imports: [InfluxModule],
  controllers: [AppController],
  providers: [AppService],
})
export class AppModule {}
