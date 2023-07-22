import { Module } from '@nestjs/common';
import { LogsController } from './Logs.controller';
import { LogsService } from './Logs.service';

@Module({
  controllers: [LogsController],
  providers: [LogsService],
})
export class LogsModule {}
