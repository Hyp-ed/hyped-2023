import { Body, Controller, Param, Post } from '@nestjs/common';
import { LogsService } from './Logs.service';

@Controller('logs')
export class LogsController {
  constructor(private uiLogsService: LogsService) {}
  @Post('ui/:podId')
  logUI(@Param('podId') podId: string, @Body() body: any) {
    return this.uiLogsService.logUI(podId, body.message);
  }
}
