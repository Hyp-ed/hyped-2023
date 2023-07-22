import { Body, Controller, Param, Post } from '@nestjs/common';
import { LogsService } from './Logs.service';

@Controller('logs')
export class LogsController {
  constructor(private uiLogsService: LogsService) {}
  @Post('ui')
  logUI(@Body() body: any) {
    return this.uiLogsService.logUI(body.message);
  }
  @Post('ui/:podId')
  logUIWithPodID(@Param('podId') podId: string, @Body() body: any) {
    return this.uiLogsService.logUIWithPodID(podId, body.message);
  }
}
