import { Body, Controller, Param, Post } from '@nestjs/common';
import { LogsService } from './Logs.service';

@Controller('logs')
export class LogsController {
  constructor(private uiLogsService: LogsService) {}
  @Post('ui')
  logUIMessage(@Body() body: any) {
    return this.uiLogsService.logUIMessage(body.message);
  }
  @Post('ui/:podId')
  logUIMessageWithPodID(@Param('podId') podId: string, @Body() body: any) {
    return this.uiLogsService.logUIMessageWithPodID(podId, body.message);
  }
}
