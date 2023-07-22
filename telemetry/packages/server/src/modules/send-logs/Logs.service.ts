import { Injectable, LoggerService } from '@nestjs/common';
import { Logger } from '../logger/Logger.decorator';

@Injectable()
export class LogsService {
  constructor(
    @Logger()
    private readonly logger: LoggerService,
  ) {}

  async logUIMessage(message: string) {
    this.logger.log(`Pod UI log: ${message}`, LogsService.name);
    return true;
  }

  async logUIMessageWithPodID(podId: string, message: string) {
    this.logger.log(`Pod "${podId}" UI log: ${message}`, LogsService.name);
    return true;
  }
}
