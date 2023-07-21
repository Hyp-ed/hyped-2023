import { Controller, Param, Post } from '@nestjs/common';
import { WarningsService } from './Warnings.service';

@Controller('pods/:podId/warnings')
export class WarningsController {
  constructor(private warningsServcie: WarningsService) {}

  @Post('latency')
  async createLatencyWarning(@Param('podId') podId: string) {
    await this.warningsServcie.createLatencyWarning(podId);
  }
}
