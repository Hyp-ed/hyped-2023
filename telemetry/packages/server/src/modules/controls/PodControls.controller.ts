import { Controller, Param, Post } from '@nestjs/common';
import { PodControlsService } from './PodControls.service';

@Controller('pods/:podId/controls')
export class PodControlsController {
  constructor(private podControlsService: PodControlsService) {}
  @Post(':control')
  controlPod(@Param('control') control: string, @Param('podId') podId: string) {
    return this.podControlsService.sendControlMessage(control, podId);
  }
}
