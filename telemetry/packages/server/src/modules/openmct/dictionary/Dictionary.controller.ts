import { Controller, Get, Param } from '@nestjs/common';
import { DictionaryService } from './Dictionary.service';
import { OpenMctPod } from '@hyped/telemetry-types';

@Controller('openmct/dictionary')
export class DictionaryController {
  constructor(private dictionaryService: DictionaryService) {}

  @Get('pods')
  getPodIds() {
    const ids = this.dictionaryService.getPodIds();
    return {
      ids,
    };
  }

  @Get('pod/:podId')
  getPod(@Param('podId') podId: string): OpenMctPod {
    return this.dictionaryService.getPod(podId);
  }
}
