import { Controller, Get, Param } from '@nestjs/common';
import { DictionaryService } from './Dictionary.service';

@Controller('openmct/dictionary')
export class DictionaryController {
  constructor(private dictionaryService: DictionaryService) {}

  @Get('pods')
  getPodIds() {
    const podIds = this.dictionaryService.getPodIds();
    return {
      podIds,
    };
  }

  @Get('pod/:podId')
  getPod(@Param('podId') podId: string) {
    return this.dictionaryService.getPod(podId);
  }
}
