import { Controller, Get } from '@nestjs/common';
import { DictionaryService } from './Dictionary.service';

@Controller('openmct/dictionary')
export class DictionaryController {
  constructor(private dictionaryService: DictionaryService) {}

  @Get()
  getDictionary() {
    const pods = this.dictionaryService.getDictionary();
    return pods;
  }
}
