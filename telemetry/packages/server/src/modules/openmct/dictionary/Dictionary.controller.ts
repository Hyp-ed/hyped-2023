import { Controller, Get } from '@nestjs/common';
import { DictionaryService } from './Dictionary.service';

@Controller('openmct/dictionary')
export class DictionaryController {
  constructor(private dictionaryService: DictionaryService) {}

  @Get()
  getDictionary() {
    const pod = this.dictionaryService.getDictionary();
    return pod;
  }
}
