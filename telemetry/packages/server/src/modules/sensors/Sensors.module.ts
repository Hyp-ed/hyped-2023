import { Module } from '@nestjs/common';
import { DictionaryController } from './dictionary/Dictionary.controller';
import { DictionaryService } from './dictionary/Dictionary.service';

@Module({
  controllers: [DictionaryController],
  providers: [DictionaryService],
})
export class SensorsModule {}
