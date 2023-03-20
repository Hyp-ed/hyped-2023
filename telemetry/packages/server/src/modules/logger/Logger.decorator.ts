import { applyDecorators, Inject } from '@nestjs/common';
import { WINSTON_MODULE_NEST_PROVIDER } from 'nest-winston';

export function Logger() {
  return applyDecorators(Inject(WINSTON_MODULE_NEST_PROVIDER));
}
