import { objectTypes } from '@hyped/telemetry-constants';
import { Injectable } from '@nestjs/common';

@Injectable()
export class ObjectTypesService {
  getObjectTypes() {
    return objectTypes;
  }
}
