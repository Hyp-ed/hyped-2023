import objectTypes from '@hyped/telemetry-constants/object-types.json';
import { Injectable } from '@nestjs/common';

@Injectable()
export class ObjectTypesService {
  getObjectTypes() {
    return objectTypes;
  }
}
