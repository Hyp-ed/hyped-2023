import { DomainObject } from "openmct/dist/src/api/objects/ObjectAPI";

export type AugmentedDomainObject = DomainObject & {
  podId: string;
}