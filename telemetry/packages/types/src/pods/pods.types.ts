import { type } from "os";

export const POD_IDS = ['1', '2'] as const;

export type BaseMeasurement = {
  name: string;
  key: string;
  unit: string;
  type: string;
}

export type RangeMeasurement = BaseMeasurement & {  
  format: 'float' | 'integer';
  range: {
    min: number;
    max: number;
  };
}

export type EnumMeasurement = BaseMeasurement & {
  format: 'enum';
  enumerations: {
    key: string;
    value: string;
  }[];
}

export type Measurement = RangeMeasurement | EnumMeasurement;

export type Pod = {
  name: string;
  podId: number;
  measurements: Record<string, Measurement>;
}

export type Pods = Record<typeof POD_IDS[number], Pod>;