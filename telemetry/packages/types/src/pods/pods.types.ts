export type BaseMeasurement = {
  name: string;
  key: string;
  unit: string;
  type: string;
}

export type Limits = {
  warning?: {
    min: number;
    max: number;
  };
  critical: {
    min: number;
    max: number;
  };
}

export type RangeMeasurement = BaseMeasurement & {  
  format: 'float' | 'integer';
  limits: Limits;
}

export type EnumMeasurement = BaseMeasurement & {
  format: 'enum';
  enumerations: {
    value: number;
    string: string;
  }[];
}

export type Measurement = RangeMeasurement | EnumMeasurement;

export type Pod = {
  name: string;
  id: string;
  measurements: Record<string, Measurement>;
}

export type Pods = Record<string, Pod>;