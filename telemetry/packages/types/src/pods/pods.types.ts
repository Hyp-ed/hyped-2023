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