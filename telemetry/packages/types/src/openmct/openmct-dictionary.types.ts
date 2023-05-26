export type OpenMctMeasurement = {
  name: string;
  key: string;
  type: string;
  values: {
    key: string;
    name: string;
    unit?: string;
    format: string;
    min?: number;
    max?: number;
    enumerations?: {
      key: string;
      value: string;
    }[];
    hints?: {
      range: number;
    };
    source?: string;
    units?: {
      domain: string;
    };
  }[];
};

export type OpenMctPod = {
  name: string;
  id: number;
  measurements: OpenMctMeasurement[];
};

export type OpenMctDictionary = Record<string, OpenMctPod>;