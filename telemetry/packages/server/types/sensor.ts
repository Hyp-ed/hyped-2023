export type Sensor =
  | {
      name: string;
      key: string;
      format: string;
      type: string;
      units: string;
      range: {
        min: number;
        max: number;
      };
      enumerations?: undefined;
    }
  | {
      name: string;
      key: string;
      format: string;
      type: string;
      units: string;
      enumerations: {
        key: string;
        value: number;
      }[];
      range?: undefined;
    };
