export type Measurement =
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
      enumerations?: never;
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
      range?: never;
    };
