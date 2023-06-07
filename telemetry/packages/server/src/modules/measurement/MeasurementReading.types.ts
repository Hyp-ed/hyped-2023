import { z } from 'zod';

export const MeasurementReadingSchema = z.object({
  podId: z.string(),
  measurementKey: z.string(),
  value: z.number(),
});

export type MeasurementReading = z.infer<typeof MeasurementReadingSchema>;
