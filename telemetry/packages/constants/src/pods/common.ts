export const accelerometerCommon = {
  format: 'float',
  type: 'acceleration',
  unit: 'm/s²',
  limits: {
    critical: {
      low: -150,
      high: 150,
    },
  },
} as const;

export const thermistorCommon = {
  format: 'float',
  type: 'thermistor',
  unit: '°C',
  limits: {
    critical: {
      low: 15,
      high: 120,
    },
  },
} as const;

export const pressureCommon = {
  format: 'float',
  type: 'pressure',
  unit: 'bar',
  limits: {
    critical: {
      low: 3.2,
      high: 3.7,
    },
  },
} as const;

export const hallEffectCommon = {
  format: 'float',
  type: 'hall_effect',
  unit: 'A',
  limits: {
    critical: {
      low: 0,
      high: 500,
    },
  },
} as const;

export const keyenceCommon = {
  format: 'integer',
  type: 'keyence',
  unit: 'number of stripes',
  limits: {
    critical: {
      low: 0,
      high: 16,
    },
  },
} as const;