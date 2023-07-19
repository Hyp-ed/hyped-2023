import openmct from 'openmct/dist/openmct';

import { DictionaryPlugin } from './plugins/dictionary-plugin';
import { HistoricalTelemetryPlugin } from './plugins/historical-telemetry-plugin';
import { RealtimeTelemetryPlugin } from './plugins/realtime-telemetry-plugin';
import { LimitPlugin } from './plugins/limit-plugin';

const TEN_SECONDS = 10 * 1000;
const THIRTY_SECONDS = 30 * 1000;
const ONE_MINUTE = THIRTY_SECONDS * 2;
const FIVE_MINUTES = ONE_MINUTE * 5;
const FIFTEEN_MINUTES = FIVE_MINUTES * 3;
const THIRTY_MINUTES = FIFTEEN_MINUTES * 2;
const ONE_HOUR = THIRTY_MINUTES * 2;
const TWO_HOURS = ONE_HOUR * 2;
const ONE_DAY = ONE_HOUR * 24;

openmct.setAssetPath('/openmct-lib');

// Local storage of dashbaords
openmct.install(openmct.plugins.LocalStorage());

// Time
openmct.install(openmct.plugins.UTCTimeSystem());
openmct.time.clock('local', { start: -TEN_SECONDS, end: 0 });

// Theme
openmct.install(openmct.plugins['Espresso']());

// Views
openmct.install(openmct.plugins.MyItems());
openmct.install(openmct.plugins.PlanLayout());
openmct.install(openmct.plugins.Timeline());
openmct.install(openmct.plugins.Hyperlink());
openmct.install(
  openmct.plugins.AutoflowView({
    type: 'telemetry.panel',
  }),
);
openmct.install(
  openmct.plugins.DisplayLayout({
    showAsView: ['summary-widget', 'example.imagery'],
  }),
);
openmct.install(openmct.plugins.LADTable());
openmct.install(openmct.plugins.SummaryWidget());
openmct.install(openmct.plugins.Timer());
openmct.install(openmct.plugins.Timelist());
openmct.install(openmct.plugins.BarChart());
openmct.install(openmct.plugins.ScatterPlot());

openmct.install(
  openmct.plugins.Conductor({
    menuOptions: [
      {
        name: 'Realtime',
        timeSystem: 'utc',
        clock: 'local',
        clockOffsets: {
          start: -THIRTY_MINUTES,
          end: THIRTY_SECONDS,
        },
        presets: [
          {
            label: '1 Hour',
            bounds: {
              start: -ONE_HOUR,
              end: THIRTY_SECONDS,
            },
          },
          {
            label: '30 Minutes',
            bounds: {
              start: -THIRTY_MINUTES,
              end: THIRTY_SECONDS,
            },
          },
          {
            label: '15 Minutes',
            bounds: {
              start: -FIFTEEN_MINUTES,
              end: THIRTY_SECONDS,
            },
          },
          {
            label: '5 Minutes',
            bounds: {
              start: -FIVE_MINUTES,
              end: THIRTY_SECONDS,
            },
          },
          {
            label: '1 Minute',
            bounds: {
              start: -ONE_MINUTE,
              end: THIRTY_SECONDS,
            },
          },
        ],
      },
      {
        name: 'Fixed',
        timeSystem: 'utc',
        bounds: {
          start: Date.now() - THIRTY_MINUTES,
          end: Date.now(),
        },
        // commonly used bounds can be stored in history
        // bounds (start and end) can accept either a milliseconds number
        // or a callback function returning a milliseconds number
        // a function is useful for invoking Date.now() at exact moment of preset selection
        presets: [
          {
            label: 'Last Day',
            bounds: {
              start: () => Date.now() - ONE_DAY,
              end: () => Date.now(),
            },
          },
          {
            label: 'Last 2 hours',
            bounds: {
              start: () => Date.now() - TWO_HOURS,
              end: () => Date.now(),
            },
          },
          {
            label: 'Last hour',
            bounds: {
              start: () => Date.now() - ONE_HOUR,
              end: () => Date.now(),
            },
          },
        ],
        // maximum recent bounds to retain in conductor history
        records: 10,
        // maximum duration between start and end bounds
        // for utc-based time systems this is in milliseconds
        // limit: ONE_DAY
      },
    ],
  }),
);

// Data
openmct.install(DictionaryPlugin());
openmct.install(HistoricalTelemetryPlugin());
openmct.install(RealtimeTelemetryPlugin());

// Limits
openmct.install(LimitPlugin());

function sleep(ms: number) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

// Wait for all plugins to be installed before starting
sleep(1000).then(() => {
  openmct.start();
});
