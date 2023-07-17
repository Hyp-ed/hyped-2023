import openmct from 'openmct/dist/openmct';

import { DictionaryPlugin } from './plugins/dictionary-plugin';
import { HistoricalTelemetryPlugin } from './plugins/historical-telemetry-plugin';
import { RealtimeTelemetryPlugin } from './plugins/realtime-telemetry-plugin';

const timeWindow = 15 * 60 * 1000;

openmct.setAssetPath('/openmct-lib');

openmct.install(openmct.plugins.LocalStorage());
openmct.install(openmct.plugins.MyItems());
openmct.install(openmct.plugins.UTCTimeSystem());

openmct.time.clock('local', { start: -timeWindow, end: 0 });
openmct.time.timeSystem('utc');
openmct.install(openmct.plugins['Espresso']());

openmct.install(
  openmct.plugins.Conductor({
    menuOptions: [
      // Configuration for the LocalClock in the UTC time system
      {
        clock: 'local',
        timeSystem: 'utc',
        clockOffsets: { start: -timeWindow, end: 0 },
        zoomOutLimit: 31 * 24 * 60 * 60 * 1000,
        zoomInLimit: 60 * 1000,
      },
    ],
  }),
);

openmct.install(DictionaryPlugin());
openmct.install(HistoricalTelemetryPlugin());
openmct.install(RealtimeTelemetryPlugin());

openmct.start();
