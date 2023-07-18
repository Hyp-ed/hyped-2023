import openmct from 'openmct/dist/openmct';

import { DictionaryPlugin } from './plugins/dictionary-plugin';
import { HistoricalTelemetryPlugin } from './plugins/historical-telemetry-plugin';
import { RealtimeTelemetryPlugin } from './plugins/realtime-telemetry-plugin';
import { LimitPlugin } from './plugins/limit-plugin';

const timeWindow = 10 * 1000;

openmct.setAssetPath('/openmct-lib');

// Local storage of dashbaords
openmct.install(openmct.plugins.LocalStorage());

// Time
openmct.install(openmct.plugins.UTCTimeSystem());
openmct.time.clock('local', { start: -timeWindow, end: 0 });
openmct.time.timeSystem('utc');

// Theme
openmct.install(openmct.plugins['Espresso']());

// Data
openmct.install(DictionaryPlugin());
openmct.install(HistoricalTelemetryPlugin());
openmct.install(RealtimeTelemetryPlugin());

// Limits
openmct.install(LimitPlugin());

// Views
openmct.install(openmct.plugins.MyItems());
openmct.install(openmct.plugins.PlanLayout());
openmct.install(openmct.plugins.Timeline());
openmct.install(openmct.plugins.Hyperlink());
openmct.install(openmct.plugins.AutoflowView({
    type: "telemetry.panel"
}));
openmct.install(openmct.plugins.DisplayLayout({
    showAsView: ['summary-widget', 'example.imagery']
}));
openmct.install(openmct.plugins.LADTable());
openmct.install(openmct.plugins.SummaryWidget());
openmct.install(openmct.plugins.Timer());
openmct.install(openmct.plugins.Timelist());
openmct.install(openmct.plugins.BarChart());
openmct.install(openmct.plugins.ScatterPlot());

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

openmct.start();
