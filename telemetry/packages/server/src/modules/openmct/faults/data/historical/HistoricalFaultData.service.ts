import { flux, fluxBool, fluxDateTime, fluxString } from '@influxdata/influxdb-client';
import { Injectable, LoggerService } from '@nestjs/common';
import { INFLUX_FAULTS_BUCKET, INFLUX_TELEMETRY_BUCKET } from '@/core/config';
import { InfluxService } from '@/modules/influx/Influx.service';
import { Logger } from '@/modules/logger/Logger.decorator';

type InfluxRow = {
  _time: string;
  _value: string;
  measurementKey: string;
  podId: string;
  format: string;
};

type GetHistoricalFaultForMeasurementInput = {
  podId: string;
  measurementKey: string;
  startTimestamp: string;
  endTimestamp: string;
};

type GetHistoricalFaultForMeasurementOptions = {
  getAcknowledged?: boolean;
}

@Injectable()
export class HistoricalFaultDataService {
  constructor(
    private influxService: InfluxService,
    @Logger()
    private readonly logger: LoggerService,
  ) {}

  public async getHistoricalFaultForMeasurement(props: GetHistoricalFaultForMeasurementInput, options: GetHistoricalFaultForMeasurementOptions = {}) {
    const {
      podId,
      measurementKey,
      startTimestamp,
      endTimestamp,
    } = props

    const {
      getAcknowledged = false
    } = options;

    const fluxStart = fluxDateTime(
      new Date(parseInt(startTimestamp)).toISOString(),
    );
    const fluxEnd = fluxDateTime(
      new Date(parseInt(endTimestamp)).toISOString(),
    );

    const query = flux`
      from(bucket: "${INFLUX_FAULTS_BUCKET}")
        |> range(start: ${fluxStart}, stop: ${fluxEnd})
        |> filter(fn: (r) => r["measurementKey"] == ${fluxString(measurementKey)})
        |> filter(fn: (r) => r["podId"] == ${fluxString(podId)})
        |> filter(fn: (r) => r["acknowledged"] == ${fluxBool(getAcknowledged)})`;

    try {
      const data = await this.influxService.query.collectRows<InfluxRow>(query);

      return data.map((row) => ({
        id: row['measurementKey'],
        timestamp: new Date(row['_time']).getTime(),
        value: row['_value'],
      }));
    } catch (e) {
      this.logger.error(
        `Failed to get historical reading for {${podId}/${measurementKey}}`,
        e,
        HistoricalFaultDataService.name,
      );
    }
  }
}
