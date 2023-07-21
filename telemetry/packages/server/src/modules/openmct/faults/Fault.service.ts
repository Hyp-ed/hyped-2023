import { InfluxService } from '@/modules/influx/Influx.service';
import { Logger } from '@/modules/logger/Logger.decorator';
import { MeasurementReading } from '@/modules/measurement/MeasurementReading.types';
import { FaultLevel } from '@hyped/telemetry-constants';
import { RangeMeasurement } from '@hyped/telemetry-types/dist/pods/pods.types';
import { Injectable, LoggerService } from '@nestjs/common';
import { HistoricalFaultDataService } from './data/historical/HistoricalFaultData.service';
import { nanoid } from 'nanoid'
import { RealtimeFaultDataGateway } from './data/realtime/RealtimeFaultData.gateway';
import { toUnixTimestamp } from '@/modules/common/utils/toUnixTimestamp';
import { OpenMctFault } from '@hyped/telemetry-types';

type Fault = {
  level: FaultLevel;
  measurement: RangeMeasurement;
  tripReading: MeasurementReading;
};

@Injectable()
export class FaultService {
  constructor(
    @Logger()
    private readonly logger: LoggerService,
    private influxService: InfluxService,
    private historicalService: HistoricalFaultDataService,
    private realtimeService: RealtimeFaultDataGateway,
  ) {}

  public addLimitBreachFault(props: Fault) {
    const currentTime = new Date();

    const { level, measurement, tripReading } = props;

    // check if unacked fault already exists within timetorefault
    // if it does, update current value
    // or add it

    // const possibleExistingFault = this.historicalService.getHistoricalFaultForMeasurement({
    //   podId: tripReading.podId,
    //   measurementKey: measurement.key,
    //   startTimestamp: (currentTime.getTime() - TIME_TO_REFAULT).toString(),
    //   endTimestamp: currentTime.getTime().toString()
    // })

    const namespace = `/${tripReading.podId}/${measurement.key}}`
    const fault: OpenMctFault = {
      type: 'global-alarm-status', // to change
      fault: {
        acknowledged: false,
        currentValueInfo: {
          value: tripReading.value,
          rangeCondition: level,
          monitoringResult: level,
        },
        id: `id-${namespace}-${nanoid()}`,
        name: `${measurement.name} is out of range`,
        namespace: `/${tripReading.podId}/${measurement.key}}`,
        seqNum: 0,
        severity: level,
        shelved: false,
        shortDescription: '',
        triggerTime: toUnixTimestamp(currentTime).toString(),
        triggerValueInfo: {
          value: tripReading.value,
          rangeCondition: level,
          monitoringResult: level,
        },
      },
    };

    this.realtimeService.sendFault(fault);
  }
}
