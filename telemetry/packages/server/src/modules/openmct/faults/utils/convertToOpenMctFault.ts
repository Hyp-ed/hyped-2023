import { OpenMctFault } from '@hyped/telemetry-types';
import { nanoid } from 'nanoid';
import { Fault } from '../Fault.service';

export function convertToOpenMctFault(
  fault: Fault
): OpenMctFault {
  const { measurement, tripReading, level } = fault;

  const namespace = `/${tripReading.podId}/${measurement.key}`;
  const triggerDate = new Date(Number(tripReading.timestamp) / 1000000);

  return {
    type: 'global-alarm-status',
    fault: {
      id: `${namespace}-${nanoid()}`,
      name: `${measurement.name} is out of range`,
      namespace,
      seqNum: 0,
      severity: level,
      shortDescription: '',
      shelved: false,
      acknowledged: false,
      // convert unix timestamp to HH:mm:ss
      triggerTime: `${triggerDate.getHours()}:${triggerDate.getMinutes()}:${triggerDate.getSeconds()}`,
      triggerValueInfo: {
        value: tripReading.value,
        rangeCondition: level,
        monitoringResult: level,
      },
      currentValueInfo: {
        value: tripReading.value,
        rangeCondition: level,
        monitoringResult: level,
      },
    },
  };
}
