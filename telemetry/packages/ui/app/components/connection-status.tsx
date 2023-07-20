import { useLatency } from '@/hooks/useLatency';
import {
  PodConnectionStatusType,
  POD_CONNECTION_STATUS,
} from '@/types/PodConnectionStatus';

interface PodConnectionStatusProps {
  mqttStatus: PodConnectionStatusType;
  podId: string;
}

/**
 * Displays the connection status of a pod
 * @param mqttStatus The current connection status of the pod
 * @param podId The ID of the pod
 */
export const PodConnectionStatus = ({
  mqttStatus,
  podId,
}: PodConnectionStatusProps) => {
  const { latency } = useLatency(podId);

  return (
    <div className="flex gap-2 items-center">
      <div
        className={`w-2 h-2 rounded-full
              ${
                mqttStatus === 'CONNECTED'
                  ? 'bg-green-500 animate-[pulse_linear_1s_infinite]'
                  : mqttStatus === POD_CONNECTION_STATUS.CONNECTING ||
                    mqttStatus === POD_CONNECTION_STATUS.UNKNOWN ||
                    mqttStatus === POD_CONNECTION_STATUS.RECONNECTING
                  ? 'bg-orange-500 animate-[pulse_linear_0.5s_infinite]'
                  : mqttStatus === POD_CONNECTION_STATUS.DISCONNECTED ||
                    mqttStatus === POD_CONNECTION_STATUS.ERROR
                  ? 'bg-red-500'
                  : ''
              }`}
      />
      <p
        className={`text-sm italic ${
          mqttStatus == 'CONNECTED'
            ? 'text-green-500'
            : mqttStatus == POD_CONNECTION_STATUS.CONNECTING ||
              mqttStatus == POD_CONNECTION_STATUS.UNKNOWN ||
              mqttStatus == POD_CONNECTION_STATUS.RECONNECTING
            ? 'text-orange-500'
            : mqttStatus == POD_CONNECTION_STATUS.DISCONNECTED
            ? 'text-red-500'
            : ''
        }`}
      >
        {mqttStatus == 'CONNECTED'
          ? 'Connected'
          : mqttStatus == POD_CONNECTION_STATUS.CONNECTING
          ? 'Connecting...'
          : mqttStatus == POD_CONNECTION_STATUS.DISCONNECTED
          ? 'Disconnected'
          : mqttStatus == POD_CONNECTION_STATUS.UNKNOWN
          ? 'Unknown/waiting...'
          : mqttStatus == POD_CONNECTION_STATUS.ERROR
          ? 'Error'
          : mqttStatus == POD_CONNECTION_STATUS.RECONNECTING
          ? 'Trying to reconnect...'
          : mqttStatus}
      </p>
    </div>
  );
};
