import { usePods } from '@/context/pods';
import { cn } from '@/lib/utils';
import { POD_CONNECTION_STATUS } from '@/types/PodConnectionStatus';

/**
 * Displays the connection status of a pod
 * @param podId The ID of the pod
 */
export const PodConnectionStatus = ({ podId }: { podId: string }) => {
  const { connectionStatus } = usePods(podId);

  const CONNECTED = connectionStatus === POD_CONNECTION_STATUS.CONNECTED;
  const UNKNOWN = connectionStatus === POD_CONNECTION_STATUS.UNKNOWN;
  const DISCONNECTED = connectionStatus === POD_CONNECTION_STATUS.DISCONNECTED;
  const ERROR = connectionStatus === POD_CONNECTION_STATUS.ERROR;

  return (
    <div className="flex gap-2 items-center">
      <div
        className={cn(
          'w-2 h-2 rounded-full',
          CONNECTED && 'bg-green-500 animate-[pulse_linear_1s_infinite]',
          UNKNOWN && 'bg-orange-500 animate-[pulse_linear_0.5s_infinite]',
          (DISCONNECTED || ERROR) && 'bg-red-500',
        )}
      />
      <p
        className={cn(
          'text-sm italic',
          CONNECTED && 'text-green-500',
          UNKNOWN && 'text-orange-500',
          (DISCONNECTED || ERROR) && 'text-red-500',
        )}
      >
        {connectionStatus}
      </p>
    </div>
  );
};
