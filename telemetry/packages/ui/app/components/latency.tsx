import { usePods } from '@/context/pods';
import { POD_CONNECTION_STATUS } from '@/types/PodConnectionStatus';

/**
 * Displays the latency between the base station and the pod
 */
export const Latency = ({ podId }: { podId: string }) => {
  const { latency, connectionStatus } = usePods(podId);

  return connectionStatus === POD_CONNECTION_STATUS.CONNECTED ? (
    <p>
      <span className="">Latency: </span>
      <span className="text-sm">{latency} ms</span>
    </p>
  ) : (
    <p>Latency: N/A</p>
  );
};
