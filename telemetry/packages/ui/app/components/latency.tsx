import { useLatency } from '@/hooks/useLatency';

/**
 * Displays the latency between the base station and the pod
 */
export const Latency = ({ podId }: { podId: string }) => {
  const { latency } = useLatency(podId);

  return (
    <p>
      <span className="">Latency: {latency}</span>
      <span className="text-sm">{latency} ms</span>
    </p>
  );
};
