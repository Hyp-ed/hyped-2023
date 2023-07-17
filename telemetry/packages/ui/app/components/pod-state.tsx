import { cn } from '@/lib/utils';
import {
  PodState,
  failureStates,
  idleStates,
  nullStates,
  okayStates,
} from '@/types/PodState';

export const PodStateIndicator = ({ state }: { state: PodState }) => (
  <div
    className={cn(
      'px-3 py-2 rounded-md',
      state in nullStates && 'bg-gray-600 border-2 border-gray-500',
      state in okayStates && 'bg-green-900 border-2 border-green-800',
      state in idleStates && 'bg-yellow-800 border-2 border-yellow-700',
      state in failureStates && 'bg-red-900 border-2 border-red-800',
    )}
  >
    <p>
      STATE: <b>{state}</b>
    </p>
  </div>
);
