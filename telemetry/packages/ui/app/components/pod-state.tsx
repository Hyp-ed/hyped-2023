import { cn } from '@/lib/utils';
import {
  PodState,
  failureStates,
  staticStates,
  nullStates,
  okayStates,
} from '@/types/PodState';
import { Button } from './ui/button';
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '@/components/ui/dialog';
import { StateMachineFlowChart } from './flow/flow-chart';

export const PodStateIndicator = ({ state }: { state: PodState }) => (
  <Dialog>
    <DialogTrigger asChild>
      <Button
        className={cn(
          'px-3 py-2 rounded-md w-full justify-start',
          state in nullStates ||
            (state in staticStates &&
              'bg-gray-600 hover:bg-gray-700 border-2 border-gray-800 text-white'),
          state in okayStates &&
            'bg-green-700 hover:bg-green-800 border-2 border-green-900 text-white',
          state in failureStates &&
            'bg-red-700 hover:bg-red-800 border-2 border-red-900 text-white',
        )}
      >
        <p>
          STATE: <b>{state.toUpperCase()}</b>
        </p>
      </Button>
    </DialogTrigger>
    <DialogContent className="min-w-[95%]">
      <DialogHeader>
        <DialogTitle>State Machine Flow Chart</DialogTitle>
      </DialogHeader>
      <StateMachineFlowChart currentState={state} />
      <DialogFooter>
        <DialogTrigger asChild>
          <Button>Close</Button>
        </DialogTrigger>
      </DialogFooter>
    </DialogContent>
  </Dialog>
);
