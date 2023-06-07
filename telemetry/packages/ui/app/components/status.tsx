import { StatusType } from '@/types/StatusType';

interface StatusIndicatorProps {
  status: StatusType;
}

export const StatusIndicator = ({ status }: StatusIndicatorProps) => (
  <div className="flex gap-2 items-center">
    <div
      className={`w-2 h-2 rounded-full
              ${
                status == 'connected'
                  ? 'bg-green-500 animate-[pulse_linear_1s_infinite]'
                  : status == 'connecting'
                  ? 'bg-orange-500 animate-[pulse_linear_0.5s_infinite]'
                  : status == 'disconnected'
                  ? 'bg-red-500'
                  : ''
              }`}
    />
    <p
      className={`text-sm italic ${
        status == 'connected'
          ? 'text-green-500'
          : status == 'connecting'
          ? 'text-orange-500'
          : status == 'disconnected'
          ? 'text-red-500'
          : ''
      }`}
    >
      {status == 'connected'
        ? 'Connected'
        : status == 'connecting'
        ? 'Connecting...'
        : status == 'disconnected'
        ? 'Disconnected'
        : ''}
    </p>
  </div>
);
