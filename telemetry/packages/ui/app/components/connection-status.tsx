import { StatusType } from '@/types/StatusType';

interface ConnectionStatusProps {
  status: StatusType;
}

export const ConnectionStatus = ({ status }: ConnectionStatusProps) => (
  <div className="flex gap-2 items-center">
    <div
      className={`w-2 h-2 rounded-full
              ${
                status === 'connected'
                  ? 'bg-green-500 animate-[pulse_linear_1s_infinite]'
                  : status === 'connecting' ||
                    status === 'waiting' ||
                    status === 'reconnecting'
                  ? 'bg-orange-500 animate-[pulse_linear_0.5s_infinite]'
                  : status === 'disconnected' || status === 'error'
                  ? 'bg-red-500'
                  : ''
              }`}
    />
    <p
      className={`text-sm italic ${
        status == 'connected'
          ? 'text-green-500'
          : status == 'connecting' ||
            status == 'waiting' ||
            status == 'reconnecting'
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
        : status == 'waiting'
        ? 'Waiting...'
        : status == 'error'
        ? 'Error'
        : status == 'reconnecting'
        ? 'Trying to reconnect...'
        : status}
    </p>
  </div>
);
