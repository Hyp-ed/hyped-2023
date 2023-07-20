import {
  PodConnectionStatusType,
  POD_CONNECTION_STATUS,
} from '@/types/PodConnectionStatus';
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
} from '@/components/ui/alert-dialog';
import { useEffect, useState } from 'react';

export const PodDisconnectError = ({
  status,
}: {
  status: PodConnectionStatusType;
}) => {
  const [open, setOpen] = useState(false);

  // Open dialog when pod disconnects or encounters an error
  useEffect(() => {
    setOpen(
      status === POD_CONNECTION_STATUS.DISCONNECTED ||
        status === POD_CONNECTION_STATUS.ERROR,
    );
  }, [status]);

  const close = () => setOpen(false);

  return (
    <AlertDialog open={open}>
      <AlertDialogContent>
        <AlertDialogHeader>
          <AlertDialogTitle>Pod disconnected!</AlertDialogTitle>
          <AlertDialogDescription>
            Lost MQTT connection to pod.
          </AlertDialogDescription>
        </AlertDialogHeader>
        <AlertDialogFooter>
          <AlertDialogAction
            className="bg-red-700 hover:bg-red-800 text-white"
            onClick={close}
          >
            Okay
          </AlertDialogAction>
        </AlertDialogFooter>
      </AlertDialogContent>
    </AlertDialog>
  );
};
