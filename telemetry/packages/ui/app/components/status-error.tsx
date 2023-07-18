import { StatusType } from '@/types/StatusType';
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
  AlertDialogTrigger,
} from '@/components/ui/alert-dialog';
import { useEffect, useState } from 'react';

export const StatusError = ({ status }: { status: StatusType }) => {
  const [open, setOpen] = useState(false);

  useEffect(() => {
    if (status === 'disconnected' || status === 'reconnecting') {
      setOpen(true);
    } else {
      setOpen(false);
    }
  }, [status]);

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
            onClick={() => setOpen(false)}
          >
            Okay
          </AlertDialogAction>
        </AlertDialogFooter>
      </AlertDialogContent>
    </AlertDialog>
  );
};
