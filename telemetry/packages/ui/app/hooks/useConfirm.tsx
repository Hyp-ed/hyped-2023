import { useEffect, useState } from 'react';
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
} from '@/components/ui/dialog';
import { Button } from '@/components/ui/button';
import { useCountdown } from 'usehooks-ts';

const COUNTDOWN = 100;
const INTERVAL = 1000;

type PromiseResolve = {
  resolve: (value: boolean) => void;
};

export const useConfirm = () => {
  const [promise, setPromise] = useState<PromiseResolve | null>(null);
  const [timeLeft, { startCountdown, stopCountdown, resetCountdown }] =
    useCountdown({
      countStart: COUNTDOWN,
      intervalMs: INTERVAL,
    });

  console.log('HEREE')

  const confirm = () => {
    startCountdown();
    return new Promise((resolve, _reject) => {
      setPromise({ resolve });
    });
  };

  const handleClose = () => {
    setPromise(null);
  };

  const handleConfirm = () => {
    (promise as PromiseResolve).resolve(true);
    handleClose();
  };

  const handleCancel = () => {
    (promise as PromiseResolve).resolve(false);
    handleClose();
  };

  const ConfirmationDialog = () => (
    <Dialog modal open={promise !== null}>
      <DialogContent>
        <DialogHeader>
          <DialogTitle>TEST</DialogTitle>
        </DialogHeader>
        <DialogFooter>
          <Button onClick={handleCancel} variant="outline">
            Cancel
          </Button>
          <Button onClick={handleConfirm}>Confirm</Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );

  return { ConfirmationDialog, confirm };
};
