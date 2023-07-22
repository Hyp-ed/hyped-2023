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
import { start } from 'repl';

const COUNTDOWN = 5;
const INTERVAL = 1000;

type StartDialogProps = {
  startCallback: () => void;
  isDialogOpen: boolean;
  closeDialog: () => void;
}

export const StartDialog = (props: StartDialogProps) => {
  const [timeLeft, { startCountdown, stopCountdown, resetCountdown }] =
    useCountdown({
      countStart: COUNTDOWN,
      intervalMs: INTERVAL,
    });

    useEffect(() => {
      startCountdown()
    }, [])

    useEffect(() => {
      if (timeLeft === 0) {
        props.startCallback()
        props.closeDialog()
      }
    }, [timeLeft])

  return (
    <Dialog modal open={props.isDialogOpen}>
      <DialogContent>
        {timeLeft}
        <DialogFooter>
          {/* <Button onClick={handleCancel} variant="outline">
            Cancel
          </Button>
          <Button onClick={handleConfirm}>Confirm</Button> */}
        </DialogFooter>
      </DialogContent>
    </Dialog>
  )};