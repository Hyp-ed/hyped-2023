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
import { useState } from 'react';

interface ErrorStateAlertProps {
  title: string;
  description: string;
}

export const ErrorStateAlert = ({
  title,
  description,
}: ErrorStateAlertProps) => {
  const [open, setOpen] = useState(true);
  return (
    // @ts-ignore
    <AlertDialog open={open}>
      {/* @ts-ignore */}
      <AlertDialogContent>
        <AlertDialogHeader>
          {/* @ts-ignore */}
          <AlertDialogTitle>{title}</AlertDialogTitle>
          {/* @ts-ignore */}
          <AlertDialogDescription>{description}</AlertDialogDescription>
        </AlertDialogHeader>
        <AlertDialogFooter>
          {/* @ts-ignore */}
          <AlertDialogCancel
            className="bg-red-700 hover:bg-red-800"
            onClick={() => setOpen(!open)}
          >
            Okay
          </AlertDialogCancel>
        </AlertDialogFooter>
      </AlertDialogContent>
    </AlertDialog>
  );
};
