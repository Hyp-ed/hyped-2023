import ReactFlow, { Position } from 'reactflow';
import 'reactflow/dist/style.css';
import { PodState, podStates } from '@/types/PodState';
import { defaultNode, failureNode, okayNode } from './nodes';
import { useMemo } from 'react';
import './styles.css';
import { getNodeType } from './utils';
import { edges } from './edges';
import { CustomNodeType } from './types';

export function StateMachineFlowChart({
  currentState,
}: {
  currentState: PodState;
}) {
  const nodeTypes = useMemo(
    () => ({
      failureNode,
      defaultNode,
      okayNode,
    }),
    [],
  );

  const nodes: CustomNodeType[] = useMemo(
    () => [
      {
        id: 'idle',
        data: {
          label: 'Idle',
          sourcePositions: [
            {
              position: Position.Right,
              id: 'right',
            },
          ],
          active: currentState === podStates.idle,
        },
        position: { x: 0, y: 200 },
        type: getNodeType(podStates.idle),
      },
      {
        id: 'calibrating',
        data: {
          label: 'Calibrating',
          sourcePositions: [
            {
              position: Position.Right,
              id: 'right',
            },
            {
              position: Position.Top,
              id: 'top',
            },
          ],
          targetPositions: [
            {
              position: Position.Left,
              id: 'left',
            },
          ],
          active: currentState === podStates.calibrating,
        },
        position: { x: 200, y: 200 },
        type: getNodeType(podStates.calibrating),
      },
      {
        id: 'failure-calibrating',
        data: {
          label: 'Failure Calibrating',
          targetPositions: [
            {
              position: Position.Bottom,
              id: 'bottom',
            },
          ],
          active: currentState === podStates.failureCalibrating,
        },
        position: { x: 200, y: 0 },
        type: getNodeType(podStates.failureCalibrating),
      },
      {
        id: 'ready',
        data: {
          label: 'Ready',
          sourcePositions: [
            {
              position: Position.Right,
              id: 'right',
            },
          ],
          targetPositions: [
            {
              position: Position.Left,
              id: 'left',
            },
          ],
          active: currentState === podStates.ready,
        },
        position: {
          x: 400,
          y: 200,
        },
        type: getNodeType(podStates.ready),
      },
      {
        id: 'accelerating',
        data: {
          label: 'Accelerating',
          targetPositions: [
            {
              position: Position.Left,
              id: 'left',
            },
          ],
          sourcePositions: [
            {
              position: Position.Right,
              id: 'right',
            },
            {
              position: Position.Top,
              id: 'top',
            },
          ],
          active: currentState === podStates.accelerating,
        },
        position: {
          x: 600,
          y: 200,
        },
        type: getNodeType(podStates.accelerating),
      },
      {
        id: 'nominal-braking',
        data: {
          label: 'Nominal Braking',
          targetPositions: [
            {
              position: Position.Left,
              id: 'left',
            },
          ],
          sourcePositions: [
            {
              position: Position.Right,
              id: 'right',
            },
            {
              position: Position.Top,
              id: 'top',
            },
          ],
          active: currentState === podStates.frictionBraking, // or podStates.motorBraking?
        },
        position: {
          x: 800,
          y: 200,
        },
        type: getNodeType(podStates.frictionBraking),
      },
      {
        id: 'stopped',
        data: {
          label: 'Stopped',
          sourcePositions: [
            {
              position: Position.Right,
              id: 'right',
            },
          ],
          targetPositions: [
            {
              position: Position.Left,
              id: 'left',
            },
          ],
          active: currentState === podStates.stopped,
        },
        position: {
          x: 1000,
          y: 200,
        },
        type: getNodeType(podStates.stopped),
      },
      {
        id: 'off',
        data: {
          label: 'Off',
          targetPositions: [
            {
              position: Position.Left,
              id: 'left',
            },
            {
              position: Position.Top,
              id: 'top',
            },
          ],
          active: currentState === podStates.off,
        },
        position: {
          x: 1200,
          y: 200,
        },
        type: getNodeType(podStates.off),
      },
      {
        id: 'failure-braking',
        data: {
          label: 'Failure Braking',
          sourcePositions: [
            {
              position: Position.Right,
              id: 'right',
            },
          ],
          targetPositions: [
            {
              position: Position.Left,
              id: 'left',
            },
          ],
          active: currentState === podStates.failureBraking,
        },
        position: {
          x: 1000,
          y: 0,
        },
        type: getNodeType(podStates.failureBraking),
      },
      {
        id: 'failure-stopped',
        data: {
          label: 'Failure Stopped',
          targetPositions: [
            {
              position: Position.Left,
              id: 'left',
            },
          ],
          sourcePositions: [
            {
              position: Position.Bottom,
              id: 'bottom',
            },
          ],
          active: currentState === podStates.failureStopped,
        },
        position: {
          x: 1200,
          y: 0,
        },
        type: getNodeType(podStates.failureStopped),
      },
    ],
    [currentState],
  );

  return (
    <div className="min-h-[350px] pt-8">
      <ReactFlow
        nodes={nodes}
        edges={edges}
        nodeTypes={nodeTypes}
        // remove all interactivity
        panOnDrag={false}
        panOnScroll={false}
        zoomOnScroll={false}
        elementsSelectable={false}
        nodesDraggable={false}
        nodesConnectable={false}
        zoomOnPinch={false}
        zoomOnDoubleClick={false}
      />
    </div>
  );
}
//
