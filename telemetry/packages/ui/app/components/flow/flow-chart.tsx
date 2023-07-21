import ReactFlow, { Position } from 'reactflow';
import 'reactflow/dist/style.css';
import { PodStateType, podStates } from '@hyped/telemetry-constants';
import { defaultNode, failureNode, okayNode, textNode } from './nodes';
import { useMemo } from 'react';
import './styles.css';
import { getNodeType } from './utils';
import { edges } from './edges';
import { CustomNodeType } from './types';

export function StateMachineFlowChart({
  currentState,
}: {
  currentState: PodStateType;
}) {
  const nodeTypes = useMemo(
    () => ({
      failureNode,
      defaultNode,
      okayNode,
      textNode,
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
          active: currentState === podStates.IDLE,
        },
        position: { x: 0, y: 200 },
        type: getNodeType(podStates.IDLE),
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
          active: currentState === podStates.CALIBRATING,
        },
        position: { x: 200, y: 200 },
        type: getNodeType(podStates.CALIBRATING),
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
          active: currentState === podStates.FAILURE_CALIBRATING,
        },
        position: { x: 200, y: 0 },
        type: getNodeType(podStates.FAILURE_CALIBRATING),
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
          active: currentState === podStates.READY,
        },
        position: {
          x: 400,
          y: 200,
        },
        type: getNodeType(podStates.READY),
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
          active: currentState === podStates.ACCELERATING,
        },
        position: {
          x: 600,
          y: 200,
        },
        type: getNodeType(podStates.ACCELERATING),
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
          active: currentState === podStates.NOMINAL_BRAKING, // or podStates.motorBraking?
        },
        position: {
          x: 800,
          y: 200,
        },
        type: getNodeType(podStates.NOMINAL_BRAKING),
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
          active: currentState === podStates.STOPPED,
        },
        position: {
          x: 1000,
          y: 200,
        },
        type: getNodeType(podStates.STOPPED),
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
          active: currentState === podStates.OFF,
        },
        position: {
          x: 1200,
          y: 200,
        },
        type: getNodeType(podStates.OFF),
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
          active: currentState === podStates.FAILURE_BRAKING,
        },
        position: {
          x: 1000,
          y: 0,
        },
        type: getNodeType(podStates.FAILURE_BRAKING),
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
          active: currentState === podStates.FAILURE_STOPPED,
        },
        position: {
          x: 1200,
          y: 0,
        },
        type: getNodeType(podStates.FAILURE_STOPPED),
      },
      {
        id: 'key-default-label',
        data: {
          label: 'Key:',
        },
        position: {
          x: 0,
          y: 360,
        },
        type: 'textNode',
      },
      {
        id: 'key-default',
        data: {
          label: 'Idle State',
        },
        position: {
          x: 0,
          y: 400,
        },
        type: 'defaultNode',
      },
      {
        id: 'key-okay',
        data: {
          label: 'Okay State',
        },
        position: {
          x: 175,
          y: 400,
        },
        type: 'okayNode',
      },
      {
        id: 'key-failure',
        data: {
          label: 'Failure State',
        },
        position: {
          x: 350,
          y: 400,
        },
        type: 'failureNode',
      },
    ],
    [currentState],
  );

  return (
    <div className="min-h-[500px] pt-8">
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
