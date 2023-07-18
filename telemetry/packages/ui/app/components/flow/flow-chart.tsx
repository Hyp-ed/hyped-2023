import ReactFlow, {
  Background,
  Edge,
  EdgeMarkerType,
  MarkerType,
  Node,
  NodeTypes,
  Position,
} from 'reactflow';
import 'reactflow/dist/style.css';
import {
  PodState,
  failureStates,
  okayStates,
  podStates,
  staticStates,
} from '@/types/PodState';
import { defaultNode, failureNode, okayNode } from './nodes';
import { useMemo } from 'react';
import './styles.css';

export type NodeDataType = {
  label: string;
  sourcePositions?: {
    position: Position;
    id: string;
  }[];
  targetPositions?: {
    position: Position;
    id: string;
  }[];
  active?: boolean;
};

const arrow: EdgeMarkerType = {
  type: MarkerType.Arrow,
  width: 32,
  height: 32,
  strokeWidth: 0.5,
};

const edges: Edge[] = [
  {
    id: 'idle-calibrating',
    source: 'idle',
    target: 'calibrating',
    type: 'step',
    markerEnd: arrow,
  },
  {
    id: 'calibrating-ready',
    source: 'calibrating',
    target: 'ready',
    sourceHandle: 'right',
    type: 'step',
    markerEnd: arrow,
  },
  {
    id: 'calibrating-failure-calibrating',
    source: 'calibrating',
    target: 'failure-calibrating',
    sourceHandle: 'top',
    type: 'step',
    markerEnd: arrow,
  },
  {
    id: 'ready-accelerating',
    source: 'ready',
    target: 'accelerating',
    type: 'step',
    markerEnd: arrow,
  },
  {
    id: 'accelerating-crusing',
    source: 'accelerating',
    target: 'crusing',
    type: 'step',
    markerEnd: arrow,
  },
  {
    id: 'accelerating-failure-braking',
    source: 'accelerating',
    target: 'failure-braking',
    sourceHandle: 'top',
    type: 'step',
    markerEnd: arrow,
  },
  {
    id: 'cruising-nominal-braking',
    source: 'cruising',
    target: 'nominal-braking',
    type: 'step',
    markerEnd: arrow,
  },
  {
    id: 'cruising-failure-braking',
    source: 'cruising',
    target: 'failure-braking',
    sourceHandle: 'top',
    type: 'step',
    markerEnd: arrow,
  },
  {
    id: 'nominal-braking-stopped',
    source: 'nominal-braking',
    target: 'stopped',
    type: 'step',
    markerEnd: arrow,
  },
  {
    id: 'nominal-braking-failure-braking',
    source: 'nominal-braking',
    target: 'failure-braking',
    sourceHandle: 'top',
    type: 'step',
    markerEnd: arrow,
  },
  {
    id: 'stopped-off',
    source: 'stopped',
    target: 'off',
    type: 'step',
    markerEnd: arrow,
  },
  {
    id: 'failure-braking-failure-stopped',
    source: 'failure-braking',
    target: 'failure-stopped',
    type: 'step',
    markerEnd: arrow,
  },
  {
    id: 'failure-stopped-off',
    source: 'failure-stopped',
    target: 'off',
    targetHandle: 'top',
    type: 'step',
    markerEnd: arrow,
  },
];

const getNodeType = (state: PodState) => {
  if (state in failureStates) return 'failureNode';
  if (state in staticStates) return 'defaultNode';
  if (state in okayStates) return 'okayNode';
};

type CustomNodeType = Omit<Node, 'data'> & {
  data: NodeDataType;
};

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
        id: 'cruising',
        data: {
          label: 'Cruising',
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
          active: currentState === podStates.cruising,
        },
        position: {
          x: 800,
          y: 200,
        },
        type: getNodeType(podStates.cruising),
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
          x: 1000,
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
          x: 1200,
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
          x: 1400,
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
          x: 1200,
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
          x: 1400,
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
