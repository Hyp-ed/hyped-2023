import {
  PodStateType,
  failureStates,
  okayStates,
  staticStates,
} from '@hyped/telemetry-constants';

/**
 * Returns the node type based on the state of the pod (okay, failure, static)
 * @param state The PodState of the pod
 * @returns The node type (okayNode, failureNode, defaultNode)
 */
export const getNodeType = (state: PodStateType) => {
  if (state in failureStates) return 'failureNode';
  if (state in staticStates) return 'defaultNode';
  if (state in okayStates) return 'okayNode';
};
