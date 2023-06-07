// E.g. pod_1 -> 1
export function getPodId(fullPodId: string): number | null {
  const regex = /^pod_(\d+)$/;
  const match = regex.exec(fullPodId);
  if (match && match.length > 1) {
    return parseInt(match[1], 10);
  }

  return null;
}
