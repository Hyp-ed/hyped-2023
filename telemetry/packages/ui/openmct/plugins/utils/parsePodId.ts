export function parsePodId(key: string): string {
  const podId = key.split('_')[1]
  return podId
}