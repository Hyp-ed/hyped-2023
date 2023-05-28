export function getPodIdFromKey(key: string): string {
  const podId = key.split('_')[1]
  return podId
}