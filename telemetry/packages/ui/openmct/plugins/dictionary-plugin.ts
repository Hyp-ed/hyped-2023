import { OpenMCT } from "openmct/dist/openmct"

export function DictionaryPlugin() {
  return function install(openmct: OpenMCT) {
    openmct.objects.addRoot({
      namespace: 'hyped',
      key: 'pod_1'
    })
  }
}