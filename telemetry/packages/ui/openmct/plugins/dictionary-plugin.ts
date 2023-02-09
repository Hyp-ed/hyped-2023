import { OpenMCT } from "openmct/dist/openmct"
import { http } from "../core/http"
import { ObjectIdentitifer } from "../types/ObjectIdentifier"

function getDictionary() {
  return http.get('/dictionary').json().then((data) => {
    return data
  })
}

const objectProvider = {
  get: (identifier: ObjectIdentitifer) => {
    return getDictionary().then((dictionary: any) => {
      if (identifier.key === 'pod_1') {
        return {
          identifier,
          name: dictionary.name,
          type: 'folder',
          location: 'ROOT'
        }
      }
    })
  }
}

export function DictionaryPlugin() {
  return function install(openmct: OpenMCT) {
    openmct.objects.addRoot({
      namespace: 'hyped',
      key: 'pod_1'
    }, openmct.priority.HIGH)

    // openmct.types.
  }
}