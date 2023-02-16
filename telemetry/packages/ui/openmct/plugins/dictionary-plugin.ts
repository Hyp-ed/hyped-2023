import { OpenMCT } from "openmct/dist/openmct"
import { http } from "../core/http"
import { ObjectIdentitifer } from "../types/ObjectIdentifier"
import { ObjectType } from "../types/ObjectType"

function getDictionary() {
  return http.get('openmct/dictionary').json().then((data) => {
    return data
  })
}

function getObjectTypes() {
  return http.get('openmct/object-types').json<ObjectType[]>().then((data) => {
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
      } else {
        // TODO: Type :)
        const measurement = dictionary.measurements.find((measurement: any) => measurement.key === identifier.key)

        return {
          identifier,
          name: measurement.name,
          // TODO: Maybe should check this exists
          type: `hyped.${measurement.type}`,
          telemetry: {
            values: measurement.values
          },
          location: "hyped:pod_1"
        }
      }
    })
  }
}

const compositionProvider = {
  appliesTo: (domainObject: any) => {
    return domainObject.identifier.namespace === 'hyped' && domainObject.type === 'folder'
  },
  load: (domainObject: any) => {
    return getDictionary().then((dictionary: any) => {
      return dictionary.measurements.map((measurement: any) => {
        return {
          namespace: 'hyped',
          key: measurement.key
        }
      })
    })
  }
}

export function DictionaryPlugin() {
  return function install(openmct: OpenMCT) {
    openmct.objects.addRoot({
      namespace: 'hyped',
      key: 'pod_1'
    }, openmct.priority.HIGH)

    openmct.objects.addProvider('hyped', objectProvider)

    getObjectTypes().then((objectTypes) => {
      objectTypes.map((objectType) => {
        openmct.types.addType(`hyped.${objectType.id}`, {
          name: objectType.name,
          // description: objectType.name,
          cssClass: objectType.icon,
        })
      })
    })

    openmct.composition.addProvider(compositionProvider as any)
  }
}

