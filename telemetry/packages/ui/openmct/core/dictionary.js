import dictionaries from "../plugins/dictionaries.json";
import Dictionary from "../server/dictionary";
import { logger } from "./logger";

export const dicts = [];

// Read dictionaries.json into Dictionary objects
dictionaries.forEach((dictionary) => {
  const dict = new Dictionary(dictionary.name, dictionary.objpath);
  dictionary.measurements.forEach((measurement) => {
    try {
      dict.addMeasurement(measurement.name, measurement.objpath, [measurement.options], {
        topic: measurement.topic,
      });
    } catch (e) {
      logger.error(`Failed to add measurement ${measurement.name} to dictionary ${dictionary.name}`, e);
    }
  });
  dicts.push(dict);
});
