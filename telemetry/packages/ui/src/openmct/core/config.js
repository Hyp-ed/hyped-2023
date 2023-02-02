import * as env from "env-var";

export const CONFIG = {
  env: env.get("NODE_ENV").default("development").asString(),
  host: env.get("HOST").default("localhost").asString(),
  port: env.get("PORT").default(8080).asPortNumber(),
  wssPort: env.get("WSS_PORT").default(8082).asPortNumber(),
  mqttBroker: env.get("MQTT_BROKER").required().asString(),
  history: {
    host: env.get("INFLUX_HOST").default("localhost").asString(),
    db: env.get("INFLUX_DB").default("cbeam").asString(),
  },
};
