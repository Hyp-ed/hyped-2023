import { CONFIG } from "./core/config";
import { dicts } from "./core/dictionary";
import { logger } from "./core/logger";
import Server from "./server/server";

// Start the server
const server = new Server({
  host: CONFIG.host,
  port: CONFIG.port,
  wss_port: CONFIG.wssPort,
  broker: CONFIG.mqttBroker,
  dictionaries: dicts,
  history: {
    host: CONFIG.history.host,
    db: CONFIG.history.db,
  },
  persistence: "openmct.plugins.LocalStorage()",
});

server.start((err) => {
  if (err) {
    logger.error("Server failed to start", err);
    process.exit(1);
  }
  logger.info(`Server listening on ${server.config.port}`);
});
