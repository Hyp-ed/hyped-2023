import { InfluxDB, QueryApi, WriteApi } from '@influxdata/influxdb-client';
import { Injectable, OnModuleInit } from '@nestjs/common';
import {
  INFLUX_BUCKET,
  INFLUX_ORG,
  INFLUX_TOKEN,
  INFLUX_URL,
} from '../core/config';

@Injectable()
export class InfluxService implements OnModuleInit {
  private connection: InfluxDB;
  public write: WriteApi;
  public query: QueryApi;

  async $connect() {
    this.connection = new InfluxDB({ url: INFLUX_URL, token: INFLUX_TOKEN });
    this.write = this.connection.getWriteApi(INFLUX_ORG, INFLUX_BUCKET);
    this.query = this.connection.getQueryApi(INFLUX_ORG);
  }

  async onModuleInit() {
    await this.$connect();
    if (!this.write) {
      throw new Error('InfluxDB write API not initialized');
    }

    if (!this.query) {
      throw new Error('InfluxDB query API not initialized');
    }
  }

  async onModuleDestroy() {
    try {
      await this.write.close();
    } catch (e) {
      console.error(e);
    }
  }
}
