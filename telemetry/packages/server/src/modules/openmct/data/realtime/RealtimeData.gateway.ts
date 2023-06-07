import { Logger } from '@/modules/logger/Logger.decorator';
import { MeasurementReading } from '@/modules/measurement/MeasurementReading.types';
import { EVENTS } from '@hyped/telemetry-constants';
import { LoggerService } from '@nestjs/common';
import {
  ConnectedSocket,
  MessageBody,
  SubscribeMessage,
  WebSocketGateway,
  WebSocketServer,
} from '@nestjs/websockets';
import { Server, Socket } from 'socket.io';

@WebSocketGateway({
  path: '/openmct/data/realtime',
  cors: {
    origin: '*',
  },
})
export class RealtimeDataGateway {
  @WebSocketServer()
  socket: Server;

  constructor(
    @Logger()
    private readonly logger: LoggerService,
  ) {}

  @SubscribeMessage(EVENTS.SUBSCRIBE_TO_MEASUREMENT)
  subscribeToMeasurement(
    @MessageBody() measurementRoom: string,
    @ConnectedSocket() client: Socket,
  ) {
    client.join(measurementRoom);
  }

  @SubscribeMessage(EVENTS.UNSUBSCRIBE_FROM_MEASUREMENT)
  unsubscribeFromMeasurement(
    @MessageBody() measurementRoom: string,
    @ConnectedSocket() client: Socket,
  ) {
    client.leave(measurementRoom);
  }

  sendMeasurementReading(props: MeasurementReading) {
    const { podId, measurementKey, value } = props;

    const measurementRoom = `${podId}/measurement/${measurementKey}`;
    this.socket.to(measurementRoom).emit('measurement', {
      podId,
      measurementKey,
      value,
      timestamp: Date.now(),
    });

    this.logger.debug(
      `Sending realtime ${value} to ${measurementRoom}`,
      RealtimeDataGateway.name,
    );
  }
}
