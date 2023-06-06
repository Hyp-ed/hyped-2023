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
  server: Server;

  @SubscribeMessage('subscribe')
  subscribeToMeasurement(
    @MessageBody() measurementRoom: string,
    @ConnectedSocket() client: Socket,
  ) {
    client.join(measurementRoom);
  }

  @SubscribeMessage('unsubscribe')
  unsubscribeFromMeasurement(
    @MessageBody() measurementRoom: string,
    @ConnectedSocket() client: Socket,
  ) {
    client.leave(measurementRoom);
  }

  sendMeasurement(podId: string, measurementKey: string, value: any) {
    const measurementRoom = `pod_${podId}/${measurementKey}`;
    this.server
      .to(`pod_${podId}/${measurementKey}`)
      .emit('measurement', { id: measurementRoom, value });
    console.log(`Sending realtime ${value} to ${podId}/${measurementKey}`);
  }
}
