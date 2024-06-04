import 'dart:async';
import 'dart:typed_data';

import 'package:controller/entities/quadcopter_data.dart';
import 'package:flutter/material.dart';
import 'package:flutter_libserialport/flutter_libserialport.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';

class MapPage extends StatefulWidget {
  const MapPage({super.key});

  @override
  State<MapPage> createState() => _MapPage();
}

class _MapPage extends State<MapPage> {
  static bool useSatelliteView = true;
  static Duration readDuration = const Duration(milliseconds: 500);

  final MapController mapController = MapController();
  final List<int> serialData = List.empty(growable: true);

  ConnectionStatus connectionStatus = ConnectionStatus.disconnected;
  DateTime lastReceivedData = DateTime.now();
  int diffReceivedData = 0;
  SerialPort? serialPort;
  QuadcopterData? quadcopterData;
  int lastAltitude = 0;
  bool poorSignal = false;

  @override
  void initState() {
    super.initState();
    configureReadInterval();
  }

  void connect() async {
    if (connectionStatus != ConnectionStatus.disconnected) {
      return;
    }

    setState(() => connectionStatus = ConnectionStatus.connecting);
    final availablePorts = SerialPort.availablePorts;

    for (final port in availablePorts) {
      try {
        final isSuccess = await tryConnectSerialPort(port);
        if (isSuccess) {
          setState(() => connectionStatus = ConnectionStatus.connected);
          return;
        }
      } catch (error) {
        print(error);
      }
    }

    setState(() => connectionStatus = ConnectionStatus.disconnected);
  }

  Future<bool> tryConnectSerialPort(String address) {
    setState(() => serialPort = SerialPort(address));

    serialPort!.openReadWrite();

    return Future.delayed(readDuration, () {
      if (serialPort!.bytesAvailable < 5) {
        serialPort!.close();
        serialPort!.dispose();

        return false;
      }

      return true;
    });
  }

  void configureReadInterval() {
    Timer.periodic(readDuration, (_) {
      if (connectionStatus == ConnectionStatus.connected &&
          serialPort != null &&
          serialPort!.isOpen) {
        const List<int> bytes = [
          36,
          77,
          1,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
        ];
        int checkByte = 0;
        for (final byte in bytes) {
          checkByte += checkByte.toUnsigned(8) ^ byte.toUnsigned(8);
        }

        serialPort!.write(Uint8List.fromList([...bytes, checkByte]));
        return;
      }
      if (connectionStatus == ConnectionStatus.connected &&
          serialPort != null &&
          serialPort!.isOpen &&
          serialPort!.bytesAvailable > 0) {
        serialData.addAll(serialPort!.read(QuadcopterData.payloadSize * 3));

        print(serialData);
        int payloadIndex = quadcopterDataFindValidPayload(serialData);
        if (payloadIndex == -1) {
          return;
        }

        QuadcopterData? lastQuadcopterData = quadcopterData;
        List<int> payload = serialData.sublist(
            payloadIndex, payloadIndex + QuadcopterData.payloadSize);

        serialData.removeRange(
            0, payloadIndex + QuadcopterData.payloadSize - 1);

        setState(() {
          quadcopterData = QuadcopterData.fromBytes(payload);
          diffReceivedData =
              DateTime.now().difference(lastReceivedData).inMilliseconds;
          lastReceivedData = DateTime.now();

          if (lastQuadcopterData != null &&
              lastQuadcopterData.start != quadcopterData!.start) {
            lastAltitude = quadcopterData!.altitude;
          } else if (lastQuadcopterData == null) {
            lastAltitude = quadcopterData!.altitude;
          }
        });
      }

      if (DateTime.now().difference(lastReceivedData) > readDuration * 3) {
        if (!poorSignal) {
          setState(() {
            poorSignal = true;
            diffReceivedData =
                DateTime.now().difference(lastReceivedData).inMilliseconds;
          });
        }
      } else if (poorSignal) {
        setState(() {
          poorSignal = false;
        });
      }
    });
  }

  void disconnect() {
    if (connectionStatus != ConnectionStatus.connected) {
      return;
    }

    if (serialPort != null) {
      if (serialPort!.isOpen) {
        serialPort!.close();
      }

      serialPort!.dispose();
      serialPort = null;
    }

    setState(() {
      connectionStatus = ConnectionStatus.disconnected;
      quadcopterData = null;
    });
  }

  String getConnectionStatusText() {
    switch (connectionStatus) {
      case ConnectionStatus.connected:
        return 'Conectado';
      case ConnectionStatus.disconnected:
        return 'Desconectado';
      case ConnectionStatus.connecting:
        return 'Conectando...';
    }
  }

  Color getConnectionStatusColor() {
    switch (connectionStatus) {
      case ConnectionStatus.connected:
        return Colors.green;
      case ConnectionStatus.disconnected:
        return Colors.red;
      case ConnectionStatus.connecting:
        return Colors.yellow;
    }
  }

  String getFlightMode() {
    switch (quadcopterData?.flightMode) {
      case 1:
        return 'Auto nivelamento';
      case 2:
        return 'Altitude';
      case 3:
        return 'GPS';
      case 4:
        return 'RTH ativo';
      case 5:
        return 'RTH Subindo';
      case 6:
        return 'RTH Retornando';
      case 7:
        return 'RTH Pousando';
      case 8:
        return 'RTH finalizado';
      case 9:
        return 'Waypoint';
      default:
        return 'N/A';
    }
  }

  String getMode() {
    switch (quadcopterData?.start) {
      case 0:
        return 'Parado';
      case 1:
        return 'Iniciando';
      case 2:
        return 'Voo';
      default:
        return 'N/A';
    }
  }

  void centerMap() {
    if (quadcopterData == null ||
        connectionStatus != ConnectionStatus.connected) {
      return;
    }

    mapController.move(
      LatLng(
        (quadcopterData?.latitude ?? 1) /
            1000000 *
            (quadcopterData?.latitudeNorth == 0 ? -1 : 1),
        (quadcopterData?.longitude ?? 1) /
            1000000 *
            (quadcopterData?.longiudeEast == 0 ? -1 : 1),
      ),
      18,
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Stack(
        children: [
          FlutterMap(
            options: const MapOptions(
              minZoom: 15,
              maxZoom: 18,
              interactionOptions: InteractionOptions(
                flags: InteractiveFlag.drag |
                    InteractiveFlag.pinchZoom |
                    InteractiveFlag.doubleTapZoom |
                    InteractiveFlag.scrollWheelZoom,
              ),
              initialCenter: LatLng(
                -7.309328,
                -39.3035202,
              ),
              initialZoom: 18,
            ),
            mapController: mapController,
            children: [
              TileLayer(
                urlTemplate: useSatelliteView
                    ? 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'
                    : 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
              ),
              MarkerLayer(
                markers: [
                  Marker(
                    width: 20.0,
                    height: 20.0,
                    point: LatLng(
                      (quadcopterData?.latitude ?? 1) /
                          1000000 *
                          (quadcopterData?.latitudeNorth == 0 ? -1 : 1),
                      (quadcopterData?.longitude ?? 1) /
                          1000000 *
                          (quadcopterData?.longiudeEast == 0 ? -1 : 1),
                    ),
                    rotate: true,
                    child: const FittedBox(
                      child: Icon(
                        Icons.airplanemode_active,
                        color: Colors.red,
                      ),
                    ),
                  ),
                ],
              ),
              const Scalebar(
                textStyle: TextStyle(color: Colors.black, fontSize: 14),
                padding: EdgeInsets.only(right: 10, left: 10, bottom: 10),
                alignment: Alignment.bottomLeft,
              ),
              const Scalebar(
                textStyle: TextStyle(color: Colors.black, fontSize: 14),
                padding: EdgeInsets.only(right: 10, left: 10, bottom: 40),
                length: ScalebarLength.s,
                alignment: Alignment.bottomLeft,
              ),
            ],
          ),
          Container(
            height: 50,
            color:
                Theme.of(context).colorScheme.inverseSurface.withOpacity(0.8),
            child: Row(
              children: [
                IconButton(
                  icon: Icon(
                    connectionStatus != ConnectionStatus.connected
                        ? Icons.refresh
                        : Icons.close,
                    color: Theme.of(context).colorScheme.onInverseSurface,
                  ),
                  onPressed: connectionStatus == ConnectionStatus.disconnected
                      ? connect
                      : disconnect,
                ),
                Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      getConnectionStatusText(),
                      style: TextStyle(
                        color: getConnectionStatusColor(),
                        fontSize: 14,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                    Text(
                      serialPort?.name ?? 'Nenhum dispositivo conectado',
                      style: TextStyle(
                        color: Theme.of(context).colorScheme.outline,
                        fontSize: 12,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                    Text(
                      serialPort?.address.toString() ?? '',
                      style: TextStyle(
                        color: Theme.of(context).colorScheme.outline,
                        fontSize: 8,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                  ],
                ),
                const SizedBox(width: 15),
                Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  crossAxisAlignment: CrossAxisAlignment.center,
                  children: [
                    Text(
                      'erro',
                      style: TextStyle(
                        color: Theme.of(context).colorScheme.onInverseSurface,
                        fontSize: 12,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                    Text(
                      quadcopterData?.error.toString() ?? '-1',
                      style: TextStyle(
                        color: Theme.of(context).colorScheme.onInverseSurface,
                        fontSize: 16,
                        fontWeight: FontWeight.w400,
                      ),
                    ),
                  ],
                ),
                const SizedBox(width: 15),
                Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  crossAxisAlignment: CrossAxisAlignment.center,
                  children: [
                    Text(
                      'Modo',
                      style: TextStyle(
                        color: Theme.of(context).colorScheme.onInverseSurface,
                        fontSize: 12,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                    Text(
                      getMode(),
                      style: TextStyle(
                        color: Theme.of(context).colorScheme.onInverseSurface,
                        fontSize: 16,
                        fontWeight: FontWeight.w400,
                      ),
                    ),
                  ],
                ),
                const SizedBox(width: 15),
                Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  crossAxisAlignment: CrossAxisAlignment.center,
                  children: [
                    Text(
                      'modo de voo',
                      style: TextStyle(
                        color: Theme.of(context).colorScheme.onInverseSurface,
                        fontSize: 12,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                    Text(
                      getFlightMode(),
                      style: TextStyle(
                        color: Theme.of(context).colorScheme.onInverseSurface,
                        fontSize: 16,
                        fontWeight: FontWeight.w400,
                      ),
                    ),
                  ],
                ),
                const Spacer(),
                Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    Icon(
                      Icons.wifi,
                      size: 18,
                      color: poorSignal
                          ? Theme.of(context).colorScheme.error
                          : Theme.of(context).colorScheme.onInverseSurface,
                    ),
                    Text(
                      diffReceivedData >= 10000
                          ? '---'
                          : '${diffReceivedData}ms',
                      style: TextStyle(
                        color: poorSignal
                            ? Theme.of(context).colorScheme.error
                            : Theme.of(context).colorScheme.onInverseSurface,
                        fontSize: 12,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                  ],
                ),
                const SizedBox(width: 15),
                Row(
                  children: [
                    Icon(
                      Icons.satellite_alt,
                      size: 16,
                      color: (quadcopterData?.numberUsedSats ?? -1) < 6
                          ? Theme.of(context).colorScheme.error
                          : Theme.of(context).colorScheme.onInverseSurface,
                    ),
                    const SizedBox(width: 5),
                    Text(
                      quadcopterData?.numberUsedSats.toString() ?? '-1',
                      style: TextStyle(
                        color: Theme.of(context).colorScheme.onInverseSurface,
                        fontSize: 16,
                        fontWeight: FontWeight.w400,
                      ),
                    ),
                  ],
                ),
                const SizedBox(width: 15),
                Row(
                  children: [
                    Icon(
                      Icons.battery_full,
                      size: 16,
                      color: Theme.of(context).colorScheme.onInverseSurface,
                    ),
                    Text(
                      '${quadcopterData?.batteryVoltage.toString() ?? '-1'}V',
                      style: TextStyle(
                        color: Theme.of(context).colorScheme.onInverseSurface,
                        fontSize: 16,
                        fontWeight: FontWeight.w400,
                      ),
                    ),
                  ],
                ),
                const SizedBox(width: 15),
                Row(
                  children: [
                    Icon(
                      Icons.thermostat,
                      size: 16,
                      color: Theme.of(context).colorScheme.onInverseSurface,
                    ),
                    Text(
                      '${(((quadcopterData?.temperature.toSigned(16) ?? 0) + 12421) / 340).toStringAsFixed(1)}C',
                      style: TextStyle(
                        color: Theme.of(context).colorScheme.onInverseSurface,
                        fontSize: 16,
                        fontWeight: FontWeight.w400,
                      ),
                    ),
                  ],
                ),
                const SizedBox(width: 15),
                Row(
                  children: [
                    Icon(
                      Icons.height,
                      size: 16,
                      color: Theme.of(context).colorScheme.onInverseSurface,
                    ),
                    Text(
                      '${(quadcopterData?.altitude ?? 1)}M',
                      style: TextStyle(
                        color: Theme.of(context).colorScheme.onInverseSurface,
                        fontSize: 16,
                        fontWeight: FontWeight.w400,
                      ),
                    ),
                    Text(
                      ' (${(quadcopterData?.altitude ?? 0) - lastAltitude})',
                      style: TextStyle(
                        color: Theme.of(context).colorScheme.onInverseSurface,
                        fontSize: 10,
                        fontWeight: FontWeight.w200,
                      ),
                    ),
                  ],
                ),
                const Spacer(
                  flex: 10,
                ),
                IconButton(
                  icon: Icon(
                    useSatelliteView ? Icons.map : Icons.satellite,
                    color: Theme.of(context).colorScheme.onInverseSurface,
                  ),
                  onPressed: () {
                    setState(() => useSatelliteView = !useSatelliteView);
                  },
                ),
              ],
            ),
          ),
        ],
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: centerMap,
        mini: true,
        child: const Icon(Icons.gps_fixed),
      ),
    );
  }

  @override
  void dispose() {
    disconnect();
    super.dispose();
  }
}

enum ConnectionStatus {
  connected,
  disconnected,
  connecting,
}
