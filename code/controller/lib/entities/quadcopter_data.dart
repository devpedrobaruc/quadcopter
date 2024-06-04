class QuadcopterData {
  static int payloadSize = 36;
  static List<int> header = [74, 66];

  int error;
  int flightMode;
  int batteryVoltage;
  int temperature;
  int angleRoll;
  int anglePitch;
  int start;
  int altitude;
  int takeoffThrottle;
  int angleYaw;
  int headingLock;
  int numberUsedSats;
  int fixType;
  int latitude;
  int longitude;
  int adjustableSetting1;
  int adjustableSetting2;
  int adjustableSetting3;
  int latitudeNorth;
  int longiudeEast;

  QuadcopterData({
    required this.error,
    required this.flightMode,
    required this.batteryVoltage,
    required this.temperature,
    required this.angleRoll,
    required this.anglePitch,
    required this.start,
    required this.altitude,
    required this.takeoffThrottle,
    required this.angleYaw,
    required this.headingLock,
    required this.numberUsedSats,
    required this.fixType,
    required this.latitude,
    required this.longitude,
    required this.adjustableSetting1,
    required this.adjustableSetting2,
    required this.adjustableSetting3,
    required this.latitudeNorth,
    required this.longiudeEast,
  });

  factory QuadcopterData.fromBytes(List<int> bytes) {
    return QuadcopterData(
      error: bytes[2],
      flightMode: bytes[3],
      batteryVoltage: bytes[4],
      temperature: bytes[5].toUnsigned(8) | (bytes[6].toUnsigned(8) << 8),
      angleRoll: bytes[7],
      anglePitch: bytes[8],
      start: bytes[9],
      altitude: bytes[10] | (bytes[11] << 8),
      takeoffThrottle: bytes[12] | (bytes[13] << 8),
      angleYaw: bytes[14] | (bytes[15] << 8),
      headingLock: bytes[16],
      numberUsedSats: bytes[17],
      fixType: bytes[18],
      latitude:
          bytes[19] | (bytes[20] << 8) | (bytes[21] << 16) | (bytes[22] << 24),
      longitude:
          bytes[23] | (bytes[24] << 8) | (bytes[25] << 16) | (bytes[26] << 24),
      adjustableSetting1: bytes[27] | (bytes[28] << 8),
      adjustableSetting2: bytes[29] | (bytes[30] << 8),
      adjustableSetting3: bytes[31] | (bytes[32] << 8),
      latitudeNorth: bytes[33],
      longiudeEast: bytes[34],
    );
  }
}

int quadcopterDataFindValidPayload(List<int> bytes) {
  int payloadIndex = -1;

  for (int i = 0; i < bytes.length - 1; i++) {
    if (bytes[i] != QuadcopterData.header[0] ||
        bytes[i + 1] != QuadcopterData.header[1]) {
      continue;
    }

    if (QuadcopterData.payloadSize > bytes.length - i) {
      continue;
    }

    int checkSum = 0;
    for (int j = i + QuadcopterData.header.length;
        j < i + (QuadcopterData.payloadSize - 1);
        j++) {
      checkSum = checkSum.toUnsigned(8) ^ bytes[j].toUnsigned(8);
    }

    if (checkSum != bytes[i + (QuadcopterData.payloadSize - 1)]) {
      continue;
    }

    payloadIndex = i;
  }

  return payloadIndex;
}
