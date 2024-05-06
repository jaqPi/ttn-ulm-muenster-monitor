function decodeUplink(input) {
  return {
    data: v2Decoder(input.bytes),
    warnings: [],
    errors: []
  };
}

function v2Decoder(bytes) {
  var decoded = {};

  // Messwerte:
  decoded.distance = Math.round(((bytes[6] << 8) + bytes[7]) / 10)/10;
  decoded.temp = ((bytes[0] << 8) + bytes[1]) / 100;
  decoded.humidity = ((bytes[4] << 8) + bytes[5]) / 100;
  decoded.pressure = ((bytes[2] << 8) + bytes[3]);
  decoded.ambientLight = ((bytes[13] << 8) + bytes[14]) / 100;
  decoded.batteryVoltage = ((bytes[20] << 8) + bytes[21]) / 100;

  // Zusätzliche Werte, etwas Statistik für die Entwicklung:
  decoded.meanDistance = ((bytes[6] << 8) + bytes[7]) / 100;
  decoded.standardDeviationDistance = ((bytes[8] << 8) + bytes[9]) / 100;
  decoded.medianDistance = ((bytes[10] << 8) + bytes[11]) / 100;
  decoded.successfulMeasurementsDistance = parseInt(bytes[12],10);
  decoded.meanAmbientLight = decoded.ambientLight;
  decoded.standardDeviationAmbientLight = ((bytes[15] << 8) + bytes[16]) / 100;
  decoded.medianLight = ((bytes[17] << 8) + bytes[18]) / 100;
  decoded.successfulMeasurementsAmbientLight = parseInt(bytes[19],10);

  return decoded;
}
