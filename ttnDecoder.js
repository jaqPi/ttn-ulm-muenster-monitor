function Decoder(bytes, port) {
  var decoded = {};

  // Messwerte:
  decoded.distance = Math.round(((bytes[6] << 8) + bytes[7]) / 10)/10;
  decoded.temp = ((bytes[0] << 8) + bytes[1]) / 100;
  decoded.humidity = ((bytes[4] << 8) + bytes[5]) / 100;
  decoded.pressure = ((bytes[2] << 8) + bytes[3]);
  decoded.ambientLight = ((bytes[11] << 8) + bytes[12]) / 100;
  decoded.batteryVoltage = ((bytes[16] << 8) + bytes[17]) / 100;

  // Zusätzliche Werte, etwas Statistik für die Entwicklung:
  decoded.meanDistance = ((bytes[6] << 8) + bytes[7]) / 100;
  decoded.standardDeviation = ((bytes[8] << 8) + bytes[9]) / 100;
  decoded.successfulMeasurements = parseInt(bytes[10],10);
  decoded.meanAmbientLight = decoded.ambientLight;
  decoded.standardDeviationAmbientLight = ((bytes[13] << 8) + bytes[14]) / 100;
  decoded.successfulMeasurementsAmbientLight = parseInt(bytes[15],10);

  return decoded;
}
