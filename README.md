# Sensor/Node für's Münster-Monitoring

Aktuell im Test, Messaufbau und ein paar Infos: https://wiki.verschwoerhaus.de/risssimulator16153


## Pin-Belegung

```
Münster-Platine              VL6180X      BME280       Pull-Up-Widerstände
---------------              -------      ------       -------------------

A4    SDA / Daten            4 / SDA      4 / SDA      *
A5    SCL / Takt             5 / SCL      3 / SCL      |   *
VCC   Versorgungsspannung    2 / VIN      1 / VIN      *   *
GND                          3 / GND      2 / GND
```
