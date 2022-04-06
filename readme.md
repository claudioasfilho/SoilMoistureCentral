# Soil Moisture Sensor central


This is the Central device that searches for BLE devices named "MoistSensor" and connects to it.

It has a state machine that fist searches for that ble device, connects and then reads a characteristic within a certain period.
