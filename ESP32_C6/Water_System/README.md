# _README_

Project Overview

Set up ESP 32 to connect to wifi. Subscribe to a channel on MQTT. Control two relays that control two valves for in ground irrigation system.

- [X] Connect to wifi
- [X] Subscribe to a MQTT channel
- [] Control two relays

---

Project Details

Using my house wifi, homeassisstant rpi, esp 32, and relay module (+irrigation stuff) to control the relays automatically.
This is to setup an inground irrigation system. 
The idea is to use homeassistant automations to send out MQTT messages. The ESP32 will be listening. When it recieves a message it will output high on a gpio, causing a relay to turn on, powering the irrigation valve.

Subsribe to each topic (relay 1 and relay 2)
topic is for relay to turn on
message is for length of time.

I want the system to fail safe. AKA if the ESP-32 resets to default to relay off. 

If the broker fails while the system is on then it won't turn off. Implement reset.

- [] Add reset to esp. 15 seconds then turn off unless on signal.

- [] Setup homeassisstant automations to send every 10 seconds while system is on.