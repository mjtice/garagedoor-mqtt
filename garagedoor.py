import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import time

# Setup GPIO
gpio_pin = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(gpio_pin, GPIO.OUT, initial=GPIO.HIGH)

# Setup MQTT
topic = 'home/garage/switch1/set'
status_topic = 'home/garage/switch1/status'

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(topic)
    client.subscribe(status_topic)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    if msg.payload.decode("utf-8") == 'PRESS':
        try:
            # Toggle the gpio pin
            GPIO.output(gpio_pin, 1)
            time.sleep(1)
            GPIO.output(gpio_pin, 0)
            msg = 'OFF'
            client.publish(status_topic, payload=msg, qos=0, retain=False)
        except Exception as e:
            msg = 'OFF'
            client.publish(status_topic, payload=msg, qos=0, retain=False)

try:
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    
    client.connect("192.168.0.2", 1883, 60)
    
    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.
    client.loop_forever()

except KeyboardInterrupt:
    GPIO.cleanup()
