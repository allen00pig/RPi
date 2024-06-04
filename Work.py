import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import board
import adafruit_dht

mqttHost = "mqtt3.thingspeak.com"
channelID = "2565040"
MQTT_SUB_TOPIC = "channels/" + channelID + "/subscribe"
MQTT_PUB_TOPIC = "channels/" + channelID + "/publish"

def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")
    client.subscribe("channels/" + channelID + "/subscribe")

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="ACoVHwkLBgY4BSI2EToNBxA")
mqttc.on_connect = on_connect #綁定連接時的處理函數
mqttc.on_message = on_message #綁定接收到訊息時的處理函數​

mqttc.username_pw_set("ACoVHwkLBgY4BSI2EToNBxA", "Lc4+OXKS4YehDmesO56QAChu")
mqttc.connect(mqttHost, 1883, 60)
mqttc.subscribe(MQTT_SUB_TOPIC) 

RELAY_PIN = 18
GPIO.setup(RELAY_PIN, GPIO.OUT, initial=GPIO.HIGH)

dht22 = adafruit_dht.DHT22(board.D13, use_pulseio=False)

while True:
    try:
        temperature = dht22.temperature
        humidity = dht22.humidity
        tPayload = "field1=" + str(temperature) + "&field2=" + str(humidity)
        print(tPayload)
        mqttc.publish(MQTT_PUB_TOPIC, payload=tPayload)
        
        if(int(temperature) >= 27 or int(humidity) >= 60):
            GPIO.output(RELAY_PIN, GPIO.HIGH)
        else:
            GPIO.output(RELAY_PIN, GPIO.LOW)
    except RuntimeError:
        time.sleep(1)
        continue
    time.sleep(5)

mqttc.disconnect()
GPIO.cleanup()