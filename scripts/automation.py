import argparse
import paho.mqtt.client as mqtt
import requests
import serial
import struct

DEFAULT_TEMP = 22.0
DEFAULT_HUM = 50.0

parser = argparse.ArgumentParser()
parser.add_argument("-d", "--dev", help="serial device", required=True)
parser.add_argument("-b", "--baud", help="serial baud rate", type=int, default=115200)
parser.add_argument("-m", "--mqtt-server", help="MQTT server address", required=True)
parser.add_argument("-c", "--mqtt-credentials", help="MQTT server credentials [user]:[password]", required=True)
parser.add_argument("-p", "--topic-prefix", help="topic prefix for MQTT", required=True)
parser.add_argument("-t", "--temp-url", help="air temperature url")
parser.add_argument("-u", "--hum-url", help="air humidity url")

args = parser.parse_args()

mqttSplitUrl = args.mqtt_server.split(":")

mqttAddress = mqttSplitUrl[0]
mqttPort = 1883

if len(mqttSplitUrl) == 2:
    mqttPort = int(mqttSplitUrl[1])

client = mqtt.Client()

if args.mqtt_credentials:
    splitCredentials = args.mqtt_credentials.split(":")
    username = splitCredentials[0]
    password = None
    if len(splitCredentials) == 2:
        password = splitCredentials[1]
    client.username_pw_set(username, password)

client.connect(mqttAddress, mqttPort, 60)
client.loop_start()

eco2Topic = args.topic_prefix + "/eco2"
etvocTopic = args.topic_prefix + "/etvoc"
baselineTopic = args.topic_prefix + "/baseline"

ser = serial.Serial(args.dev, args.baud)

while True:
    try:
        line = str(ser.readline(), "utf-8").strip()
    except UnicodeDecodeError:
        print("Could not decode line!")
        continue

    if line.startswith("baseline:"):
        # baseline: [100,114]
        baseline = line.split("[")[1][:-1]

        client.publish(baselineTopic, payload=baseline, qos=1, retain=True)
    elif line.startswith("eCO2:"):
        # eCO2: 421, eTVOC: 3, raw_current: 38, raw_voltage: 281
        splitLine = line.split(",")
        eco2 = int(splitLine[0].split(":")[1].strip())
        etvoc = int(splitLine[1].split(":")[1].strip())

        if eco2 != 9999 and etvoc != 999:
            print(f"Got new sample: eCO2={eco2}ppm eTVOC={etvoc}ppb")
            client.publish(eco2Topic, payload=eco2, qos=1)
            client.publish(etvocTopic, payload=etvoc, qos=1)
        else:
            print("Got default sensor values")

        if args.temp_url or args.hum_url:
            temp = DEFAULT_TEMP
            hum = DEFAULT_HUM

            try:
                if args.temp_url:
                    r = requests.get(args.temp_url, timeout=2)
                    r.raise_for_status()
                    temp = float(r.text)
                if args.hum_url:
                    r = requests.get(args.hum_url, timeout=2)
                    r.raise_for_status()
                    hum = float(r.text)

                ser.write(b"e" + struct.pack('>f', hum) + struct.pack('>f', temp) + b'\r')
                resLine = str(ser.readline(), "utf-8").strip()
                if resLine == "OK":
                    print(f"Successfully wrote environment: temp={temp} hum={hum}")
                else:
                    print(f"Could not write environment: temp={temp} hum={hum}")
            except Exception as e:
                print(e)
    else:
        print("Line format unknown: " + line)
