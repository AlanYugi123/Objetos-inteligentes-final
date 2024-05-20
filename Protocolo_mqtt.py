import serial
import time
import paho.mqtt.client as mqtt
from datetime import datetime

# Configurações do broker MQTT
broker = "localhost"  # Substitua pelo endereço IP do seu broker MQTT
port = 1883
topic_pub = "led/status"
topic_sub = "led/control"

# Configurações da porta serial
serial_port = "COM5"  # Substitua pelo nome da porta serial do seu Arduino
baud_rate = 9600

try:
    ser = serial.Serial(serial_port, baud_rate)
except serial.SerialException as e:
    print(f"Erro ao abrir a porta serial {serial_port}: {e}")
    exit()

# Callback para conexão com o broker MQTT
def on_connect(client, userdata, flags, rc):
    print("Conectado ao broker MQTT")
    client.subscribe(topic_sub)

# Callback para recebimento de mensagens do MQTT
def on_message(client, userdata, msg):
    print(f"Recebido: {msg.topic} {msg.payload.decode()}")
    if msg.topic == topic_sub:
        ser.write((msg.payload.decode() + '\n').encode())

# Configura o cliente MQTT
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

try:
    client.connect(broker, port, 60)
except Exception as e:
    print(f"Erro ao conectar ao broker MQTT: {e}")
    exit()

# Loop principal
try:
    client.loop_start()
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if "LED ON" in line or "LED OFF" in line:
                now = datetime.now()
                timestamp = now.strftime("%Y-%m-%d %H:%M:%S")
                message = f"{timestamp} - {line}"
                print(f"Publicando: {message}")
                client.publish(topic_pub, message)
        time.sleep(1)
except KeyboardInterrupt:
    ser.close()
    client.loop_stop()
    client.disconnect()