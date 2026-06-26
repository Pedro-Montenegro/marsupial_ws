import Jetson.GPIO as GPIO
import time

# Definir o modo de numeração (BOARD usa o número físico do pino)
GPIO.setmode(GPIO.BOARD)

# Pino que você deseja usar
input_pin = 9 

# Configura o pino como entrada com PULL_UP interno
GPIO.setup(input_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("status: Aguardando sinal no pino {input_pin}...")

try:
    while True:
        value = GPIO.input(input_pin)
        if value == GPIO.LOW:
            print("Botão pressionado! (Sinal LOW detectado)")
        else:
            print("Aguardando... (Sinal HIGH via Pull-up)")
        time.sleep(0.5)
finally:
    GPIO.cleanup() # Limpa as configurações ao sair