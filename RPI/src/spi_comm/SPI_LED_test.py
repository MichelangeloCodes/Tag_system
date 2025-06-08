import spidev
import time

# SPI bus en device (meestal bus=0, device=0 voor CS0)
spi = spidev.SpiDev()
spi.open(0, 0)       # open(bus, device)
spi.max_speed_hz = 50000  # pas aan als nodig

def send_number(seconds):
    if 0 < seconds <= 60:
        print(f"Stuur: {seconds} seconden")
        spi.xfer([seconds])
    else:
        print("Ongeldig getal (moet tussen 1-60 zijn)")

# Voorbeeld: stuur '5' om LED 5 seconden aan te zetten
send_number(5)

spi.close()
