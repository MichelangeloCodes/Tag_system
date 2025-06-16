import spidev  # Bibliotheek voor SPI-communicatie
import time    # (optioneel, hier niet gebruikt maar handig voor debuggen of vertragingen)

# Maak een SPI object aan en open bus 0, apparaat 0 (meestal verbonden met CS0)
spi = spidev.SpiDev()
spi.open(0, 0)              # open(busnummer, devicenummer)
spi.max_speed_hz = 50000    # Stel SPI-snelheid in (aanpassen indien nodig)

def send_number(seconds):
    """
    Stuurt een geheel getal (1 t/m 60) via SPI naar de STM32.
    Dit getal bepaalt hoelang de LED aan blijft.
    """

    if 0 < seconds <= 60:
        print(f"Stuur: {seconds} seconden")
        spi.xfer([seconds])  # xfer stuurt een lijst van bytes via SPI
    else:
        print("Ongeldig getal (moet tussen 1 en 60 zijn)")

# Vraag de gebruiker om een getal in te voeren       
user_input = input("Hoeveel seconden moet de LED aan blijven? (1-60): ")
# Probeer de input om te zetten naar een geheel getal
seconds = int(user_input)
# Stuur het getal naar de STM32
send_number(seconds)

# Voorbeeld: stuur '5' om LED 5 seconden aan te zetten
send_number(5)

spi.close()
