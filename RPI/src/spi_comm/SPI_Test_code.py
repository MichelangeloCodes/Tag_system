
import spidev;
import time;

spi = spidev.SpiDev()
spi.opem(0,0) #Bus 0, device 0 (cs0)
spi.max_speed_hz = 500000

try:
    while True:
        resp = spi.xfer2([0x42]) #stuur 0x42
        print(f"response: {resp}")
        time.sleep(1)
except KeyboardInterrupt:
    spi.close

print(spidev);
