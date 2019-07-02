import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create the ADC object using the I2C bus
ads = ADS.ADS1115(i2c)

# Create single-ended input on channel 0
chan1 = AnalogIn(ads, ADS.P0)
chan2 = AnalogIn(ads, ADS.P1)

# Create differential input between channel 0 and 1
#chan = AnalogIn(ads, ADS.P0, ADS.P1)

print("{:>5}\t{:>10}".format('Silnik 1', 'Silnik 2'))

while True:
	print("{:>5.3f}\t{:>5.3f}".format(chan1.voltage-0.68, chan2.voltage))
	print("{:>5.3f}\t{:>5.3f}".format (round((chan1.voltage-0.391)*360/(1.906 - 0.391), 1), round((chan2.voltage-0.391)*360/(1.906 - 0.391), 1)))
	time.sleep(0.5)
# print("{:>5}\t{:>5.3f}".format((chan.value-6217)/(31422 - 6217)*360, (chan.voltage-0.78)*113.1363))

