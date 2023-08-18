set(DRV_SRC  
adc.c
bpwm.c
clk.c
crc.c
fmc.c
gpio.c
i2c.c
llsi.c
pdma.c
retarget.c
spi.c
sys.c
timer.c
uart.c
usbd.c
wdt.c
wwdt.c
dac.c
acmp.c
)


add_library(standard_driver ${DRV_SRC})
