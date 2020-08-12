# rasbian-dtoverlay


## Compile dtb and put it directly into it's destination:
~~~~
chmod +x dtc.sh
./dtc.sh
~~~~

### makerlabtft154
~~~~
sudo dtc -@ -I dts -O dtb -o /boot/overlays/makerlabtft154.dtbo makerlabtft154-overlay.dts
~~~~
### makerlabtft40-resistive
~~~~
sudo dtc -@ -I dts -O dtb -o /boot/overlays/makerlabtft40-resistive.dtbo makerlabtft40-resistive-overlay.dts
~~~~

## cmdline.txt
~~~~
fbcon=map:10 fbcon=font:VGA8x16
~~~~
## config.txt
### makerlabtft154
~~~~
dtoverlay=makerlabtft154
dtparam=speed=32000000
dtparam=rotation=0
~~~~

### makerlabtft40-resistive
~~~~
dtoverlay=makerlabtft40-resistive
dtparam=speed=32000000
dtparam=rotation=270
dtparam=swapxy=1
~~~~

## PIN CONNECT
~~~~
DISPLAY SPI
cs-gpios    = CE0
sclk-gpios  = SCLK
mosi-gpios  = MOSI
miso-gpios  = MISO
dc-gpios    = BCM27 [pin13]
reset-gpios = BCM26 [pin37]
led-gpios   = BCM13 [pin33]

~~~~
### BIRD TECHSTEP
