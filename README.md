# tiny display
tiny (start kernel 5.4-y)
linux drivers gpu drm tiny

## Out-of-tree tiny modules

Not everyone is able or want to spend time on getting a driver included in Linux, but many can write drivers that works and wants to share it.

This repo is dedicated to the maker community and people that make things work.

### First
~~~
sudo apt update
sudo apt upgrade
sudo reboot
~~~

### Install Raspberry Pi Kernel Headers
~~~~
sudo apt install git bc bison flex libssl-dev libncurses5-dev
sudo apt-get install raspberrypi-kernel-headers
~~~~

### tiny
~~~~
git clone https://github.com/birdtechstep/tiny.git
cd tiny
make

sudo cp *.ko /lib/modules/`uname -r`/kernel/drivers/gpu/drm/tiny/
sudo depmod
~~~~

### Dual Display Set
~~~~
sudo nano /usr/share/X11/xorg.conf.d/99-fbturbo.conf
~~~~

~~~~
# This is a minimal sample config file, which can be copied to
# /etc/X11/xorg.conf in order to make the Xorg server pick up
# and load xf86-video-fbturbo driver installed in the system.
#
# When troubleshooting, check /var/log/Xorg.0.log for the debugging
# output and error messages.
#
# Run "man fbturbo" to get additional information about the extra
# configuration options for tuning the driver.

Section "Device"
        Identifier      "Allwinner A10/A13 FBDEV"
        Driver          "fbturbo"
        Option          "fbdev" "/dev/fb0"

        Option          "SwapbuffersWait" "true"
EndSection

Section "Device"
        Identifier      "FBDEV 1"
        Driver          "fbturbo"
        Option          "fbdev" "/dev/fb1"
EndSection

Section "Screen"
        Identifier      "HDMI"
        Device          "Allwinner A10/A13 FBDEV"
        Monitor         "Monitor name 0"
EndSection

Section "Screen"
        Identifier      "TFT LCD"
        Device          "FBDEV 1"
        Monitor         "Monitor name 1"
EndSection

Section "ServerLayout"
        Identifier      "Default Layout"
        Screen          0 "TFT LCD"
        Screen          1 "HDMI" RightOf "TFT LCD"
EndSection
~~~~

### Set Input cell
~~~~
[ 1 0 0 ]
[ 0 1 0 ]
[ 0 0 1 ]
~~~~
Option "TransformationMatrix" "c0 0 c1 0 c2 c3 0 0 1"

•	c0 = touch_area_width / total_width

•	c2 = touch_area_height / total_height

•	c1 = touch_area_x_offset / total_width

•	c3 = touch_area_y_offset / total_height

#### Example

X 480/1366+480 = 480/1846 = 0.2600216684723727

Y 320/768  = 320/768 = 0.4166666666666667

X Offset 1366/1846 = 0.7399783315276273


Option "TransformationMatrix" "0.2600216684723727 0 0.7399783315276273 0 0.4166666666666667 0 0 0 1"

~~~~
sudo nano /usr/share/X11/xorg.conf.d/40-libinput.conf
~~~~
add Option

~~~~
Section "InputClass"
        Identifier "libinput touchscreen catchall"
        MatchIsTouchscreen "on"
        MatchDevicePath "/dev/input/event*"
        Driver "libinput"
        Option "TransformationMatrix" "0.2600216684723727 0 0.7399783315276273 0 0.4166666666666667 0 0 0 1"
EndSection
~~~~

### test video on fb1
~~~~
sudo apt install -y mplayer
wget http://fredrik.hubbe.net/plugger/test.mpg
mplayer -nolirc -vo fbdev2:/dev/fb1 -zoom -x 480 -y 320 test.mpg
~~~~

#### BIRD TECHSTEP

##### Donation
If this project help you reduce time to develop, you can give me a cup of coffee :) 

[![paypal](https://www.paypalobjects.com/en_GB/TH/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=CYA3UGY8TNY82)