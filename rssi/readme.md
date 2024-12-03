# Wifi Ranging using RSSI

The intent of this document is to show how range can be determined
between two nodes using COTS wifi hardware. This document uses
receiveded signal strength indicator (RSSI) to estimate distance between
a wifi client and and wifi access point (AP).

## Hardware setup

A regular COTS Wifi AP can be used, but these results were generated
with another laptop acting as an AP. This was done for more control
over the wifi channel being used to capture RSSI readings: the local
wifi environment was 5GHz band, so the laptop was set up to host an AP
on a 2.4GHz channel. The laptop was running linux and the program
`create_ap` with the following configuration:

```
CHANNEL=default
GATEWAY=192.168.12.1
WPA_VERSION=2
ETC_HOSTS=0
DHCP_DNS=gateway
NO_DNS=0
NO_DNSMASQ=0
HIDDEN=0
MAC_FILTER=0
MAC_FILTER_ACCEPT=/etc/hostapd/hostapd.accept
ISOLATE_CLIENTS=0
SHARE_METHOD=nat
IEEE80211N=0
IEEE80211AC=0
IEEE80211AX=0
HT_CAPAB=[HT40+]
VHT_CAPAB=
DRIVER=nl80211
NO_VIRT=0
COUNTRY=
FREQ_BAND=2.4
NEW_MACADDR=
DAEMONIZE=0
NO_HAVEGED=0
WIFI_IFACE=wlp0s20f3
INTERNET_IFACE=eno1
SSID=<ap name>
PASSPHRASE=<pass phase>
USE_PSK=0
```


Another laptop was used as the wifi client, connecting to the above AP
using the built-in wifi connection programs.

The AP laptop was static and the client laptop was moved between 1 and
8 meters away in a straight line.

## Software set up

Measurements were generated on the client laptop using a few python
scripts. `utils.py` contains the function to generate the actual
measurement and uses only built-in python modules.
`plot_rssi_ranging.py` reads the measurments, aggregates, filters and
plots the data and uses a few external dependcies (pandas,
matplotlib).

## Experiment

Three experiments were run: LOS, NLOS and interference. In the LOS
case, there was direct line of sight between the AP and client. In the
NLOS case, a large peice of metal was put roughly 0.5 meters in front
of the AP, blocking line of sight. Finally, in the interfernce case, a
mobile phone streaming vidoe was also connected to the AP and placed
roughly 0.25 meters away from the client.

The AP laptop starts the access point with the command: `create_ap
--config /etc/create_ap.conf`. The client laptop is placed at 1 meter
increments from 1 to 8 meters. `rssi_capture` function is run from
`utils.py` at each measurement point, with a file name indicating its
position. This was repeated for the NLOS and interference cases.

## Plotting data

The data was plotted by running `plot_rssi_ranging.py` in the
directory with the capture `*.csv` files from above.
