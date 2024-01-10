# Nordic sidewalk template_subghz with added LR1110 Wifi scan

## hardware
[NRF52840-DK](https://www.nordicsemi.com/Products/Development-hardware/nrf52840-dk) with [LR1110 development kit for 915MHz north america](https://www.semtech.com/products/wireless-rf/lora-edge/lr1110dvk1tcks)
## prerequisites
1) Set up your [Nordic development environment](https://nrfconnect.github.io/sdk-sidewalk/setting_up_sidewalk_environment/setting_up_sdk.html) if you haven't already done so.  And you have [provisioned the device](https://nrfconnect.github.io/sdk-sidewalk/setting_up_sidewalk_environment/setting_up_sidewalk_product.html) and loaded the ``Nordic_MFG.hex`` into the board.
   * If you have never used sidewalk before, it is recommended to first run template_subghz (as provided by nordic) with the sx1262 shield
2) later, after building & flashing, the LR11xx firmware version will print in the logs upon startup.  Check that  for LR1110 its at least version 0x0308, or for LR112x its at least version 0x0102.   If its earlier / older, then use the [firmware updater tool](https://github.com/Lora-net/SWTL001/wiki).  Or ``../SWLT001`` on this platform.

## LR1110 build:
``west build -b nrf52840dk_nrf52840 -- -DRADIO=LR1110  -DOVERLAY_CONFIG="cli.conf"``

``west flash``

## running
The LR1110 shield has 3 LEDs.  Green indicating sidewalk RX, red indicating sidewalk TX, and yellow indicating WiFi scan.  Sidewalk CSS mode (LDR mode) limits 19 bytes per packet, so a Wifi scan will be sent up in two to three fragments.

The usb-uart is available just as with the original template_subghz from Nordic (i.e. 115200bps), showing logs and status.  When built with ``cli.conf``You can interact on the command line in addition to viewing the execution log.   The available CLI for LR1110 is seen by just typing only ``lr1110``. 
commands of interest   Additionally, there is ``wifi`` command in the CLI.   The auto-timer to trigger scans is accessible with ``lr1110 gt`` no argument to read value, or use zero to shut off timer.

## Reporting format to cloud:
Fragmentation to accommodate smaller MTU:  a single byte at the start of application-layer payload is used to indicate 3 items: message type, total number of fragments, and current fragment.
| bits of header byte | description |  
| ----------- | ----------- |  
| 6,7 | message type |  
| 3,4,5 | total fragments |
| 0,1,2 | current fragment |
* **WiFi**: is sent with message type 1 in the header, followed by the U-WIFILOC-MAC message as described at
https://www.loracloud.com/documentation/modem_services?url=mdmsvc.html#lora-edge-wi-fi-positioning-protocol
