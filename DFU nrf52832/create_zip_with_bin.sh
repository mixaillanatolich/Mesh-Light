echo End Flash DFU

echo Start Firmware Update

echo 1. Create DFU achive
nrfutil dfu genpkg --application light_nrf52832_xxAA_s132_7.0.1.hex \
    --company-id 0x0000F0A5 \
    --application-id 1 \
    --application-version 1 \
    --sd-req 0x009D \
    --mesh dfu_bin.zip
    