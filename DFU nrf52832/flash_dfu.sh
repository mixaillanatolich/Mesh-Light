echo Start

echo 1. Erase All
nrfjprog -f NRF52 --eraseall

echo 2. Soft Device
nrfjprog -f NRF52 --program s132_nrf52_7.0.1_softdevice.hex --chiperase

echo 3. Bootloader
nrfjprog -f NRF52 --program mesh_bootloader_gccarmemb_nrf52832_xxAA.hex

echo 4. DFU Example
nrfjprog -f NRF52 --program light_nrf52832_xxAA_s132_7.0.1.hex

echo 5. Device Page
cd tools/dfu/
python device_page_generator.py -d nrf52832_xxAA -sd "s132_7.0.1" -o "bin/device_page_nrf52832_xxAA_s132_7.0.1.hex"
cd ../../
nrfjprog --program tools/dfu/bin/device_page_nrf52832_xxAA_s132_7.0.1.hex

echo 6. Reset
nrfjprog --reset

echo End
