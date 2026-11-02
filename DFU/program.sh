nrfjprog --eraseall
sleep 0.5
nrfjprog --program s113_nrf52_7.0.1_softdevice.hex --chiperase --verify
sleep 0.5
nrfjprog --program mesh_bootloader_gccarmemb_nrf52833_xxAA.hex --verify
sleep 0.5
nrfjprog --program light_switch_server_nrf52833_xxAA_s113_7.0.1.hex --verify
sleep 0.5
nrfjprog --program device_page_nrf52833_xxAA_s113_7.0.1.hex --verify
sleep 0.5
nrfjprog --reset
