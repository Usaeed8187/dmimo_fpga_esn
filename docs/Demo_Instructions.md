# dMIMO Demo Procedures

### Install UHD and GNU Radio
1. Install UHD 4.8 to the Linux system. 
2. Install GNU Radio according to the guide (docs/GNURadio_Setup.md).

### Setup USRP N321
After each power cycle, setup USRP N321 to enable LO export. 
```
cd usrp/
mkdir build/
cd build/
cmake ..
make -j4
./usrp_n321_setup
```

### Generate data files
Generate data files using Matlab. These data include synchronization preambles, 
LTF sequences for channel estimation, and random source data for BER calculation.
```
cd matlab/
matlab -nodesktop -nosplash
>> ncjt_sc_sigen_all
>> ncjt_mc_sigen_all
>> ncjt_sc_sigen_he_all
>> ncjt_mc_sigen_he_all
>> su_mimo_sigen_2t2s
>> su_mimo_sigen_2t2s_csi
>> su_mimo_sigen_4t2s
>> su_mimo_sigen_4t4s
>> su_mimo_sigen_he_2t2s
>> su_mimo_sigen_he_4t2s
>> mu_mimo_ul_sigen_2t2s
>> mu_mimo_ul_sigen_4t2s
>> mu_mimo_ul_sigen_he_2t2s
```

### Running the dMIMO QTR2 demo

Open GRU Radio Companion, run the GRC tests in ```demo_grc/sc_ncjt_qtr2/``` folder.

| GRC filename             | Description                  | Notes                        | 
|--------------------------|------------------------------|------------------------------|
| NCJT_SC_TxS_gNB_N310.grc | TX Squad gNB top-level model | Start first                  |
| NCJT_SC_TxUE1_X310.grc   | TX Squad UE1 top-level model | Use OctoClock for Tx UEs     |
| NCJT_SC_TxUE2_X310.grc   | TX Squad UE2 top-level model |                              |
| NCJT_SC_RxUE1_X310.grc   | RX Squad UE1 top-level model | Use stable clocks for Rx UEs |
| NCJT_SC_RxUE2_X310.grc   | RX Squad UE2 top-level model |                              |
| NCJT_SC_RxS_gNB_PDC_N310.grc | Rx Squad gNB top-level model |                          |   | 

For video streaming demo, run the Python script ```streamer.py``` in the ```grc-ncjt/``` folder.
Select streaming using video file, H.265 codec, and a source video file. 
Use ```Restart Stream``` to start or reset video streaming.

