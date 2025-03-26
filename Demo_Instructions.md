# dMIMO Demo Procedures

### Setup USRP N321/N320

```
cd usrp/
mkdir build/
cd build/
cmake ..
make -j4
./usrp_n321_setup
```

### Generate data files
```
cd matlab/
matlab -nodesktop -nosplash
>> sc_ncjt_sigen_all
>> mc_ncjt_sigen_all
>> su_mimo_sigen_2t2s
>> su_mimo_sigen_2t2s_csi
>> su_mimo_sigen_4t2s
>> su_mimo_sigen_4t4s
>> mu_mimo_ul_sigen_2t2s
>> mu_mimo_ul_sigen_4t2s
```

### Run the GRC tests

Open GRU Radio Companion, run the GRC tests in ```demo_grc/``` folder.






