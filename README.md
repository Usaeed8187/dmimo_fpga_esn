# dMIMO Demo


## Description
This folder contains GNU Radio implmentation for the dMIMO demos.


## Getting Started

Setup Git SSH command from Linux terminal (see https://code.vt.edu/help/user/ssh).
```
export GIT_SSH_COMMAND="ssh -i ~/.ssh/id_ed25519"
```
Clone the **main** branch of this repository.
```
cd <workspace_dir>
git clone git@code.vt.edu:yiliang/dmimodemo
cd dmimodemo


## NCJT Demo

Build the GNU Radio application as follows.
```
cd gr_ncjt
mkdir build
cd build/
cmake ..
make
make install
```

