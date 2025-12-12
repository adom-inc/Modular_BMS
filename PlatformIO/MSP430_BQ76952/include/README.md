Below are the instructions to download PlatformIO inside of the Adom docker container

1. Download the .vsix file for microsoft C/C++ tools inside native VS Code
2. Drag and drop this file into the extensions window inside your docker contrainer VS Codium
3. Ignore the error it gives you
4. Repeat steps 1 and 2 for PlatformIO from native VS Code
5. Install python 3.6 like it will suggest

**NOTE** Currently, only the platformIO CLI is working properly. Create your directories in the PIO CLI and your projects from there as well

Follow the next set of instructions needed for building the BMS project and possibly others on the MSP toolchain:

Install conda and activate the python 2.7 profile:


```sh
# install conda (follow prompts and accept bullshit ToS)
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh

# create python 2.7 profile
conda create -n py27 python=2.7
conda activate py27
```


Run this once in your terminal session before uploading (this is a hack to get python 2.7 libraries to be available like libpython2.7.so.1.0):

```sh
# change this if you installed miniconda somewhere else
export LD_LIBRARY_PATH="/root/miniconda3/pkgs/python-2.7.18-h42bf7aa_3/lib:$LD_LIBRARY_PATH"
```

Upload with:

```sh
pio run -t upload
```