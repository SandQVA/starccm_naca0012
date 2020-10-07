# CFDcoupling STARCCM
simili gym environment to be used in conjunction with the SandQVA/cfd_sureli repository to do DRL of a flying flat plate.

Requirements:
- Python 3.7 or older versions
- torch, torchvision
- imageio
- gym
- matplotlib
- PyYAML
- numpy

Usage : 
- Clone the SandQVA/cfd_sureli repository
- Inside this repository create a cfd folder
- Go to this new folder and clone .../starccm repository
- Download from the following link (https://nextcloud.isae.fr/f/144526995) the folder containing the simulations and copy them in cfd_sureli/cfd/starccm/
- Set the config.yaml file contained in cfd_sureli/cfd/starccm/ according to the case you want to run
- run the command python3 train 'your_agent'
- run the starccm simualtion .sim with the macro macro_externalfiles.java (always before running python)
  starccm -batch macro_externalfiles.java naca0012coarse.sim
- run the command python3 test 'your_agent' --f='file created by the training phase'
- run the starccm simualtion .sim with the macro macro_externalfiles.java (always before running python)
- to obtaning images of the test:
- in the folder cfd_sureli/cfd/starccm/actions create a folder with the a ...bestactions.csv with two columns: 1 time, 2 action to perform and a ...tmax.txt with maximum evaluation time
- edit the file macro_path.java with inital, final positions and the path and folder where the previous files are located
- run the simulation ...images.sim with the macro
