# Prepare your pi2go to rock

- Install Jessie lite
Jessie is too big (especially for 4Go SD card). Make sure your are battery powered while the installation
- Make specific password
- update the distrib
- install vim
- install wicd-curses
- configure wifi access
- get a fixed IP
- make it accessible from outside
- Add ssh key
- install python3 (apt-get)
- install pip (apt-get)
- install ratom (https://atom.io/packages/remote-atom)
- pi2go http://www.pi2go.co.uk/
- install git
- sudo apt-get install python-smbus
- raspbian config
   - Advanced options, setup I2C and load on boot
   - Timezone, Region and keyborad
   - update tool
   - hostname
- install jupyter (via pip3) see: http://jupyter.readthedocs.org/en/latest/install.html -> python3-dev packages needed apt-get install build-essential python3-dev
- config for root user (.bashrc)
- configure jupyter notebook server (http://jupyter-notebook.readthedocs.org/en/latest/public_server.html)
- Add support for python2: http://ipython.readthedocs.org/en/latest/install/kernel_install.html
- Have jupyter on startup (https://www.raspberrypi.org/documentation/linux/usage/rc-local.md)
- install smbus for python3
sudo apt-get update
sudo apt-get install python3-smbus


## PythonPath ##
- pi2go.py and other modules in /home/pi/libs
- Add code in .bashrc of pi user (it's loaded by root .bashrc)
# PYTHON PATH
export PYTHONPATH="${PYTHONPATH}:/home/pi/libs"
- Jupyter handles path differently via ipython so we must add it in ipython startup dir:
/root/.ipython/profile_default/startup/00-pythonpath.py
import sys
sys.path.append('/home/pi/libs')

## To Do ##
- Use https (test for perf) on Jupyter ?
- Change pi user (issue now with pi2go)
