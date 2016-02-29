# friendly-pi2go
Some tools/notes for my pi2go robot

## Done ##
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

## To Do ##
- Have jupyter on startup
- Use https (test for perf) on Jupyter ?
- configure dir and python path
- make the libs python3 compatible
- Change pi user (issue now with pi2go)


## Not done ##
might be an issue ?
- sudo nano /etc/modules and add “i2c-dev” as the last line
