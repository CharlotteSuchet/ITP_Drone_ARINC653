#!/bin/sh

# Constantes
HOST=192.168.1.1
LOGIN=Charles
PASS=a

cd /home/charles/Documents/ARDrone_RISLER/
ftp -in <<EOF
open $HOST 
user $LOGIN $PASS
delete acquisition_capteur.elf
put acquisition_capteur.elf
exit
EOF
echo 'ftp connection done'

telnet 192.168.1.1 << EOF
cd data/video
chmod +x acquisition_capteur.elf
exit
EOF

echo 'telnet connection done'



