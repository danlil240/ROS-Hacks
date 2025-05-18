#!/bin/bash

source aliases.sh
printf "${LIGHT_BLUE_TXT}Installing ROS-Hacks${NC}.\n"

# Add to ~/.bashrc:
echo "source ~/.ROS-Hacks/ROS-Hacks.sh" >>~/.bashrc

# Replace ~/.inputrc
if [[ -f ~/.inputrc ]];then
    mv ~/.inputrc ~/.inputrc.bak
fi
ln -nsf ${PWD}/inputrc ~/.inputrc

# Add crontab updater
#crontab -l > mycron
##echo "* 00 * * * $PWD/updater.sh" >> mycron
#crontab mycron
#rm mycron
