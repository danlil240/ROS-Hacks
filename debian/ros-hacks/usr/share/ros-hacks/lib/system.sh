#!/usr/bin/env bash
# ==========================================================
# ROS-Hacks - System utilities (moved from functions.sh)
# ==========================================================

function fixJB() {
    local CLION_FILE=~/.local/share/applications/jetbrains-clion.desktop
    local PYCHARM_FILE=~/.local/share/applications/jetbrains-pycharm.desktop
    local PYCHARM_CE_FILE=~/.local/share/applications/jetbrains-pycharm-ce.desktop

    if [ -f "$CLION_FILE" ]; then
        printf "${GREEN_TXT}Patching CLion shortcut for ROS2 compatibility.${NC}\n"
        sed -i -e 's/Exec="/Exec=bash -i -c "/g' "$CLION_FILE"
        sed -i -e 's/Name=CLion/Name=ROS2 CLion/g' "$CLION_FILE"
    else
        printf "${YELLOW_TXT}CLion not found in ~/.local/share/applications${NC}\n"
    fi

    if [ -f "$PYCHARM_FILE" ]; then
        printf "${GREEN_TXT}Patching PyCharm Professional shortcut for ROS2 compatibility.${NC}\n"
        sed -i -e 's/Exec="/Exec=bash -i -c "/g' "$PYCHARM_FILE"
        sed -i -e 's/Name=PyCharm Professional/Name=ROS2 PyCharm Professional/g' "$PYCHARM_FILE"
    else
        printf "${YELLOW_TXT}PyCharm Professional not found in ~/.local/share/applications${NC}\n"
    fi

    if [ -f "$PYCHARM_CE_FILE" ]; then
        printf "${GREEN_TXT}Patching PyCharm CE shortcut for ROS2 compatibility.${NC}\n"
        sed -i -e 's/Exec="/Exec=bash -i -c "/g' "$PYCHARM_CE_FILE"
        sed -i -e 's/Name=PyCharm /Name=ROS2 PyCharm /g' "$PYCHARM_CE_FILE"
    else
        printf "${YELLOW_TXT}PyCharm CE not found in ~/.local/share/applications${NC}\n"
    fi
}
