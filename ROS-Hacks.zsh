#!/usr/bin/env zsh

setopt PROMPT_SUBST

ROSHACKS_DIR="$(cd -- "$(dirname -- "${(%):-%x}")" >/dev/null 2>&1 && pwd)"
ROSHACKS_LIB_DIR="${ROSHACKS_DIR}/lib"

if [[ -f "${ROSHACKS_DIR}/aliases.sh" ]]; then
  source "${ROSHACKS_DIR}/aliases.sh"
else
  print "[ROS-Hacks] Warning: Could not find aliases.sh file." >&2
fi

if [[ -f "${ROSHACKS_DIR}/functions.sh" ]]; then
  source "${ROSHACKS_DIR}/functions.sh"
else
  print "[ROS-Hacks] Error: Could not find functions.sh file." >&2
  return 1
fi

if [[ -d "${ROSHACKS_LIB_DIR}" ]]; then
  for _rh_mod in "${ROSHACKS_LIB_DIR}"/*.sh; do
    [[ -f "${_rh_mod}" ]] && source "${_rh_mod}"
  done
  unset _rh_mod
fi

if [[ -f "${ROSHACKS_DIR}/zsh-keybindings.zsh" ]]; then
  source "${ROSHACKS_DIR}/zsh-keybindings.zsh"
fi

if ! typeset -f __git_ps1 >/dev/null 2>&1; then
  __git_ps1() {
    local b
    b=$(git rev-parse --abbrev-ref HEAD 2>/dev/null) || return 0
    [[ -n "$b" ]] || return 0
    printf " (%s)" "$b"
  }
fi

PROMPT=$'%F{green}%n%f %F{green}$(get_current_ws_name):$ROS_DOMAIN_ID%f %F{blue}%~%f%F{cyan}$(__git_ps1)%f:\n%(?.%F{green}➜.%F{red}➜)%f '

get_current_ws
if [[ -n "${curr_ws:-}" && -d "${curr_ws}" ]]; then
  source_ws "${curr_ws}"
fi
