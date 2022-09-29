#  Bash completion functions to support the rosman CLI

function _complete_rosman
{
  local arg opts
  COMPREPLY=()
  arg="${COMP_WORDS[COMP_CWORD]}"

  if [[ $COMP_CWORD == 1 ]]; then
    opts="node topic param service"
    COMPREPLY=($(compgen -W "$opts" -- ${arg}))
  elif [[ $COMP_CWORD == 2 ]] ; then # Should this be >= 2??
    case ${COMP_WORDS[1]} in
      node)
        opts=$(rosnode list 2> /dev/null)
        COMPREPLY=($(compgen -W "$opts" -- ${arg}))
        ;;
      topic)
        opts=$(rostopic list 2> /dev/null)
        COMPREPLY=($(compgen -W "$opts" -- ${arg}))
        ;;
      param)
        opts=$(rosparam list 2> /dev/null)
        COMPREPLY=($(compgen -W "$opts" -- ${args}))
        ;;
      service)
        opts=$(rosservice list 2> /dev/null)
        COMPREPLY=($(compgen -W "$opts" -- ${args}))
        ;;
    esac
  fi
}

complete -F "_complete_rosman" "rosman"
