# If not running interactively, don't do anything
[ -z "$PS1" ] && return

export PS1="\[\e[36m\]spot\[\e[m\] \[\e[33m\]\w\[\e[m\] > "
export TERM=xterm-256color
alias grep="grep --color=auto"
alias ls="ls --color=auto"

echo -e "\e[1;31m"
cat<<TF
Welcome to the Spot Docker Image
TF
echo -e "\e[0;33m"

if [[ $EUID -eq 0 ]]; then
  echo 'export PS1="\[\e[31m\]spot\[\e[m\] \[\e[33m\]\w\[\e[m\] > "' >> /root/.bashrc
  cat <<WARN
WARNING: You are running this container as root, which can cause new files in
mounted volumes to be created as the root user on your host machine.
To avoid this, run the container by specifying your user's userid:
$ docker run -u \$(id -u):\$(id -g) args...
WARN
else
  cat <<EXPL
You are running this container as user with ID $(id -u) and group $(id -g),
which should map to the ID and group for your user on the Docker host. Great!
EXPL
fi

# Turn off colors
echo -e "\e[m"