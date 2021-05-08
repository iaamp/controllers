# olympe SDK troubleshooting
* PYTHONPATH=
/home/erl/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/erl/code/parrot-groundsdk/packages/olympe/src:/home/erl/code/parrot-groundsdk/out/olympe-linux/final/usr/lib/python:/home/erl/code/parrot-groundsdk/out/olympe-linux/final/usr/lib/python/site-packages:/home/erl/code/parrot-groundsdk/out/olympe-linux/final/usr/local/lib/python:/home/erl/code/parrot-groundsdk/out/olympe-linux/final/usr/local/lib/python/site-packages:/home/erl/code/parrot-groundsdk/out/olympe-linux/staging-host/usr/lib/arsdkgen:/usr/lib/python3/dist-packages
* sdk/products/olympe/linux/env/shell -->
#!/bin/bash
[[ $0 != $BASH_SOURCE ]] && SCRIPT_PATH=$(realpath $BASH_SOURCE) || SCRIPT_PATH="`readlink -f "$    0"`"
ENV_DIR="`dirname "$SCRIPT_PATH"`"

bash -rcfile <(echo "
  source ~/.bashrc
  PS1=\"(olympe-python3) \${PS1}\"
  source ${ENV_DIR}/setenv
  alias python=python3
  source /opt/ros/noetic/setup.bash
  source ~/catkin_ws/devel/setup.bash
")

WHATEVERPATH=$PYTHONPATH
echo "***************"
echo $WHATEVERPATH
echo "***************"
export PYTHONPATH=$WHATEVERPATH:"/usr/lib/python3/dist-packages"
echo "***************"
echo $PYTHONPATH
echo "***************"

