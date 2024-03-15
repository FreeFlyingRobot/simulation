toplevel_dir=`git rev-parse --show-toplevel`

echo "Root is - ${toplevel_dir}"

export PROJECT_ROOT=${toplevel_dir}

# Setup utils
export PROJECT_UTILS_ROOT=${PROJECT_ROOT}/_utils
export PROJECT_SCRIPTS_ROOT=${PROJECT_ROOT}/scripts

export PATH=${PROJECT_SCRIPTS_ROOT}:$PATH

source ${PROJECT_UTILS_ROOT}/base.sh
information "Sourced base script"

env_file="${PROJECT_ROOT}/.env"
test -f ${env_file} && source ${env_file} && information "Sourced ${env_file}"

# Dirs variabless
export EXTERNAL_DIR=${PROJECT_ROOT}/external
export MODELS_DIR=${PROJECT_ROOT}/models

# Setup python
export PYTHONPATH=${PROJECT_UTILS_ROOT}:$PYTHONPATH

# Setup ROS
install_setup="${PROJECT_ROOT}/install/setup.bash"
test -f ${install_setup} && source ${install_setup} && information "Sourced install/setup.bash"

# Setup Gazebo
export SDF_PATH=${SDF_PATH}:${MODELS_DIR}
export DISPLAY=:0

# Entire script should be location independent, so push/pop dir to run any top-level commands
pushd . > /dev/null
cd $PROJECT_ROOT
# Any commands relative to top dir go here
popd > /dev/null

success "Done"
return 0
