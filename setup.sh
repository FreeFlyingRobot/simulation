toplevel_dir=`git rev-parse --show-toplevel`

echo "Root is - ${toplevel_dir}"

export PROJECT_ROOT=${toplevel_dir}

# Setup utils
export PROJECT_UTILS_ROOT=${PROJECT_ROOT}/_utils
export PROJECT_SCRIPTS_ROOT=${PROJECT_ROOT}/scripts

export PATH=${PROJECT_SCRIPTS_ROOT}:${PROJECT_UTILS_ROOT}:$PATH
export PYTHONPATH=${PROJECT_UTILS_ROOT}:$PYTHONPATH

source ${PROJECT_UTILS_ROOT}/base.sh
information "Sourced base script"

env_file="${PROJECT_ROOT}/.env"
test -f ${env_file} && source ${env_file} && information "Sourced ${env_file}"

# Dirs variabless
export EXTERNAL_DIR=${PROJECT_ROOT}/external
export MODELS_DIR=${PROJECT_ROOT}/models

pushd . > /dev/null
cd $PROJECT_ROOT
# Any commands relative to top dir go here
popd > /dev/null

# Usefull functions

function clear_build() {
    rm -rf "${EXTERNAL_DIR}/drivers/build" "${EXTERNAL_DIR}/drivers/devel" "${EXTERNAL_DIR}/drivers/install"
    rm -rf "${PROJECT_ROOT}/build" "${PROJECT_ROOT}/log" "${PROJECT_ROOT}/install"
}

install_setup="${PROJECT_ROOT}/install/setup.bash"
test -f ${install_setup} && source ${install_setup} && information "Sources install/setup.bash"

# Gazebo settings
export DISPLAY=:0

cecho -c 'green' "Done"
return 0
