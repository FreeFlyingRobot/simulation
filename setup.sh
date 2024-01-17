toplevel_dir=`git rev-parse --show-toplevel`

echo "Root is - ${toplevel_dir}"

export PROJECT_ROOT=${toplevel_dir}

# Setup utils
export PROJECT_UTILS_ROOT=${PROJECT_ROOT}/_utils
source ${PROJECT_UTILS_ROOT}/base.sh

# Dirs variabless
export PATH=${PROJECT_SCRIPTS_ROOT}:$PATH
export EXTERNAL_DIR=${PROJECT_ROOT}/external

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

cecho -c 'green' "Sourced all"
return 0
