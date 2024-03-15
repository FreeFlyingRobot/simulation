python := "/usr/bin/python3"

spawn_platform:
    ./scripts/build_model.mjs airstand/platform
    gz service -s /world/airstand_world/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "models/airstand/platform/platform.urdf", name: "urdf_model"'

gazebo WORLD="models/airstand/airstand.sdf":
    gz sim --render-engine ogre {{WORLD}}

clear_build:
    rm -rf "${EXTERNAL_DIR}/build" "${EXTERNAL_DIR}/log" "${EXTERNAL_DIR}/install"
    rm -rf "${PROJECT_ROOT}/build" "${PROJECT_ROOT}/log" "${PROJECT_ROOT}/install"
