#!/bin/bash
# Entry point of the sam3_npu sidecar container (docker-compose.yaml). Runs as root: on the IQ10 only root
# reaches the NPU's FastRPC daemon. The image entrypoint is skipped (its user setup fails as root), so this
# repeats the two parts of it the server needs: the ROS and workspace setup, and the DDS configuration, from
# MoveIt Pro's own scripts so the sidecar joins the Runtime's DDS graph exactly as the Runtime does.
# Host files come from SAM3_NPU_DIR (mounted at the same path): sam3_merged.bin and appbuilder/ (QAI
# AppBuilder unpacked from its wheel, with its bundled QNN runtime). See design_docs/iq10_sam3_npu_sidecar.md.
set -eo pipefail
SAM3_NPU_DIR=${SAM3_NPU_DIR:?set SAM3_NPU_DIR to the host folder holding sam3_merged.bin and appbuilder/}
MODEL=${SAM3_NPU_MODEL:-$SAM3_NPU_DIR/sam3_merged.bin}
APPBUILDER=${SAM3_NPU_APPBUILDER:-$SAM3_NPU_DIR/appbuilder}
CORES=${SAM3_NPU_CORES:-0}
for f in "$MODEL" "$APPBUILDER/qai_appbuilder/libs/libQnnHtp.so"; do
  [[ -f "$f" ]] || { echo "start_sam3_npu.sh: $f not found" >&2; exit 1; }
done

# Pro's scripts are written to be sourced by its entrypoint, not under set -e: a harmless non-zero
# command inside them would end this script. Their results are checked below instead.
set +eu
# shellcheck source=/dev/null
source /moveit_studio_utils/setup_workspaces.sh
# shellcheck source=/dev/null
source /moveit_studio_utils/copy_dds_configs.sh
set -e
[[ -n "$CYCLONEDDS_URI" || "$RMW_IMPLEMENTATION" != rmw_cyclonedds_cpp ]] || { echo "no DDS configuration" >&2; exit 1; }
ros2 pkg prefix sam3_npu_server > /dev/null || { echo "sam3_npu_server is not built in $USER_WS" >&2; exit 1; }
export ROS_LOG_DIR=/tmp/sam3_npu_ros_log

# AppBuilder bundles the QNN runtime with the HTP V81 stub and skeleton. The NSP domains look the skeleton
# up through the CDSP*_LIBRARY_PATH variables, not ADSP_LIBRARY_PATH alone: with only the latter set, the
# DSP fails to load it (openSessionForPriority 0x80000406, "Device Creation failure").
QNN_LIBS="$APPBUILDER/qai_appbuilder/libs"
export QNN_HOST_LIB_DIR="$QNN_LIBS"
export LD_LIBRARY_PATH="$QNN_LIBS:${LD_LIBRARY_PATH:-}"
DSP_PATHS="$QNN_LIBS;/vendor/dsp/cdsp;/vendor/dsp/cdsp0;/usr/share/fastrpc;/usr/lib/rfsa/adsp"
export ADSP_LIBRARY_PATH="$DSP_PATHS" CDSP_LIBRARY_PATH="$DSP_PATHS" CDSP0_LIBRARY_PATH="$DSP_PATHS"
export CDSP1_LIBRARY_PATH="$DSP_PATHS" CDSP2_LIBRARY_PATH="$DSP_PATHS" CDSP3_LIBRARY_PATH="$DSP_PATHS"
# Only AppBuilder is added, so ROS keeps the image's NumPy.
export PYTHONPATH="$APPBUILDER:$PYTHONPATH"

PARAMS="$(ros2 pkg prefix sam3_npu_server)/share/sam3_npu_server/config/sam3_npu.yaml"
exec python3 -m sam3_npu_server.server --ros-args --params-file "$PARAMS" \
  -p model_path:="$MODEL" -p cores:="$CORES"
