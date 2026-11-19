# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""ROS 2 SegmentText service that runs the merged SAM3 QNN context on the IQ10's NPU through QAI AppBuilder.

Runs as root in the sam3_npu sidecar container, the only route to FastRPC on that board; start it with
scripts/start_sam3_npu.sh, which sets the QNN and DSP library paths. The model and AppBuilder are host
files (SAM3_NPU_DIR), not part of this package. It commands nothing on the robot.
"""

import os
import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from shelf_stocking_msgs.srv import SegmentText

from sam3_npu_server import core
from sam3_npu_server.tokenizer import Sam3Tokenizer

EXPECTED_INPUTS = {"image", "text_tokens"}
EXPECTED_OUTPUTS = {"pred_logits", "pred_masks", "presence_logit_dec"}


class Sam3NpuServer(Node):
    def __init__(self):
        super().__init__("sam3_npu_server")
        self.declare_parameter("model_path", "")
        # Text such as "0" or "0,1,2,3"; a bare digit would otherwise arrive as an integer.
        self.declare_parameter("cores", "0", ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("service_name", "/sam3_npu/segment_text")
        self.declare_parameter("max_prompts", 16)
        model_path = self.get_parameter("model_path").value
        cores = str(self.get_parameter("cores").value)
        self.max_prompts = int(self.get_parameter("max_prompts").value)
        if not model_path or not os.path.isfile(model_path):
            raise RuntimeError(f"model_path '{model_path}' is not a file")
        lib_dir = os.environ.get("QNN_HOST_LIB_DIR", "")
        if not os.path.isfile(os.path.join(lib_dir, "libQnnHtp.so")):
            raise RuntimeError(f"QNN_HOST_LIB_DIR '{lib_dir}' has no libQnnHtp.so; start through start_sam3_npu.sh")

        from qai_appbuilder import DataType, LogLevel, PerfProfile, ProfilingLevel, QNNConfig, QNNContext, Runtime

        self._burst = PerfProfile.BURST
        QNNConfig.Config(Runtime.HTP, LogLevel.WARN, ProfilingLevel.OFF)
        started = time.perf_counter()
        self.qnn = QNNContext(
            model_name="sam3_npu",
            model_path=model_path,
            backend_lib_path=os.path.join(lib_dir, "libQnnHtp.so"),
            system_lib_path=os.path.join(lib_dir, "libQnnSystem.so"),
            input_data_type=DataType.FLOAT,
            output_data_type=DataType.FLOAT,
            coreIdsStr=cores,
        )
        self.input_names = [core.logical_name(n) for n in self.qnn.getInputName()]
        self.output_names = [core.logical_name(n) for n in self.qnn.getOutputName()]
        self.output_shapes = [tuple(s) for s in self.qnn.getOutputShapes()]
        # AppBuilder logs a failed context load and carries on with no graph, so check what it loaded.
        if set(self.input_names) != EXPECTED_INPUTS or not EXPECTED_OUTPUTS <= set(self.output_names):
            raise RuntimeError(f"{model_path} did not load as the merged SAM3 graph: inputs {self.input_names}, "
                               f"outputs {self.output_names}. Check the QNN log above and the board's NPU.")
        self.get_logger().info(f"SAM3 context loaded on the NPU in {time.perf_counter() - started:.1f} s from "
                               f"{model_path} (cores {cores})")
        self.tokenize = Sam3Tokenizer(context_length=core.TOKEN_COUNT)
        self.lock = threading.Lock()
        self._warm_up()
        service = self.get_parameter("service_name").value
        self.service = self.create_service(SegmentText, service, self._on_segment_text)
        self.get_logger().info(f"Serving {service}")

    def _warm_up(self):
        """One inference before serving, so the first request does not pay the HTP's first-run setup."""
        started = time.perf_counter()
        self.infer(np.zeros((1, 3, core.INPUT_SIZE, core.INPUT_SIZE), np.float32), "an object")
        self.get_logger().info(f"Warm-up inference took {time.perf_counter() - started:.2f} s")

    def infer(self, image_nchw: np.ndarray, prompt: str) -> dict:
        tokens = np.asarray(self.tokenize(prompt), dtype=np.float32).reshape(1, core.TOKEN_COUNT)
        feeds = {"image": image_nchw, "text_tokens": tokens}
        with self.lock:
            outputs = self.qnn.Inference([np.ascontiguousarray(feeds[n], dtype=np.float32) for n in self.input_names],
                                         self._burst)
        return {name: np.asarray(value, dtype=np.float32).reshape(shape)
                for name, shape, value in zip(self.output_names, self.output_shapes, outputs)}

    def _on_segment_text(self, request, response):
        response.success = False
        try:
            if not request.prompts or len(request.prompts) > self.max_prompts:
                raise ValueError(f"send 1 to {self.max_prompts} prompts, got {len(request.prompts)}")
            if any(not p.strip() for p in request.prompts):
                raise ValueError("prompts must not be empty")
            if not 0.0 <= request.confidence_threshold <= 1.0:
                raise ValueError("confidence_threshold must be within 0 and 1")
            img = request.image
            image_nchw = core.preprocess(core.image_to_rgb(img.encoding, img.height, img.width, img.step, img.data))
            elapsed = 0.0
            for index, prompt in enumerate(request.prompts):
                started = time.perf_counter()
                out = self.infer(image_nchw, prompt)
                elapsed += time.perf_counter() - started
                for _, score, mask in core.select_detections(out["pred_logits"], out["presence_logit_dec"],
                                                             out["pred_masks"], request.confidence_threshold,
                                                             img.height, img.width):
                    m = Image()
                    m.header = img.header
                    m.height, m.width, m.encoding, m.step = img.height, img.width, "mono8", img.width
                    m.data = mask.tobytes()
                    response.masks.append(m)
                    response.scores.append(score)
                    response.prompt_index.append(index)
            response.inference_ms = float(elapsed * 1000.0)
            response.success = True
            response.message = f"{len(response.masks)} masks from {len(request.prompts)} prompts"
            self.get_logger().info(f"{response.message} in {response.inference_ms:.0f} ms: {list(request.prompts)}")
        except Exception as error:  # noqa: BLE001 -- every failure goes back to the caller
            response.masks, response.scores, response.prompt_index = [], [], []
            response.message = f"{type(error).__name__}: {error}"
            self.get_logger().error(f"segment_text failed: {response.message}")
        return response


def main():
    rclpy.init()
    node = Sam3NpuServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # docker stop sends SIGTERM, which rclpy turns into an external shutdown.
        pass
    finally:
        # Release the context explicitly: a process that exits holding DSP buffers has leaked them on
        # Qualcomm boards before (see design_docs/iq10_sam3_npu_sidecar.md).
        node.qnn.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
