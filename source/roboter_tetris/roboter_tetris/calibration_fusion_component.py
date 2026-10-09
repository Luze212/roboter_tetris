"""AICA service component for manually combining two base-camera calibrations.

This component has no robot or camera connections. Loading it never moves the
robot or writes a file; only its StringTrigger service performs the fusion.
"""

import math

from modulo_components.lifecycle_component import LifecycleComponent
from modulo_interfaces.srv import StringTrigger
import state_representation as sr

from .calibration_fusion import (
    DEFAULT_BASE_CAM_FILE, DEFAULT_OUTPUT_FILE, DEFAULT_ROBOT_CAM_FILE,
    fuse_calibration_files,
)


class BaseCamCalibrationFusion(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        self.add_parameter(sr.Parameter("base_cam_file", DEFAULT_BASE_CAM_FILE, sr.ParameterType.STRING),
                           "Ergebnis der eigenständigen BaseCam-Kalibrierung; relativ zum Paket oder absolut.")
        self.add_parameter(sr.Parameter("robot_cam_file", DEFAULT_ROBOT_CAM_FILE, sr.ParameterType.STRING),
                           "Ergebnis der Robot-Kamera-Hand-Auge-Kalibrierung.")
        self.add_parameter(sr.Parameter("output_file", DEFAULT_OUTPUT_FILE, sr.ParameterType.STRING),
                           "Separate Ergebnisdatei für die gewichtete Basiskamera-Pose.")
        self.add_parameter(sr.Parameter("robot_cam_weight_percent", 50.0, sr.ParameterType.DOUBLE),
                           "0 = nur BaseCam, 100 = nur Robot-Kamera-Hand-Auge; dazwischen Interpolation.")
        self.add_predicate("has_result", False)
        self.add_predicate("has_failed", False)
        self.add_service("write_fused_base_cam_calibration", StringTrigger, self._write_service)

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        if name in ("base_cam_file", "robot_cam_file", "output_file"):
            return not parameter.is_empty() and bool(str(parameter.get_value()).strip())
        if name == "robot_cam_weight_percent":
            if parameter.is_empty():
                return False
            try:
                weight = float(parameter.get_value())
            except (TypeError, ValueError):
                return False
            return math.isfinite(weight) and 0.0 <= weight <= 100.0
        return True

    def on_configure_callback(self) -> bool:
        return True

    def on_activate_callback(self) -> bool:
        self.set_predicate("has_result", False)
        self.set_predicate("has_failed", False)
        self.get_logger().info(
            "Kalibrierungs-Kombination bereit. Keine Bewegung; Datei nur über "
            "write_fused_base_cam_calibration schreiben."
        )
        return True

    def _write_service(self, request: StringTrigger.Request) -> StringTrigger.Response:
        response = StringTrigger.Response()
        try:
            result = fuse_calibration_files(
                self.get_parameter("base_cam_file").get_value(),
                self.get_parameter("robot_cam_file").get_value(),
                self.get_parameter("output_file").get_value(),
                self.get_parameter("robot_cam_weight_percent").get_value(),
            )
        except (OSError, TypeError, ValueError) as exc:
            self.set_predicate("has_result", False)
            self.set_predicate("has_failed", True)
            response.success = False
            response.message = f"Keine Ergebnisdatei geschrieben: {exc}"
            self.get_logger().error(response.message)
            return response
        self.set_predicate("has_result", True)
        self.set_predicate("has_failed", False)
        response.success = True
        response.message = (
            f"Aktive Datei: {result['output_path']}; Archiv: {result['archive_path']}; Robot-Kamera-Anteil "
            f"{result['robot_cam_weight_percent']:.1f} %; Quellenabstand "
            f"{result['translation_difference_mm']:.2f} mm / "
            f"{result['rotation_difference_deg']:.3f} Grad"
        )
        self.get_logger().info(response.message)
        return response

    def on_step_callback(self):
        pass
