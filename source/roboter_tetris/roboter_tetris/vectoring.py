"""AICA lifecycle component `vectoring` -- velocity estimation (project goal 3).

Thin shell around :class:`roboter_tetris.track_estimation.TrackEstimator`, which
holds the whole procedure and is tested on its own. This file only wires the
estimator to S1 (``objects`` from `base_cam`) and S3 (``tracks``).

One activation is one run: the pooled belt velocity starts empty and grows with
every block that settles.
"""

from modulo_components.lifecycle_component import LifecycleComponent
import state_representation as sr
from std_msgs.msg import Float64MultiArray

from .contracts import ContractError, pack_tracks, unpack_objects
from .track_estimation import EstimatorParams, TrackEstimator

# No new S1 frame for this long -> input counts as stalled (wall-clock seconds).
STALE_TIMEOUT_S = 1.0
ERROR_LOG_PERIOD_S = 1.0


class Vectoring(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # -- Parameters (operator-facing descriptions) ----------------------------
        self.add_parameter(
            sr.Parameter("settle_half_window", 5, sr.ParameterType.INT),
            "Messungen je Halbfenster des Einschwingtests - zählt Messungen, nicht Zeit. "
            "Am Aufbau liefert base_cam 5-7 Messungen/s (Nachtrag 13): 5 = 0,7-1 s. "
            "Ein Klotz gilt als eingeschwungen, wenn die Geschwindigkeit beider "
            "Halbfenster übereinstimmt.")
        self.add_parameter(
            sr.Parameter("settle_v_tolerance", 0.010, sr.ParameterType.DOUBLE),
            "So weit (m/s) dürfen die Geschwindigkeiten der beiden Halbfenster "
            "auseinanderliegen, damit sie als konstant gelten. Zu klein: Klötze "
            "schwingen bei Rauschen nie ein. Zu groß: ein kippender Klotz wird zu "
            "früh final. (D20)")
        self.add_parameter(
            sr.Parameter("outlier_distance_m", 0.02, sr.ParameterType.DOUBLE),
            "Abstand (m) zur Vorhersage, ab dem eine einzelne Messung als "
            "Fehldetektion verworfen wird. (D21)")
        self.add_parameter(
            sr.Parameter("outlier_persist_frames", 3, sr.ParameterType.INT),
            "So viele Ausreißer in Folge gelten als echte Lageänderung — etwa ein "
            "umgekippter Klotz — und starten den Track neu. (D21)")
        self.add_parameter(
            sr.Parameter("smoothing_window", 30, sr.ParameterType.INT),
            "Anzahl Messungen, über die Position, Orientierung und Abmessungen "
            "gemittelt werden — erst ab dem Einschwingen.")
        self.add_parameter(
            sr.Parameter("track_expiry_s", 0.5, sr.ParameterType.DOUBLE),
            "Nach dieser Zeit (s) ohne Messung wird ein Track vergessen. Sein "
            "Beitrag zur Bandgeschwindigkeit bleibt erhalten.")

        # -- Input / output (std_msgs signals are plain Python values) -----------
        self._objects_in = []
        self.add_input("objects", "_objects_in", Float64MultiArray)
        # S3 is never empty: before the first frame t = 0 and n_pool = 0.
        self._tracks = pack_tracks(0.0, (0.0, 0.0), 0, [])
        self.add_output("tracks", "_tracks", Float64MultiArray)

        # -- Predicates -----------------------------------------------------------
        self.add_predicate("has_tracks", False)
        self.add_predicate("has_belt_estimate", False)
        self.add_predicate("is_receiving", False)

        # -- State ----------------------------------------------------------------
        self._estimator = TrackEstimator()
        self._last_t = None
        self._last_frame_walltime = None
        self._last_error_walltime = None

    # -- Validation ---------------------------------------------------------------

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        if parameter.is_empty():
            self.get_logger().warn(f"{name} must not be empty")
            return False
        value = parameter.get_value()
        if name in ("settle_half_window", "smoothing_window") and value < 2:
            self.get_logger().warn(f"{name} must be at least 2")
            return False
        if name == "outlier_persist_frames" and value < 1:
            self.get_logger().warn("outlier_persist_frames must be at least 1")
            return False
        if name in ("settle_v_tolerance", "outlier_distance_m",
                    "track_expiry_s") and value <= 0.0:
            self.get_logger().warn(f"{name} must be positive")
            return False
        return True

    def _params(self) -> EstimatorParams:
        return EstimatorParams(
            settle_half_window=int(self.get_parameter("settle_half_window").get_value()),
            settle_v_tolerance=self.get_parameter("settle_v_tolerance").get_value(),
            outlier_distance_m=self.get_parameter("outlier_distance_m").get_value(),
            outlier_persist_frames=int(
                self.get_parameter("outlier_persist_frames").get_value()),
            smoothing_window=int(self.get_parameter("smoothing_window").get_value()),
            track_expiry_s=self.get_parameter("track_expiry_s").get_value(),
        )

    # -- Lifecycle ----------------------------------------------------------------

    def on_configure_callback(self) -> bool:
        return True

    def on_activate_callback(self) -> bool:
        # A new run: the belt estimate starts empty (Nachtrag 6 / Z2).
        self._estimator = TrackEstimator(self._params())
        self._last_t = None
        self._last_frame_walltime = None
        self._tracks = pack_tracks(0.0, (0.0, 0.0), 0, [])
        for name in ("has_tracks", "has_belt_estimate", "is_receiving"):
            self.set_predicate(name, False)
        return True

    def on_deactivate_callback(self) -> bool:
        return True

    # -- Helpers ------------------------------------------------------------------

    def _log_error_throttled(self, message: str) -> None:
        now = self.get_clock().now()
        if (self._last_error_walltime is None
                or (now - self._last_error_walltime).nanoseconds / 1e9 >= ERROR_LOG_PERIOD_S):
            self.get_logger().error(message)
            self._last_error_walltime = now

    def _handle_stale(self) -> None:
        """No new frame for a while: report no tracks, keep the belt estimate.

        The header keeps its last ``t``, so a receiver sees the input has stopped.
        """
        if self._last_frame_walltime is None:
            return
        age_s = (self.get_clock().now() - self._last_frame_walltime).nanoseconds / 1e9
        if age_s > STALE_TIMEOUT_S:
            v_belt, n_pool = self._estimator.belt_velocity()
            self._tracks = pack_tracks(self._last_t, v_belt or (0.0, 0.0), n_pool, [])
            self.set_predicate("is_receiving", False)
            self.set_predicate("has_tracks", False)

    # -- Periodic processing --------------------------------------------------------

    def on_step_callback(self):
        try:
            msg = unpack_objects(self._objects_in)
        except ContractError as exc:
            self._log_error_throttled(f"vectoring: S1 verworfen — {exc}")
            return
        if msg is None or msg.t == self._last_t:
            self._handle_stale()              # nothing yet, or the same frame again
            return
        self._last_t = msg.t
        self._last_frame_walltime = self.get_clock().now()

        # Parameters may be changed live in the UI; hand the current set over.
        self._estimator.params = self._params()
        self._estimator.update(msg.t, msg.objects)
        v_belt, n_pool, entries = self._estimator.snapshot(msg.t)
        self._tracks = pack_tracks(msg.t, v_belt or (0.0, 0.0), n_pool, entries)

        self.set_predicate("is_receiving", True)
        self.set_predicate("has_belt_estimate", n_pool > 0)
        self.set_predicate("has_tracks", any(e.status == 0 for e in entries))
