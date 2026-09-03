"""
Autonomous Behavior Finite State Machine (FSM).
Handles transitions between Normal Navigation, Visual Feature Recovery, and Safe-Stop.
"""

from enum import Enum
import time


class NavState(Enum):
    NAVIGATING = "NAVIGATING"
    FEATURE_RECOVERY = "FEATURE_RECOVERY"
    SAFE_STOP = "SAFE_STOP"


class BehaviorStateMachine:
    """
    Finite state machine governing UGV high-level tactical autonomy and failsafes.
    """

    def __init__(
        self,
        min_features: int = 50,
        recovery_timeout_sec: float = 4.0,
        min_planner_freq_hz: float = 5.0
    ):
        self.min_features = min_features
        self.recovery_timeout = recovery_timeout_sec
        self.min_planner_freq = min_planner_freq_hz

        self.current_state = NavState.NAVIGATING
        self.state_enter_time = time.time()
        self.recovery_start_time = 0.0
        self.oscillation_phase = 0.0

    def update(
        self,
        tracked_features: int,
        planner_freq: float,
        cpu_temp: float = 55.0,
        critical_fault: bool = False
    ) -> NavState:
        """
        Evaluates system vitals and transitions states accordingly.

        Returns:
            current_state: Updated NavState.
        """
        now = time.time()

        # 1. Critical Watchdog Checks (Immediate Safe Stop)
        if critical_fault or (planner_freq < self.min_planner_freq and planner_freq > 0.0) or cpu_temp > 82.0:
            if self.current_state != NavState.SAFE_STOP:
                self.transition_to(NavState.SAFE_STOP)
            return self.current_state

        # 2. State-Specific Logic
        if self.current_state == NavState.NAVIGATING:
            if tracked_features < self.min_features and tracked_features >= 0:
                self.recovery_start_time = now
                self.transition_to(NavState.FEATURE_RECOVERY)

        elif self.current_state == NavState.FEATURE_RECOVERY:
            # Check if features have recovered
            if tracked_features >= self.min_features:
                self.transition_to(NavState.NAVIGATING)
            elif (now - self.recovery_start_time) > self.recovery_timeout:
                # Recovery timed out without regaining landmarks -> Safe Stop
                self.transition_to(NavState.SAFE_STOP)

        elif self.current_state == NavState.SAFE_STOP:
            # Stay in safe stop unless features and planner frequency are restored
            if not critical_fault and tracked_features >= self.min_features and planner_freq >= self.min_planner_freq:
                self.transition_to(NavState.NAVIGATING)

        return self.current_state

    def transition_to(self, new_state: NavState):
        self.current_state = new_state
        self.state_enter_time = time.time()

    def get_recovery_cmd(self, nominal_v: float) -> tuple:
        """
        Generates tactical recovery control: 50% forward speed with slow ±15° yaw oscillation
        to assist optical flow / visual feature re-acquisition.

        Returns:
            (v_override, omega_override)
        """
        elapsed = time.time() - self.recovery_start_time
        # In-place/slow crawl oscillation: frequency 0.5 Hz, amplitude 0.35 rad/s
        import math
        omega_osc = 0.35 * math.sin(2.0 * math.pi * 0.5 * elapsed)
        v_crawl = max(0.0, nominal_v * 0.5)
        return v_crawl, omega_osc
