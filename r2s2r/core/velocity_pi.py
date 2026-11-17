"""PI correction in command space; does not alter policy actions or joint gains."""
from dataclasses import dataclass
import numpy as np

@dataclass
class PISettings:
    kp: np.ndarray
    ki: np.ndarray
    integral_limit: np.ndarray
    correction_limit: np.ndarray
    slew_rate: np.ndarray
    filter_tau: float = 0.25
    warmup_seconds: float = 1.0
    zero_threshold: float = 0.001

    @classmethod
    def from_dict(cls, values):
        return cls(**values)

    def __post_init__(self):
        for name in ("kp", "ki", "integral_limit", "correction_limit", "slew_rate"):
            value = np.asarray(getattr(self, name), dtype=np.float64)
            if value.shape != (3,) or not np.isfinite(value).all() or np.any(value < 0):
                raise ValueError(f"{name} must contain three finite nonnegative values")
            setattr(self, name, value)
        if not np.isfinite([self.filter_tau, self.warmup_seconds, self.zero_threshold]).all():
            raise ValueError("PI timing and zero threshold must be finite")
        if min(self.filter_tau, self.warmup_seconds, self.zero_threshold) < 0:
            raise ValueError("PI timing and zero threshold must be nonnegative")

class VelocityPI:
    def __init__(self, settings, command_limit):
        self.settings = settings
        self.limit = np.asarray(command_limit, dtype=np.float64)
        if self.limit.shape != (3,) or not np.isfinite(self.limit).all() or np.any(self.limit <= 0):
            raise ValueError("command_limit must contain three finite positive values")
        self.reset()

    def reset(self, reference=None, measurement=None):
        self.integral = np.zeros(3)
        self.filtered = None if measurement is None else np.asarray(measurement, dtype=np.float64).copy()
        self.previous_reference = np.zeros(3) if reference is None else np.asarray(reference, dtype=np.float64).copy()
        self.previous_output = np.clip(self.previous_reference, -self.limit, self.limit)
        self.saturated = np.zeros(3, dtype=bool)
        self.active = False

    def update(self, reference, measurement, dt, enabled=True):
        reference = np.asarray(reference, dtype=np.float64)
        measurement = np.asarray(measurement, dtype=np.float64)
        if reference.shape != (3,) or measurement.shape != (3,):
            raise ValueError("reference and measurement must contain three values")
        if not np.isfinite(reference).all() or not np.isfinite(measurement).all() or not np.isfinite(dt) or dt <= 0:
            raise ValueError("PI requires finite inputs and a positive simulation timestep")
        s = self.settings
        reference = np.clip(reference, -self.limit, self.limit)
        # An all-zero user command clears learned corrections and requests a stop.
        if not enabled or np.max(np.abs(reference)) <= s.zero_threshold:
            self.reset(reference, measurement)
            return reference.copy()
        if self.filtered is None:
            self.reset(reference, measurement)
        alpha = 1.0 if s.filter_tau == 0 else -np.expm1(-dt / s.filter_tau)
        self.filtered += alpha * (measurement - self.filtered)
        reversal = reference * self.previous_reference < 0
        became_zero = (np.abs(reference) <= s.zero_threshold) & (np.abs(self.previous_reference) > s.zero_threshold)
        self.integral[reversal | became_zero] = 0.0
        error = reference - self.filtered
        candidate = np.clip(self.integral + s.ki * error * dt, -s.integral_limit, s.integral_limit)

        def apply(integral):
            correction = s.kp * error + integral
            target = reference + np.clip(correction, -s.correction_limit, s.correction_limit)
            target = np.clip(target, -self.limit, self.limit)
            output = self.previous_output + np.clip(target - self.previous_output, -s.slew_rate * dt, s.slew_rate * dt)
            return output, reference + correction

        output, raw = apply(candidate)
        # Conditional integration accounts for correction, command and slew limits.
        blocked = error * (raw - output) > 1e-12
        candidate[blocked] = self.integral[blocked]
        self.integral = candidate
        output, raw = apply(self.integral)
        self.saturated = np.abs(raw - output) > 1e-10
        self.previous_output = output.copy()
        self.previous_reference = reference.copy()
        self.active = True
        return output
