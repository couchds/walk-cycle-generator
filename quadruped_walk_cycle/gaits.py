import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Gait:
    """Footfall timing and body-motion metadata for a quadruped gait."""

    label: str
    phases: dict
    duty_factor: float
    body_bob_frequency: float
    body_sway_frequency: float
    body_pitch_frequency: float
    body_roll_frequency: float
    body_bob_scale: float = 1.0
    body_sway_scale: float = 1.0
    body_pitch_scale: float = 1.0
    body_roll_scale: float = 1.0


GAITS = {
    "COMPACT_WALK": Gait(
        label="Compact Walk",
        phases={"rl": 0.00, "fl": 0.75, "rr": 0.50, "fr": 0.25},
        duty_factor=0.82,
        body_bob_frequency=2.0,
        body_sway_frequency=1.0,
        body_pitch_frequency=1.0,
        body_roll_frequency=1.0,
        body_bob_scale=0.55,
        body_sway_scale=0.42,
        body_pitch_scale=0.50,
        body_roll_scale=0.38,
    ),
    "WALK": Gait(
        label="Walk",
        phases={"fl": 0.00, "rr": 0.25, "fr": 0.50, "rl": 0.75},
        duty_factor=0.78,
        body_bob_frequency=2.0,
        body_sway_frequency=1.0,
        body_pitch_frequency=1.0,
        body_roll_frequency=1.0,
        body_bob_scale=0.82,
        body_sway_scale=0.72,
        body_pitch_scale=0.72,
        body_roll_scale=0.65,
    ),
    "TROT": Gait(
        label="Trot",
        phases={"fl": 0.00, "rr": 0.00, "fr": 0.50, "rl": 0.50},
        duty_factor=0.58,
        body_bob_frequency=2.0,
        body_sway_frequency=1.0,
        body_pitch_frequency=2.0,
        body_roll_frequency=1.0,
        body_bob_scale=0.90,
        body_sway_scale=0.35,
        body_pitch_scale=0.70,
        body_roll_scale=0.25,
    ),
    "PACE": Gait(
        label="Pace",
        phases={"fl": 0.00, "rl": 0.00, "fr": 0.50, "rr": 0.50},
        duty_factor=0.60,
        body_bob_frequency=2.0,
        body_sway_frequency=1.0,
        body_pitch_frequency=1.0,
        body_roll_frequency=1.0,
        body_bob_scale=0.95,
        body_sway_scale=0.75,
        body_pitch_scale=0.65,
        body_roll_scale=0.72,
    ),
    "BOUND": Gait(
        label="Bound",
        phases={"rl": 0.00, "rr": 0.00, "fl": 0.50, "fr": 0.50},
        duty_factor=0.52,
        body_bob_frequency=1.0,
        body_sway_frequency=1.0,
        body_pitch_frequency=1.0,
        body_roll_frequency=1.0,
        body_bob_scale=1.05,
        body_sway_scale=0.0,
        body_pitch_scale=0.85,
        body_roll_scale=0.0,
    ),
}


def smoothstep(value):
    """Return a clamped smooth interpolation value from 0 to 1."""
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def lerp(start, end, amount):
    """Linearly interpolate between two scalar values."""
    return start + (end - start) * amount


def cycle_position(frame, start, end):
    """Return the normalized looping position of a frame in a cycle."""
    duration = max(1.0, float(end - start))
    return ((float(frame) - float(start)) / duration) % 1.0


def leg_motion(cycle_pos, phase, duty_factor, stride_length, step_height):
    """Compute forward offset, lift, and swing state for one leg."""
    foot_phase = (cycle_pos + phase) % 1.0
    swing_fraction = max(0.05, 1.0 - duty_factor)

    if foot_phase < swing_fraction:
        swing_t = smoothstep(foot_phase / swing_fraction)
        forward = lerp(-stride_length * 0.5, stride_length * 0.5, swing_t)
        lift = math.sin(math.pi * swing_t) * step_height
    else:
        stance_t = (foot_phase - swing_fraction) / max(0.05, duty_factor)
        forward = lerp(stride_length * 0.5, -stride_length * 0.5, stance_t)
        lift = 0.0

    return forward, lift, foot_phase < swing_fraction
