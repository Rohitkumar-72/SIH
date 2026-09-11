"""Velocity profile definitions, safety tiers, and throughput break-even modeling.

Physical AMRs have strict kinematic envelopes. This module formalizes
validated profiles from physical fidelity (capped at 0.46 m/s hardware limit)
up to explicit failure-envelope experiments.
"""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple


@dataclass(frozen=True)
class VelocityProfile:
    name: str
    nominal_speed_mps: float
    max_speed_mps: float
    tier: str
    certified: bool
    status: str
    description: str

    def displacement_per_step(self, max_step_size_s: float = 0.02) -> float:
        """Displacement in metres per physics step."""
        return self.nominal_speed_mps * max_step_size_s

    def break_even_rtf(self, baseline_velocity: float = 4.0, baseline_rtf: float = 1.726) -> float:
        """Calculate RTF required to achieve nominal motion throughput of baseline.
        
        Formula:
            required_new_rtf = baseline_rtf * baseline_velocity / proposed_velocity
        """
        if self.nominal_speed_mps <= 0:
            return float("inf")
        return baseline_rtf * baseline_velocity / self.nominal_speed_mps


# Formal Fleet Velocity Profiles
PROFILES: Dict[str, VelocityProfile] = {
    "physical_fidelity": VelocityProfile(
        name="physical_fidelity",
        nominal_speed_mps=0.31,
        max_speed_mps=0.46,
        tier="PHYSICAL_FIDELITY",
        certified=True,
        status="CERTIFIED_PHYSICAL_FIDELITY",
        description="Physical TurtleBot 4 fidelity profile; capped at 0.46 m/s hardware limit, initial test point at 0.31 m/s."
    ),
    "physical_max": VelocityProfile(
        name="physical_max",
        nominal_speed_mps=0.46,
        max_speed_mps=0.46,
        tier="PHYSICAL_FIDELITY",
        certified=True,
        status="CERTIFIED_PHYSICAL_LIMIT",
        description="Hardware ceiling (0.46 m/s) of physical TurtleBot 4 differential drivetrain."
    ),
    "synthetic_0_75": VelocityProfile(
        name="synthetic_0_75",
        nominal_speed_mps=0.75,
        max_speed_mps=0.75,
        tier="SYNTHETIC_UNCERTIFIED",
        certified=False,
        status="UNCERTIFIED_AWAITING_POSE_INTEGRITY",
        description="Synthetic accelerated profile (0.75 m/s); uncertified until user returns logs satisfying all pose-integrity criteria."
    ),
    "synthetic_1_00": VelocityProfile(
        name="synthetic_1_00",
        nominal_speed_mps=1.00,
        max_speed_mps=1.00,
        tier="SYNTHETIC_UNCERTIFIED",
        certified=False,
        status="UNCERTIFIED_AWAITING_POSE_INTEGRITY",
        description="Synthetic accelerated profile (1.00 m/s); uncertified until user returns logs satisfying all pose-integrity criteria."
    ),
    "failure_envelope_2_00": VelocityProfile(
        name="failure_envelope_2_00",
        nominal_speed_mps=2.00,
        max_speed_mps=2.00,
        tier="FAILURE_ENVELOPE_EXPERIMENT",
        certified=False,
        status="EXPERIMENTAL_FAILURE_ENVELOPE",
        description="Explicit failure-envelope experiment (2.00 m/s) to evaluate contact dynamics degradation and slipping."
    ),
    "failure_envelope_4_00": VelocityProfile(
        name="failure_envelope_4_00",
        nominal_speed_mps=4.00,
        max_speed_mps=4.00,
        tier="FAILURE_ENVELOPE_EXPERIMENT",
        certified=False,
        status="EXPERIMENTAL_FAILURE_ENVELOPE",
        description="Legacy 4.00 m/s failure-envelope experiment; baseline produced zero physically valid task completions."
    ),
}

# Default profile for validated operations
DEFAULT_PROFILE_NAME = "physical_fidelity"


def get_profile(name_or_speed: str) -> Optional[VelocityProfile]:
    """Look up a profile by name or by numeric speed string."""
    key = name_or_speed.strip().lower()
    if key in PROFILES:
        return PROFILES[key]

    # Try matching by numeric speed
    try:
        val = float(key)
        for p in PROFILES.values():
            if abs(p.nominal_speed_mps - val) < 1e-3:
                return p
    except ValueError:
        pass
    return None


def resolve_velocity_profile(name_or_speed: str) -> VelocityProfile:
    """Resolve a profile, or construct an ad-hoc uncertified profile with warnings."""
    profile = get_profile(name_or_speed)
    if profile is not None:
        return profile

    # Parse as arbitrary float speed
    try:
        spd = float(name_or_speed)
    except ValueError:
        raise ValueError(
            f"Unknown velocity profile '{name_or_speed}'. "
            f"Valid profiles: {list(PROFILES.keys())} or a numeric float."
        )

    if spd <= 0.46:
        tier = "PHYSICAL_FIDELITY"
        certified = True
        status = "CUSTOM_PHYSICAL_FIDELITY"
        desc = f"Custom speed ({spd:.2f} m/s) within physical hardware limit (0.46 m/s)."
    elif spd <= 1.0:
        tier = "SYNTHETIC_UNCERTIFIED"
        certified = False
        status = "UNCERTIFIED_AWAITING_POSE_INTEGRITY"
        desc = f"Custom synthetic speed ({spd:.2f} m/s); uncertified until user returns pose-integrity logs."
    else:
        tier = "FAILURE_ENVELOPE_EXPERIMENT"
        certified = False
        status = "EXPERIMENTAL_FAILURE_ENVELOPE"
        desc = f"Custom excessive speed ({spd:.2f} m/s); treated as failure-envelope experiment."

    return VelocityProfile(
        name=f"custom_{spd:.2f}mps",
        nominal_speed_mps=spd,
        max_speed_mps=spd,
        tier=tier,
        certified=certified,
        status=status,
        description=desc
    )


def compute_throughput_break_even(
    proposed_speed_mps: float,
    current_speed_mps: float = 4.0,
    current_rtf: float = 1.726
) -> float:
    """Calculate required RTF to break even on nominal motion throughput."""
    if proposed_speed_mps <= 0:
        return float("inf")
    return current_rtf * current_speed_mps / proposed_speed_mps
