"""Common launch-parameter defaults shared by REAL and SITL bringup.

Edit common mission/FCU values here when REAL and SITL should change together.
Profile-specific values must stay in the corresponding launch file.
"""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


# ============================================================================
# Common launch arguments: MAVROS / ROS runtime
# ============================================================================
COMMON_RUNTIME_ARGUMENT_DEFAULTS = {
    'gcs_url': '',
    'target_system_id': '1',
    'target_component_id': '1',
    'use_sim_time': 'false',
}


# ============================================================================
# Common FCU/composite-mission parameters
#
# These values were explicitly configured to the same values in both the
# current REAL and SITL launch files. Keeping them here makes this file the
# single default-value source for the shared behavior.
# ============================================================================
COMMON_FCU_ARGUMENT_DEFAULTS = {
    # Payload command inserted into the composite mission.
    'servo_channel': '7',
    'safe_pwm': '1350',
    'release_pwm': '1900',

    # Mission service transaction control used by both profiles.
    'service_timeout_sec': '8.0',

    # Original mission splice boundaries.
    'insert_wp_index': '5',
    'resume_wp_index': '10',

    # A / B / R / C / D geometry.
    'a_offset_m': '160.0',
    'b_offset_m': '95.0',
    'release_offset_m': '50.0',
    'd_offset_m': '50.0',

    # Waypoint acceptance radii.
    'a_acceptance_radius_m': '30.0',
    'b_acceptance_radius_m': '15.0',
    'c_acceptance_radius_m': '8.0',
    'd_acceptance_radius_m': '30.0',

    # Observation-only A-B trajectory check values present in both profiles.
    'b_check_half_width_m': '17.0',
    'b_check_length_m': '60.0',
    'b_check_required_samples': '3',
    'gps_stale_timeout_sec': '1.0',

    # Composite target-segment altitude.
    'mission_altitude_m': '15.0',
}


# ============================================================================
# Common MissionManager parameters
# ============================================================================
COMMON_MISSION_MANAGER_ARGUMENT_DEFAULTS = {
    'acceptance_radius_m': '8.0',
    # MissionManager uses the same insertion boundary as FCUInterface to decide
    # whether a newly received target is still allowed to trigger ABRCD.
    'reject_target_at_or_after_insert_wp': 'true',
}


COMMON_ARGUMENT_DEFAULTS = {
    **COMMON_RUNTIME_ARGUMENT_DEFAULTS,
    **COMMON_FCU_ARGUMENT_DEFAULTS,
    **COMMON_MISSION_MANAGER_ARGUMENT_DEFAULTS,
}


def declare_common_arguments():
    """Declare every shared parameter once for either launch profile."""
    return [
        DeclareLaunchArgument(name, default_value=default_value)
        for name, default_value in COMMON_ARGUMENT_DEFAULTS.items()
    ]


def common_launch_configurations():
    """Return LaunchConfiguration objects keyed by shared parameter name."""
    return {
        name: LaunchConfiguration(name)
        for name in COMMON_ARGUMENT_DEFAULTS
    }
