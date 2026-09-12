# Mission safety state machine

The mission manager starts in `BOOT`, waits for a stable MAVROS state link and
the expected FCU identity in `WAIT_FCU`, and then enters `STANDBY`. Startup
never grants mission authority. GPS, position, EKF, armed state, and AUTO mode
are checked by the attack-start gate when a new target arrives; they do not
prevent the manager from waiting in `STANDBY`.

Before real-hardware use, verify that the installed MAVROS publishes:

```text
/mavros/state
/mavros/global_position/global
/mavros/gpsstatus/gps1/raw
/mavros/estimator_status
/mavros/sys_status
```

and provides:

```text
/mavros/vehicle_info_get
```

Topic names or plugin availability can differ between MAVROS installations.
Adapt the subscriptions to the installed interfaces; do not disable required
checks merely to make the attack-start gate pass.

## Mission authorization

A valid target received in `STANDBY` is rejected unless the FCU is connected
and correctly identified, the aircraft is armed and in AUTO, required
navigation data is fresh and healthy, no serious system status is present, and
`/fcu/goto_global` is available. A rejected target is discarded. One manager
process can authorize at most one attack.

```bash
# Observe state and diagnostics.
ros2 topic echo /mission/safety_state
ros2 topic echo /mission/safety_status

# Send a new target after the ground operator has armed and selected AUTO.
ros2 topic pub --once /vision/target_command uav_interfaces/msg/TargetCommand \
  "{latitude: 22.88480535, longitude: 113.49559465, heading_deg: 90.0}"
```

Disable mission authority with:

```bash
ros2 service call /mission/disable std_srvs/srv/Trigger "{}"
```

`TargetCommand` currently has no source timestamp, authenticated operator, or
external target ID. The manager therefore generates a boot-local ID and receive
timestamp. Extend the interface before accepting targets from an untrusted
network.

Entering `SAFE` blocks new mission-manager outputs and invalidates the current
target, but the current FCU interface has no cancellable action or dedicated
composite-mission abort service. A mission already uploaded to the FCU can keep
executing autonomously. Diagnostics explicitly report that no FCU abort was
executed. An operator-approved abort adapter must be implemented and validated
before SAFE can claim active FCU mission cancellation.
