# OrcaRL2

```bash

https://osrf.github.io/ros2multirobotbook/intro.html

https://github.com/open-rmf/rmf_demos

https://open-rmf.github.io/rmf-panel-js/

https://github.com/open-rmf/rmf_demos/tree/foxy/rmf_demos_panel/task_lists

ros2 run rmf_demos_tasks dispatch_patrol -p coe lounge -n 3 --use_sim_time

ros2 run rmf_demos_tasks dispatch_delivery -p pantry -ph coke_dispenser -d hardware_2 -dh coke_ingestor --use_sim_time

ros2 run rmf_demos_tasks dispatch_loop -s coe -f lounge -n 3 --use_sim_time
ros2 run rmf_demos_tasks dispatch_delivery -p pantry -pd coke_dispenser -d hardware_2 -di coke_ingestor --use_sim_time




[
{"task_type":"Loop", "start_time":0, "priority":0, "description": {"num_loops":5, "start_name":"coe", "finish_name":"lounge"}},
{"task_type":"Delivery", "start_time":0, "priority":0, "description": {"option": "coke"}},
{"task_type":"Loop", "start_time":0, "priority":0, "description": {"num_loops":5, "start_name":"pantry", "finish_name":"supplies"}}
]


vboxuser@Ubuntu20:~$ ros2 node info /tinyRobot_fleet_adapter
/opt/ros/foxy/bin/ros2:6: DeprecationWarning: pkg_resources is deprecated as an API. See https://setuptools.pypa.io/en/latest/pkg_resources.html
  from pkg_resources import load_entry_point
/tinyRobot_fleet_adapter
  Subscribers:
    /clock: rosgraph_msgs/msg/Clock
    /dispenser_results: rmf_dispenser_msgs/msg/DispenserResult
    /dispenser_states: rmf_dispenser_msgs/msg/DispenserState
    /dock_summary: rmf_fleet_msgs/msg/DockSummary
    /door_states: rmf_door_msgs/msg/DoorState
    /door_supervisor_heartbeat: rmf_door_msgs/msg/SupervisorHeartbeat
    /fire_alarm_trigger: std_msgs/msg/Bool
    /fleet_states: rmf_fleet_msgs/msg/FleetState
    /ingestor_results: rmf_ingestor_msgs/msg/IngestorResult
    /ingestor_states: rmf_ingestor_msgs/msg/IngestorState
    /lane_closure_requests: rmf_fleet_msgs/msg/LaneRequest
    /lift_states: rmf_lift_msgs/msg/LiftState
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rmf_task/bid_notice: rmf_task_msgs/msg/BidNotice
    /rmf_task/dispatch_request: rmf_task_msgs/msg/DispatchRequest
    /rmf_traffic/blockade_heartbeat: rmf_traffic_msgs/msg/BlockadeHeartbeat
    /rmf_traffic/fail_over_event: rmf_traffic_msgs/msg/FailOverEvent
    /rmf_traffic/negotiation_conclusion: rmf_traffic_msgs/msg/NegotiationConclusion
    /rmf_traffic/negotiation_forfeit: rmf_traffic_msgs/msg/NegotiationForfeit
    /rmf_traffic/negotiation_notice: rmf_traffic_msgs/msg/NegotiationNotice
    /rmf_traffic/negotiation_proposal: rmf_traffic_msgs/msg/NegotiationProposal
    /rmf_traffic/negotiation_rejection: rmf_traffic_msgs/msg/NegotiationRejection
    /rmf_traffic/negotiation_repeat: rmf_traffic_msgs/msg/NegotiationRepeat
    /rmf_traffic/participants: rmf_traffic_msgs/msg/Participants
    /rmf_traffic/query_update_1: rmf_traffic_msgs/msg/MirrorUpdate
    /rmf_traffic/registered_queries: rmf_traffic_msgs/msg/ScheduleQueries
    /rmf_traffic/schedule_inconsistency: rmf_traffic_msgs/msg/ScheduleInconsistency
  Publishers:
    /adapter_door_requests: rmf_door_msgs/msg/DoorRequest
    /adapter_lift_requests: rmf_lift_msgs/msg/LiftRequest
    /closed_lanes: rmf_fleet_msgs/msg/ClosedLanes
    /dispenser_requests: rmf_dispenser_msgs/msg/DispenserRequest
    /fleet_states: rmf_fleet_msgs/msg/FleetState
    /ingestor_requests: rmf_ingestor_msgs/msg/IngestorRequest
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rmf_task/bid_proposal: rmf_task_msgs/msg/BidProposal
    /rmf_task/dispatch_ack: rmf_task_msgs/msg/DispatchAck
    /rmf_traffic/blockade_cancel: rmf_traffic_msgs/msg/BlockadeCancel
    /rmf_traffic/blockade_reached: rmf_traffic_msgs/msg/BlockadeReached
    /rmf_traffic/blockade_ready: rmf_traffic_msgs/msg/BlockadeReady
    /rmf_traffic/blockade_release: rmf_traffic_msgs/msg/BlockadeRelease
    /rmf_traffic/blockade_set: rmf_traffic_msgs/msg/BlockadeSet
    /rmf_traffic/itinerary_clear: rmf_traffic_msgs/msg/ItineraryClear
    /rmf_traffic/itinerary_delay: rmf_traffic_msgs/msg/ItineraryDelay
    /rmf_traffic/itinerary_erase: rmf_traffic_msgs/msg/ItineraryErase
    /rmf_traffic/itinerary_extend: rmf_traffic_msgs/msg/ItineraryExtend
    /rmf_traffic/itinerary_set: rmf_traffic_msgs/msg/ItinerarySet
    /rmf_traffic/negotiation_ack: rmf_traffic_msgs/msg/NegotiationAck
    /rmf_traffic/negotiation_forfeit: rmf_traffic_msgs/msg/NegotiationForfeit
    /rmf_traffic/negotiation_notice: rmf_traffic_msgs/msg/NegotiationNotice
    /rmf_traffic/negotiation_proposal: rmf_traffic_msgs/msg/NegotiationProposal
    /rmf_traffic/negotiation_refusal: rmf_traffic_msgs/msg/NegotiationRefusal
    /rmf_traffic/negotiation_rejection: rmf_traffic_msgs/msg/NegotiationRejection
    /rmf_traffic/negotiation_repeat: rmf_traffic_msgs/msg/NegotiationRepeat
    /robot_mode_requests: rmf_fleet_msgs/msg/ModeRequest
    /robot_path_requests: rmf_fleet_msgs/msg/PathRequest
    /rosout: rcl_interfaces/msg/Log
    /task_summaries: rmf_task_msgs/msg/TaskSummary
  Service Servers:
    /tinyRobot_fleet_adapter/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /tinyRobot_fleet_adapter/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /tinyRobot_fleet_adapter/get_parameters: rcl_interfaces/srv/GetParameters
    /tinyRobot_fleet_adapter/list_parameters: rcl_interfaces/srv/ListParameters
    /tinyRobot_fleet_adapter/set_parameters: rcl_interfaces/srv/SetParameters
    /tinyRobot_fleet_adapter/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:
    /rmf_traffic/register_participant: rmf_traffic_msgs/srv/RegisterParticipant
    /rmf_traffic/request_changes: rmf_traffic_msgs/srv/RequestChanges
    /rmf_traffic/unregister_participant: rmf_traffic_msgs/srv/UnregisterParticipant
  Action Servers:

  Action Clients:


```