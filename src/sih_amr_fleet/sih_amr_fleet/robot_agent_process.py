#!/usr/bin/env python3
"""Consolidated Robot Agent Process for SIH Decentralized Multi-AMR Fleet.

Bundles all local coordination, navigation, safety, and consensus nodes for one
AMR into a single multi-node ROS 2 process, drastically eliminating Python
interpreter and thread context-switching overhead on multi-robot hosts.
"""

import sys
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor

from .blockage_detector_node import BlockageDetectorNode
from .cbba_node import CbbaNode
from .charging_pad_node import ChargingPadNode
from .corridor_mutex_node import CorridorMutexNode
from .docking_coordinator_node import DockingCoordinatorNode
from .health_node import HealthNode
from .local_costmap_node import LocalCostmapNode
from .orca_node import OrcaNode
from .path_follower_node import PathFollowerNode
from .peer_tracker_node import PeerTrackerNode
from .reservation_manager_node import ReservationManagerNode
from .safety_supervisor_node import SafetySupervisorNode
from .task_execution_node import TaskExecutionNode
from .whca_planner_node import WhcaPlannerNode


def main(args=None):
    rclpy.init(args=args)
    
    # 2 worker threads in the executor allow non-blocking path planning alongside control callbacks
    executor = MultiThreadedExecutor(num_threads=2)
    nodes = []

    try:
        nodes = [
            LocalCostmapNode(),
            BlockageDetectorNode(),
            PeerTrackerNode(),
            HealthNode(),
            ChargingPadNode(),
            DockingCoordinatorNode(),
            CbbaNode(),
            WhcaPlannerNode(),
            ReservationManagerNode(),
            CorridorMutexNode(),
            PathFollowerNode(),
            OrcaNode(),
            SafetySupervisorNode(),
            TaskExecutionNode(),
        ]
        for node in nodes:
            executor.add_node(node)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for node in nodes:
            try:
                executor.remove_node(node)
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
