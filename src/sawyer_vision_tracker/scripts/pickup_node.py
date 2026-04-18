#!/usr/bin/env python3
"""
ROS node: subscribes to the vision tracker's target pose and executes
a pick-up sequence on the Sawyer robot using MoveIt.

States: IDLE -> WAITING -> APPROACHING -> LOWERING -> GRASPING -> LIFTING -> DONE
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from sawyer_vision_tracker.sawyer_actions import sawyer_actions
from sawyer_vision_tracker.utils import euclidean_distance

# state names
IDLE = "IDLE"
WAITING = "WAITING"
APPROACHING = "APPROACHING"
LOWERING = "LOWERING"
GRASPING = "GRASPING"
LIFTING = "LIFTING"
DONE = "DONE"


class PickupNode:

    def __init__(self):
        rospy.init_node("pickup_node")

        # params
        pickup_cfg = rospy.get_param("pickup", {})
        self.approach_z = pickup_cfg.get("approach_height", 0.25)
        self.grasp_z = pickup_cfg.get("grasp_height", 0.05)
        self.lift_dz = pickup_cfg.get("lift_height", 0.15)
        self.settle_time = pickup_cfg.get("settle_time", 1.0)
        self.min_stable = pickup_cfg.get("min_stable_frames", 10)
        self.auto_pickup = rospy.get_param("~auto_pickup", False)

        # state
        self.state = IDLE
        self.target_pose = None
        self.target_valid = False
        self.stable_count = 0
        self.last_target_xy = None
        self.locked_target = None  # pose locked at start of pickup

        # robot
        rospy.loginfo("[Pickup] Initializing sawyer_actions...")
        self.robot = sawyer_actions()

        # publishers
        self.state_pub = rospy.Publisher("/pickup/state", String, queue_size=1)

        # subscribers
        rospy.Subscriber(
            "/vision_tracker/target_pose", PoseStamped, self.pose_callback
        )
        rospy.Subscriber(
            "/vision_tracker/target_valid", Bool, self.valid_callback
        )

        # service
        rospy.Service("/pickup/execute", Trigger, self.execute_callback)

        rospy.loginfo("[Pickup] Ready. Call /pickup/execute to start.")

    def pose_callback(self, msg):
        self.target_pose = msg

    def valid_callback(self, msg):
        self.target_valid = msg.data

    def execute_callback(self, req):
        """Service handler: trigger a pickup attempt."""
        if self.state != IDLE:
            return TriggerResponse(
                success=False,
                message=f"Busy, current state: {self.state}",
            )
        if not self.target_valid or self.target_pose is None:
            return TriggerResponse(
                success=False, message="No valid target detected"
            )
        self.state = WAITING
        self.stable_count = 0
        self.last_target_xy = None
        return TriggerResponse(success=True, message="Pickup started")

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.state_pub.publish(String(data=self.state))

            if self.state == IDLE:
                self._handle_idle()
            elif self.state == WAITING:
                self._handle_waiting()
            elif self.state == APPROACHING:
                self._handle_approaching()
            elif self.state == LOWERING:
                self._handle_lowering()
            elif self.state == GRASPING:
                self._handle_grasping()
            elif self.state == LIFTING:
                self._handle_lifting()
            elif self.state == DONE:
                self._handle_done()

            rate.sleep()

    def _handle_idle(self):
        if self.auto_pickup and self.target_valid and self.target_pose is not None:
            rospy.loginfo("[Pickup] Auto-trigger: target detected")
            self.state = WAITING
            self.stable_count = 0
            self.last_target_xy = None

    def _handle_waiting(self):
        """Wait for the target to be stable for min_stable consecutive readings."""
        if not self.target_valid or self.target_pose is None:
            self.stable_count = 0
            self.last_target_xy = None
            return

        p = self.target_pose.pose.position
        current_xy = (p.x, p.y)

        if self.last_target_xy is not None:
            dist = euclidean_distance(current_xy, self.last_target_xy)
            if dist < 0.02:  # within 2cm tolerance
                self.stable_count += 1
            else:
                self.stable_count = 0
        else:
            self.stable_count = 1

        self.last_target_xy = current_xy

        if self.stable_count >= self.min_stable:
            # lock the target position
            self.locked_target = (p.x, p.y, p.z)
            rospy.loginfo(
                f"[Pickup] Target locked at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
            )
            self.state = APPROACHING

    def _handle_approaching(self):
        tx, ty, _ = self.locked_target
        rospy.loginfo("[Pickup] Opening gripper...")
        self.robot.open_gripper()

        rospy.loginfo(
            f"[Pickup] Approaching ({tx:.3f}, {ty:.3f}, z={self.approach_z:.3f})"
        )
        ok = self.robot.move_to(tx, ty, z=self.approach_z)
        if not ok:
            rospy.logwarn("[Pickup] Approach failed, retrying once...")
            ok = self.robot.move_to(tx, ty, z=self.approach_z)
        if ok:
            self.state = LOWERING
        else:
            rospy.logerr("[Pickup] Approach failed twice, aborting")
            self.state = IDLE

    def _handle_lowering(self):
        # re-read latest pose for correction if still valid
        if self.target_valid and self.target_pose is not None:
            p = self.target_pose.pose.position
            tx, ty = p.x, p.y
            rospy.loginfo(f"[Pickup] Updated target: ({tx:.3f}, {ty:.3f})")
        else:
            tx, ty, _ = self.locked_target

        rospy.loginfo(
            f"[Pickup] Lowering to grasp height z={self.grasp_z:.3f}"
        )
        ok = self.robot.move_to(tx, ty, z=self.grasp_z)
        if ok:
            self.state = GRASPING
        else:
            rospy.logerr("[Pickup] Lower failed, aborting")
            self.state = IDLE

    def _handle_grasping(self):
        rospy.loginfo("[Pickup] Closing gripper...")
        self.robot.close_gripper()
        rospy.loginfo(f"[Pickup] Settling for {self.settle_time:.1f}s...")
        rospy.sleep(self.settle_time)
        self.state = LIFTING

    def _handle_lifting(self):
        rospy.loginfo(f"[Pickup] Lifting by {self.lift_dz:.3f}m")
        ok = self.robot.lift(delta_z=self.lift_dz)
        if ok:
            self.state = DONE
        else:
            rospy.logerr("[Pickup] Lift failed")
            self.state = IDLE

    def _handle_done(self):
        rospy.loginfo("[Pickup] Pickup complete!")
        self.state = IDLE


if __name__ == "__main__":
    node = PickupNode()
    node.run()
