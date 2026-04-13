#!/usr/bin/env python3
"""
Quick gripper signal test - minimal version for rapid testing.
"""

import rospy
import time
from intera_interface import get_current_gripper_interface

rospy.init_node('quick_gripper_test')
gripper = get_current_gripper_interface()

print("\n=== Quick Gripper Signal Test ===\n")

# Get signals
signals = gripper.get_ee_signals()
print(f"Available signals: {list(signals.keys())}\n")

# Read current state
print("Current state:")
for sig in ['power', 'grip', 'open', 'closed']:
    if sig in signals:
        val = gripper.get_ee_signal_value(sig)
        print(f"  {sig}: {val}")

# Enable power
print("\nEnabling power...")
gripper.set_ee_signal_value("power", True)
time.sleep(0.5)
print(f"Power enabled: {gripper.get_ee_signal_value('power')}")

# Test close
print("\nTesting CLOSE (grip=True)...")
gripper.set_ee_signal_value("grip", True)
time.sleep(2)
print(f"  Grip signal: {gripper.get_ee_signal_value('grip')}")
print(f"  Open sensor: {gripper.get_ee_signal_value('open')}")
print(f"  Closed sensor: {gripper.get_ee_signal_value('closed')}")

# Test open
print("\nTesting OPEN (grip=False)...")
gripper.set_ee_signal_value("grip", False)
time.sleep(2)
print(f"  Grip signal: {gripper.get_ee_signal_value('grip')}")
print(f"  Open sensor: {gripper.get_ee_signal_value('open')}")
print(f"  Closed sensor: {gripper.get_ee_signal_value('closed')}")

print("\n=== Test Complete ===\n")
