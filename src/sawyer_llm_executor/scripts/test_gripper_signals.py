#!/usr/bin/env python3
"""
Test script to verify gripper software signals are working correctly.
This helps isolate whether the issue is software (signals not being sent)
or hardware (signals sent but gripper not responding).

Usage:
    source intera.sh
    python3 test_gripper_signals.py
"""

import rospy
import time
from intera_interface import get_current_gripper_interface

def test_gripper_signals():
    """Test gripper signal interface and verify signals are being sent."""
    
    rospy.init_node('test_gripper_signals', anonymous=True)
    
    print("\n" + "="*60)
    print("GRIPPER SIGNAL TEST")
    print("="*60 + "\n")
    
    try:
        # Get gripper interface
        print("[1/6] Connecting to gripper interface...")
        gripper = get_current_gripper_interface()
        print("✓ Gripper interface connected\n")
        
        # Check gripper type
        from intera_interface import SimpleClickSmartGripper
        is_stp = isinstance(gripper, SimpleClickSmartGripper)
        gripper_type = "Smart Tool Plate" if is_stp else "Electric Gripper"
        print(f"[2/6] Gripper type: {gripper_type}\n")
        
        if not is_stp:
            print("⚠ This script is designed for Smart Tool Plate grippers.")
            print("  Electric grippers use different signals.\n")
            return
        
        # Check if ready
        print("[3/6] Checking gripper readiness...")
        if not gripper.is_ready():
            print("⚠ Gripper not ready. Attempting initialization...")
            if hasattr(gripper, 'needs_init') and gripper.needs_init():
                gripper.initialize()
                time.sleep(1)
        print(f"✓ Gripper ready: {gripper.is_ready()}\n")
        
        # Get available signals
        print("[4/6] Listing available signals...")
        signals = gripper.get_ee_signals()
        print(f"✓ Available signals: {list(signals.keys())}\n")
        
        # Read initial state
        print("[5/6] Reading initial signal states...")
        print("-" * 40)
        initial_state = {}
        for signal_name in ['power', 'grip', 'open', 'closed']:
            if signal_name in signals:
                try:
                    value = gripper.get_ee_signal_value(signal_name)
                    initial_state[signal_name] = value
                    print(f"  {signal_name:10s}: {value}")
                except Exception as e:
                    print(f"  {signal_name:10s}: ERROR - {e}")
        print("-" * 40 + "\n")
        
        # Test signal toggling
        print("[6/6] Testing signal toggling...")
        print("\n--- Test 1: Enable Power ---")
        try:
            gripper.set_ee_signal_value("power", True)
            time.sleep(0.5)
            power_state = gripper.get_ee_signal_value("power")
            print(f"  Power signal set to: {power_state}")
            if power_state:
                print("  ✓ Power signal enabled successfully")
            else:
                print("  ✗ Power signal failed to enable")
        except Exception as e:
            print(f"  ✗ Error setting power: {e}")
        
        print("\n--- Test 2: Read Current State ---")
        try:
            is_open = gripper.get_ee_signal_value("open") if "open" in signals else None
            is_closed = gripper.get_ee_signal_value("closed") if "closed" in signals else None
            current_grip = gripper.get_ee_signal_value("grip")
            print(f"  Current state:")
            print(f"    open sensor:  {is_open}")
            print(f"    closed sensor: {is_closed}")
            print(f"    grip signal:  {current_grip}")
        except Exception as e:
            print(f"  ✗ Error reading state: {e}")
        
        print("\n--- Test 3: Toggle Grip Signal (Close) ---")
        try:
            # Set grip to True (close)
            print("  Setting grip signal to True (close)...")
            gripper.set_ee_signal_value("grip", True)
            time.sleep(0.2)
            grip_after = gripper.get_ee_signal_value("grip")
            print(f"  Grip signal after set: {grip_after}")
            
            # Wait and check sensors
            print("  Waiting 2 seconds for actuation...")
            time.sleep(2)
            
            is_open_after = gripper.get_ee_signal_value("open") if "open" in signals else None
            is_closed_after = gripper.get_ee_signal_value("closed") if "closed" in signals else None
            print(f"  State after close command:")
            print(f"    open sensor:  {is_open_after} (was {initial_state.get('open', 'N/A')})")
            print(f"    closed sensor: {is_closed_after} (was {initial_state.get('closed', 'N/A')})")
            
            if is_closed_after != initial_state.get('closed'):
                print("  ✓ Closed sensor changed - gripper may have moved")
            else:
                print("  ⚠ Closed sensor unchanged - gripper may not have moved")
        except Exception as e:
            print(f"  ✗ Error during close test: {e}")
        
        print("\n--- Test 4: Toggle Grip Signal (Open) ---")
        try:
            # Set grip to False (open)
            print("  Setting grip signal to False (open)...")
            gripper.set_ee_signal_value("grip", False)
            time.sleep(0.2)
            grip_after = gripper.get_ee_signal_value("grip")
            print(f"  Grip signal after set: {grip_after}")
            
            # Wait and check sensors
            print("  Waiting 2 seconds for actuation...")
            time.sleep(2)
            
            is_open_after = gripper.get_ee_signal_value("open") if "open" in signals else None
            is_closed_after = gripper.get_ee_signal_value("closed") if "closed" in signals else None
            print(f"  State after open command:")
            print(f"    open sensor:  {is_open_after}")
            print(f"    closed sensor: {is_closed_after}")
            
            if is_open_after != initial_state.get('open'):
                print("  ✓ Open sensor changed - gripper may have moved")
            else:
                print("  ⚠ Open sensor unchanged - gripper may not have moved")
        except Exception as e:
            print(f"  ✗ Error during open test: {e}")
        
        # Final state summary
        print("\n" + "="*60)
        print("FINAL STATE SUMMARY")
        print("="*60)
        try:
            final_power = gripper.get_ee_signal_value("power")
            final_grip = gripper.get_ee_signal_value("grip")
            final_open = gripper.get_ee_signal_value("open") if "open" in signals else None
            final_closed = gripper.get_ee_signal_value("closed") if "closed" in signals else None
            
            print(f"Power:  {final_power}")
            print(f"Grip:   {final_grip}")
            print(f"Open:   {final_open}")
            print(f"Closed: {final_closed}")
            
            print("\n" + "-"*60)
            print("INTERPRETATION:")
            print("-"*60)
            if final_power:
                print("✓ Power signal is enabled")
            else:
                print("✗ Power signal is disabled - this may prevent actuation")
            
            if final_grip == initial_state.get('grip'):
                print("⚠ Grip signal returned to initial value")
            else:
                print(f"✓ Grip signal changed from {initial_state.get('grip')} to {final_grip}")
            
            if final_open == initial_state.get('open') and final_closed == initial_state.get('closed'):
                print("⚠ Sensors did not change - gripper likely did NOT move physically")
                print("  → This suggests a HARDWARE issue (air, valve, mechanical)")
            else:
                print("✓ Sensors changed - gripper may have moved")
                print("  → If you don't see physical movement, check sensor alignment")
            
        except Exception as e:
            print(f"Error reading final state: {e}")
        
        print("\n" + "="*60)
        print("Test complete!")
        print("="*60 + "\n")
        
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        print("\nMake sure:")
        print("  1. Robot is enabled (rosrun intera_interface enable_robot.py -e)")
        print("  2. Gripper is properly attached to the robot")
        print("  3. You have sourced intera.sh")

if __name__ == '__main__':
    try:
        test_gripper_signals()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
    except rospy.ROSInterruptException:
        pass
