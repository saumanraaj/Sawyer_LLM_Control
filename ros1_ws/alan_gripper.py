#!/usr/bin/env python3
import argparse
import rospy
from intera_core_msgs.msg import IOComponentCommand

DEFAULT_TOPIC = "/io/end_effector/command"

# From your config:
# grip_HylfwjncK7 sink -> A_out2, and A_out2 bit_index = 1, command gpo_write
DEFAULT_PORT = "A_out2"
DEFAULT_BIT  = 1

def pub_cmd(pub, op: str, args: str):
    msg = IOComponentCommand()
    msg.time = rospy.Time.now()
    msg.op = op
    msg.args = args
    pub.publish(msg)
    rospy.loginfo("Published: op=%s args=%s", op, args)

def try_close_open(pub, close: bool, style: str, port: str, bit: int):
    v = 1 if close else 0

    if style == "port_eq":
        # Example: "A_out2=1"
        pub_cmd(pub, "gpo_write", f"{port}={v}")
    elif style == "kv":
        # Example: "port=A_out2 value=1"
        pub_cmd(pub, "gpo_write", f"port={port} value={v}")
    elif style == "bit":
        # Example: "bit=1 value=1"
        pub_cmd(pub, "gpo_write", f"bit={bit} value={v}")
    elif style == "json":
        # Example JSON-ish payload (some stacks accept this)
        pub_cmd(pub, "gpo_write", f'{{"port":"{port}","bit":{bit},"value":{v}}}')
    else:
        raise ValueError("Unknown style")

def main():
    p = argparse.ArgumentParser(description="Test Sawyer PSG pneumatic gripper via IOComponentCommand (op/args).")
    p.add_argument("--topic", default=DEFAULT_TOPIC)
    p.add_argument("--port", default=DEFAULT_PORT, help="GPIO output port name (from config). Default A_out2")
    p.add_argument("--bit", type=int, default=DEFAULT_BIT, help="GPIO bit index (from config). Default 1")
    p.add_argument("--style", default="port_eq", choices=["port_eq", "kv", "bit", "json"],
                   help="Args formatting style to try.")
    p.add_argument("--open", action="store_true")
    p.add_argument("--close", action="store_true")
    p.add_argument("--pulse", action="store_true")
    p.add_argument("--duration", type=float, default=0.4)
    args = p.parse_args(rospy.myargv()[1:])

    rospy.init_node("alan_gripper_psg", anonymous=True)
    pub = rospy.Publisher(args.topic, IOComponentCommand, queue_size=5)
    rospy.sleep(0.5)

    if args.open and args.close:
        rospy.logerr("Pick only one of --open or --close (or use --pulse).")
        return

    if not (args.open or args.close or args.pulse):
        args.pulse = True

    if args.close:
        try_close_open(pub, True, args.style, args.port, args.bit)
        return

    if args.open:
        try_close_open(pub, False, args.style, args.port, args.bit)
        return

    # pulse
    try_close_open(pub, True, args.style, args.port, args.bit)
    rospy.sleep(args.duration)
    try_close_open(pub, False, args.style, args.port, args.bit)

if __name__ == "__main__":
    main()

