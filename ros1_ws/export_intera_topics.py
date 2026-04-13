#!/usr/bin/env python3
import argparse
import rospy
from datetime import datetime

DEFAULT_KEYWORDS = [
    "io", "end_effector", "grip", "gripper", "psg", "pneu", "pneumatic", "solenoid", "valve", "vac"
]

def main():
    parser = argparse.ArgumentParser(
        description="Export ROS published topics (optionally filtered by keywords) to a text file."
    )
    parser.add_argument(
        "-o", "--out", type=str, default=None,
        help="Output filename. Default: topics_<timestamp>.txt"
    )
    parser.add_argument(
        "-k", "--keywords", nargs="*", default=DEFAULT_KEYWORDS,
        help="Keywords to filter topics (case-insensitive). Default: common IO/gripper-related keywords."
    )
    parser.add_argument(
        "--all", action="store_true",
        help="Export ALL topics (ignores --keywords). Warning: can be huge."
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("export_intera_topics", anonymous=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_file = args.out or f"topics_{timestamp}.txt"

    topics = rospy.get_published_topics()  # list of (topic_name, topic_type)

    def keep(topic_name: str) -> bool:
        if args.all:
            return True
        low = topic_name.lower()
        return any(kw.lower() in low for kw in args.keywords)

    kept = [(t, ty) for (t, ty) in topics if keep(t)]

    kept.sort(key=lambda x: x[0])

    with open(out_file, "w", encoding="utf-8") as f:
        f.write(f"# Exported ROS published topics\n")
        f.write(f"# Time: {datetime.now().isoformat()}\n")
        f.write(f"# ROS_MASTER_URI: {rospy.get_param('/roslaunch/uris', 'unknown')}\n")
        f.write(f"# Total topics: {len(topics)}\n")
        f.write(f"# Exported topics: {len(kept)}\n")
        if not args.all:
            f.write(f"# Filter keywords: {args.keywords}\n")
        f.write("\n")

        for t, ty in kept:
            f.write(f"{t}\t[{ty}]\n")

    print(f"Wrote {len(kept)} topics (out of {len(topics)}) to: {out_file}")

if __name__ == "__main__":
    main()

