#!/usr/bin/env python3
"""Extract camera frames from a ROS1 bag into ordinary image files."""

import argparse
import os
import sys


def parse_args():
    parser = argparse.ArgumentParser(
        description="Extract /camera/image_raw frames from a ROS1 bag."
    )
    parser.add_argument("--bag", required=True, help="Path to the .bag file")
    parser.add_argument(
        "--out", required=True, help="Directory where extracted images are written"
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.4,
        help="Minimum seconds between saved frames (default: 0.4)",
    )
    parser.add_argument(
        "--topic", default="/camera/image_raw", help="Image topic in the bag"
    )
    parser.add_argument(
        "--jpg-quality",
        type=int,
        default=95,
        help="JPEG quality from 0 to 100 (default: 95)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.interval < 0:
        raise SystemExit("--interval must be >= 0")
    if not 0 <= args.jpg_quality <= 100:
        raise SystemExit("--jpg-quality must be between 0 and 100")
    if not os.path.isfile(args.bag):
        raise SystemExit("Bag file does not exist: {}".format(args.bag))

    # These imports are intentionally delayed so --help works without a ROS shell.
    try:
        import cv2
        import rosbag
        from cv_bridge import CvBridge
    except ImportError as exc:
        raise SystemExit(
            "ROS Python dependencies are unavailable. Source /opt/ros/noetic/setup.bash "
            "before running this script: {}".format(exc)
        )

    os.makedirs(args.out, exist_ok=True)
    bridge = CvBridge()
    next_save_time = None
    saved = 0
    seen = 0

    try:
        bag = rosbag.Bag(args.bag, "r")
    except Exception as exc:
        raise SystemExit("Cannot open bag {}: {}".format(args.bag, exc))

    try:
        for topic, msg, bag_time in bag.read_messages(topics=[args.topic]):
            seen += 1
            stamp = getattr(getattr(msg, "header", None), "stamp", bag_time)
            current_time = stamp.to_sec() if hasattr(stamp, "to_sec") else bag_time.to_sec()
            if next_save_time is not None and current_time < next_save_time:
                continue

            try:
                image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as exc:
                print("Skipping frame {}: {}".format(seen, exc), file=sys.stderr)
                continue

            filename = os.path.join(args.out, "frame_{:06d}.jpg".format(saved))
            ok = cv2.imwrite(
                filename,
                image,
                [int(cv2.IMWRITE_JPEG_QUALITY), args.jpg_quality],
            )
            if not ok:
                print("Failed to write {}".format(filename), file=sys.stderr)
                continue
            saved += 1
            next_save_time = current_time + args.interval
            print("{}".format(filename))
    finally:
        bag.close()

    if seen == 0:
        raise SystemExit(
            "No messages found on {}. Check the topic with: "
            "rosbag info {}".format(args.topic, args.bag)
        )
    print("Saved {} image(s) from {} message(s).".format(saved, seen))


if __name__ == "__main__":
    main()
