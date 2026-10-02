#!/usr/bin/env python3
"""Plot a KinWBC joint-command CSV log (tests/test_WBCKin_walk_joystick.cpp).

That test writes record/test_WBCKin_walk_joystick_joint_cmd.csv every tick
once WALK starts: a header row ("time,<joint>_pos_cmd,...,<joint>_vel_cmd,...")
followed by one data row per tick. Unlike scripts/plot_log.py (which reads
DataLogger's columnless .log + a separate matlabReadDataScript.txt layout
file), this CSV is self-describing -- the header row alone is enough to find
and label every column, no side-channel layout file needed.

Usage:
    python3 scripts/plot_joint_cmd.py
    python3 scripts/plot_joint_cmd.py --log record/test_WBCKin_walk_joystick_joint_cmd.csv --out record/joint_cmd.png
    python3 scripts/plot_joint_cmd.py --no-show --out record/joint_cmd.png
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--log", type=Path, default=Path("record/test_WBCKin_walk_joystick_joint_cmd.csv"),
                         help="path to the joint-command CSV (default: %(default)s)")
    parser.add_argument("--out", type=Path, default=None,
                         help="save the figure to this path (e.g. record/joint_cmd.png)")
    parser.add_argument("--no-show", action="store_true",
                         help="don't open an interactive window (useful for headless runs)")
    args = parser.parse_args()

    if not args.log.exists():
        raise SystemExit(f"log file not found: {args.log}")

    data = np.genfromtxt(args.log, delimiter=",", names=True)
    if data.ndim == 0:
        data = data.reshape(1)

    columns = data.dtype.names
    t = data["time"]

    pos_cols = [c for c in columns if c.endswith("_pos_cmd")]
    vel_cols = [c for c in columns if c.endswith("_vel_cmd")]
    if not pos_cols and not vel_cols:
        raise SystemExit(f"no *_pos_cmd/*_vel_cmd columns found in {args.log} (columns: {columns})")

    groups = [(name, cols) for name, cols in
              [("joint position command (rad)", pos_cols), ("joint velocity command (rad/s)", vel_cols)]
              if cols]

    fig, axes = plt.subplots(len(groups), 1, sharex=True, figsize=(10, 4.5 * len(groups)))
    if len(groups) == 1:
        axes = [axes]

    for ax, (ylabel, cols) in zip(axes, groups):
        for col in cols:
            # "left_hip_pitch_joint_pos_cmd" -> "left_hip_pitch_joint"
            label = col.rsplit("_", 2)[0]
            ax.plot(t, data[col], label=label, linewidth=1)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=7, ncol=2)

    axes[-1].set_xlabel("sim time (s)")
    fig.suptitle(f"{args.log}")
    fig.tight_layout()

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.out, dpi=150)
        print(f"saved plot to {args.out}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
