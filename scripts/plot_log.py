#!/usr/bin/env python3
"""Plot DataLogger output (common/data_logger.h/.cpp) from a simulation run.

DataLogger writes two files per run:
  - datalog.log               comma-separated rows, one per logged control
                               tick, columns in the order items were added
                               via DataLogger::addIterm.
  - matlabReadDataScript.txt   auto-generated alongside it, mapping each
                               item name to its 1-indexed column range,
                               e.g. "joint_pos=dataRec(:,2:7);".

This script parses the column layout out of matlabReadDataScript.txt
(rather than hardcoding it), so it keeps working if you add/remove/resize
logged items in the C++ side without touching this script.

Usage:
    python3 scripts/plot_log.py
    python3 scripts/plot_log.py --log record/datalog.log --out record/plot.png
    python3 scripts/plot_log.py --no-show --out record/plot.png
"""

import argparse
import json
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def parse_column_layout(script_path: Path) -> dict[str, tuple[int, int]]:
    """Return {item_name: (start_col, end_col)}, both 0-indexed inclusive."""
    pattern = re.compile(r"^(\w+)\s*=\s*dataRec\(:,\s*(\d+)\s*:\s*(\d+)\s*\);")
    layout = {}
    for line in script_path.read_text().splitlines():
        m = pattern.match(line.strip())
        if not m:
            continue
        name, start_1idx, end_1idx = m.group(1), int(m.group(2)), int(m.group(3))
        layout[name] = (start_1idx - 1, end_1idx - 1)
    if not layout:
        raise ValueError(f"no 'name=dataRec(:,a:b);' lines found in {script_path}")
    return layout


def load_joint_names(config_path: Path) -> list[str] | None:
    """Joint names in the same order PVT_Ctr/MJ_Interface see them: jsoncpp's
    getMemberNames() returns object keys sorted alphabetically, not file order."""
    if not config_path.exists():
        return None
    with config_path.open() as f:
        cfg = json.load(f)
    return sorted(cfg.keys())


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--log", type=Path, default=Path("record/datalog.log"),
                         help="path to the .log file written by DataLogger (default: %(default)s)")
    parser.add_argument("--script", type=Path, default=None,
                         help="path to matlabReadDataScript.txt (default: alongside --log)")
    parser.add_argument("--config", type=Path, default=Path("config/right_joint_ctrl_config.json"),
                         help="joint-ctrl JSON used to label per-joint columns (default: %(default)s)")
    parser.add_argument("--out", type=Path, default=None,
                         help="save the figure to this path (e.g. record/plot.png)")
    parser.add_argument("--no-show", action="store_true",
                         help="don't open an interactive window (useful for headless runs)")
    args = parser.parse_args()

    script_path = args.script or (args.log.parent / "matlabReadDataScript.txt")
    if not args.log.exists():
        raise SystemExit(f"log file not found: {args.log}")
    if not script_path.exists():
        raise SystemExit(f"column-layout file not found: {script_path}")

    layout = parse_column_layout(script_path)
    data = np.loadtxt(args.log, delimiter=",")
    if data.ndim == 1:
        data = data.reshape(1, -1)

    joint_names = load_joint_names(args.config)

    # Use "simTime" as the x-axis if it was logged, else fall back to row index.
    if "simTime" in layout:
        start, end = layout["simTime"]
        t = data[:, start] if start == end else data[:, start:end + 1].mean(axis=1)
        x_label = "sim time (s)"
    else:
        t = np.arange(data.shape[0])
        x_label = "sample"

    items = [name for name in layout if name != "simTime"]
    if not items:
        raise SystemExit("nothing to plot besides simTime")

    fig, axes = plt.subplots(len(items), 1, sharex=True, figsize=(10, 2.5 * len(items)))
    if len(items) == 1:
        axes = [axes]

    for ax, name in zip(axes, items):
        start, end = layout[name]
        width = end - start + 1
        cols = data[:, start:end + 1]

        use_joint_names = joint_names is not None and len(joint_names) == width
        for col in range(width):
            label = joint_names[col] if use_joint_names else f"{name}[{col}]"
            ax.plot(t, cols[:, col], label=label, linewidth=1)

        ax.set_ylabel(name)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=7, ncol=2)

    axes[-1].set_xlabel(x_label)
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
