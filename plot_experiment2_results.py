import csv
import os
import sys
from collections import defaultdict

import matplotlib.pyplot as plt


STATE_INIT = "INIT"
STATE_INIT_PREPARE = "INIT_PREPARE"
STATE_AUTO_PERTURB_CLOSE = "AUTO_PERTURB_CLOSE"
STATE_AUTO_PERTURB_OPEN = "AUTO_PERTURB_OPEN"


def parse_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def parse_bool(value):
    return str(value).strip().lower() in {"1", "true", "yes"}


def is_time_series_csv(path):
    try:
        with open(path, "r", newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = next(reader, [])
            return "contact_force" in header and "trial_elapsed_sec" in header
    except Exception:
        return False


def collect_csv_paths(args):
    csv_paths = []
    for arg in args:
        abs_path = os.path.abspath(arg)
        if os.path.isdir(abs_path):
            for name in sorted(os.listdir(abs_path)):
                candidate = os.path.join(abs_path, name)
                if candidate.lower().endswith(".csv") and is_time_series_csv(candidate):
                    csv_paths.append(candidate)
        elif os.path.isfile(abs_path) and is_time_series_csv(abs_path):
            csv_paths.append(abs_path)
    return csv_paths


def load_time_series_rows(path):
    rows = []
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def extract_trial_rows(rows):
    trial_rows = []
    for row in rows:
        init_phase = parse_bool(row.get("init_phase", "0"))
        trial_elapsed_sec = parse_float(row.get("trial_elapsed_sec"), -1.0)
        if not init_phase and trial_elapsed_sec >= 0.0:
            trial_rows.append(row)
    return trial_rows or rows


def detect_perturb_starts(rows):
    starts = []
    prev_state = None
    for row in rows:
        state = row["state"]
        if state in {STATE_AUTO_PERTURB_CLOSE, STATE_AUTO_PERTURB_OPEN} and state != prev_state:
            starts.append((parse_float(row["trial_elapsed_sec"], parse_float(row["time_sec"])), state))
        prev_state = state
    return starts


def plot_trial_curves(rows, title):
    rows = extract_trial_rows(rows)
    time_sec = [
        parse_float(row["trial_elapsed_sec"], parse_float(row["time_sec"])) for row in rows
    ]
    contact_force = [parse_float(row["contact_force"]) for row in rows]
    gripper_opening = [parse_float(row["gripper_opening"]) for row in rows]
    force_error = [parse_float(row["force_error"]) for row in rows]

    target_low = parse_float(rows[0]["target_low"]) if rows else 0.0
    target_high = parse_float(rows[0]["target_high"]) if rows else 0.0
    perturb_starts = detect_perturb_starts(rows)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(title)

    axes[0].plot(time_sec, contact_force, color="tab:red", linewidth=1.6)
    axes[0].axhline(target_low, color="tab:green", linestyle="--", linewidth=1.0, label="target_low")
    axes[0].axhline(target_high, color="tab:blue", linestyle="--", linewidth=1.0, label="target_high")
    axes[0].set_ylabel("contact_force")
    axes[0].legend(loc="upper right")

    seen_close = False
    seen_open = False
    for start_time, state in perturb_starts:
        if state == STATE_AUTO_PERTURB_CLOSE:
            label = "AUTO_PERTURB_CLOSE start" if not seen_close else None
            seen_close = True
            axes[0].axvline(start_time, color="tab:red", alpha=0.25, linestyle=":")
            if label is not None:
                axes[0].plot([], [], color="tab:red", alpha=0.25, linestyle=":", label=label)
        else:
            label = "AUTO_PERTURB_OPEN start" if not seen_open else None
            seen_open = True
            axes[0].axvline(start_time, color="tab:blue", alpha=0.25, linestyle=":")
            if label is not None:
                axes[0].plot([], [], color="tab:blue", alpha=0.25, linestyle=":", label=label)

    axes[0].legend(loc="upper right")

    axes[1].plot(time_sec, gripper_opening, color="tab:purple", linewidth=1.6)
    axes[1].set_ylabel("gripper_opening")

    axes[2].plot(time_sec, force_error, color="tab:orange", linewidth=1.6)
    axes[2].set_xlabel("trial_elapsed_sec")
    axes[2].set_ylabel("force_error")

    fig.tight_layout()


def compute_trial_summary(rows):
    active_rows = extract_trial_rows(rows)

    if not active_rows:
        return None

    condition = active_rows[0]["condition"]
    avg_force_error = sum(parse_float(row["force_error"]) for row in active_rows) / len(active_rows)
    in_target_ratio = sum(parse_bool(row["in_target_range"]) for row in active_rows) / len(active_rows)

    return {
        "condition": condition,
        "avg_force_error": avg_force_error,
        "in_target_range_ratio": in_target_ratio,
    }


def plot_condition_summary(csv_paths):
    grouped = defaultdict(list)
    for path in csv_paths:
        rows = load_time_series_rows(path)
        summary = compute_trial_summary(rows)
        if summary is not None:
            grouped[summary["condition"]].append(summary)

    if not grouped:
        return

    conditions = sorted(grouped.keys())
    avg_force_errors = []
    avg_in_target_ratios = []
    for condition in conditions:
        summaries = grouped[condition]
        avg_force_errors.append(
            sum(item["avg_force_error"] for item in summaries) / len(summaries)
        )
        avg_in_target_ratios.append(
            sum(item["in_target_range_ratio"] for item in summaries) / len(summaries)
        )

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    fig.suptitle("Experiment 2 Condition Summary")

    axes[0].bar(conditions, avg_force_errors, color=["tab:gray", "tab:cyan"][: len(conditions)])
    axes[0].set_ylabel("avg_force_error")
    axes[0].set_title("Average Force Error")

    axes[1].bar(conditions, avg_in_target_ratios, color=["tab:gray", "tab:cyan"][: len(conditions)])
    axes[1].set_ylabel("in_target_range_ratio")
    axes[1].set_ylim(0.0, 1.0)
    axes[1].set_title("In-Target Ratio")

    fig.tight_layout()


def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_experiment2_results.py <time_series_csv_or_dir> [...]")
        sys.exit(1)

    csv_paths = collect_csv_paths(sys.argv[1:])
    if not csv_paths:
        print("No Experiment 2 time-series CSV files found.")
        sys.exit(1)

    print("Loaded files:")
    for path in csv_paths:
        print(f"  {path}")

    first_rows = load_time_series_rows(csv_paths[0])
    if not first_rows:
        print(f"No rows found in {csv_paths[0]}")
        sys.exit(1)

    plot_trial_curves(first_rows, title=os.path.basename(csv_paths[0]))

    if len(csv_paths) > 1:
        plot_condition_summary(csv_paths)

    plt.show()


if __name__ == "__main__":
    main()
