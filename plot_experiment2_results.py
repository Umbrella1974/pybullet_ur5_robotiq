import argparse
import csv
import fnmatch
import math
import os
import re
import statistics
from collections import defaultdict
from dataclasses import dataclass

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch


STATE_AUTO_PERTURB_CLOSE = "AUTO_PERTURB_CLOSE"
STATE_AUTO_PERTURB_OPEN = "AUTO_PERTURB_OPEN"

EVENT_AUTO_CLOSE = "AUTO_PERTURB_START_CLOSE"
EVENT_AUTO_OPEN = "AUTO_PERTURB_START_OPEN"
EVENT_USER_CLOSE = "USER_INPUT_Z"
EVENT_USER_OPEN = "USER_INPUT_X"

DEFAULT_CONDITION_ORDER = ["visual_only", "vision_haptic"]
TRIAL_FILENAME_RE = re.compile(
    r"^experiment2_(?P<participant>.+)_(?P<condition>visual_only|vision_haptic)_(?P<stamp>\d{8}_\d{6})_timeseries\.csv$"
)


@dataclass
class DisturbanceRecord:
    direction: str
    start_time: float
    window_end_time: float
    response_time: float
    threshold_reaction_time: float
    recovery_time: float


@dataclass
class TrialRun:
    source_file: str
    timeseries_path: str
    event_path: str
    participant_id: str
    condition: str
    formal_experiment: bool
    timestamp_tag: str
    sort_key: str
    rows: list
    trial_rows: list
    event_rows: list
    trial_start_time_sec: float
    trial_id: str = ""


def parse_float(value, default=math.nan):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def parse_bool(value):
    return str(value).strip().lower() in {"1", "true", "yes"}


def safe_mean(values):
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return math.nan
    return statistics.mean(finite)


def safe_median(values):
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return math.nan
    return statistics.median(finite)


def to_csv_value(value):
    if isinstance(value, float) and math.isnan(value):
        return "NaN"
    return value


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def configure_plot_style(paper_mode):
    plt.style.use("default")
    plt.rcParams.update(
        {
            "figure.dpi": 150 if not paper_mode else 180,
            "savefig.dpi": 180 if not paper_mode else 300,
            "font.size": 10 if not paper_mode else 11,
            "axes.titlesize": 12 if not paper_mode else 13,
            "axes.labelsize": 10 if not paper_mode else 11,
            "legend.fontsize": 9 if not paper_mode else 10,
            "xtick.labelsize": 9 if not paper_mode else 10,
            "ytick.labelsize": 9 if not paper_mode else 10,
        }
    )


def save_figure(fig, out_base_path, paper_mode):
    fig.savefig(f"{out_base_path}.png", bbox_inches="tight")
    if paper_mode:
        fig.savefig(f"{out_base_path}.svg", bbox_inches="tight")
    plt.close(fig)


def load_csv_rows(path):
    rows = []
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def is_time_series_csv(path):
    try:
        with open(path, "r", newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = next(reader, [])
            return "contact_force" in header and "trial_elapsed_sec" in header
    except Exception:
        return False


def is_event_csv(path):
    try:
        with open(path, "r", newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = next(reader, [])
            return "event_type" in header and "time_sec" in header
    except Exception:
        return False


def extract_trial_rows(rows):
    trial_rows = []
    for row in rows:
        init_phase = parse_bool(row.get("init_phase", "0"))
        trial_elapsed_sec = parse_float(row.get("trial_elapsed_sec"), -1.0)
        if not init_phase and trial_elapsed_sec >= 0.0:
            trial_rows.append(row)
    return trial_rows or rows


def parse_filename_metadata(path):
    basename = os.path.basename(path)
    match = TRIAL_FILENAME_RE.match(basename)
    if not match:
        return "", "", ""
    return (
        match.group("participant"),
        match.group("condition"),
        match.group("stamp"),
    )


def collect_time_series_paths(data_dir):
    csv_paths = []
    for name in sorted(os.listdir(data_dir)):
        candidate = os.path.join(data_dir, name)
        if os.path.isfile(candidate) and candidate.lower().endswith(".csv") and is_time_series_csv(candidate):
            csv_paths.append(candidate)
    return csv_paths


def find_event_path(time_series_path):
    candidate = time_series_path.replace("_timeseries.csv", "_events.csv")
    if os.path.exists(candidate) and is_event_csv(candidate):
        return candidate
    return ""


def current_sort_key(rows, timestamp_tag, source_file):
    if rows:
        ts = rows[0].get("timestamp_iso", "")
        if ts:
            return ts
    if timestamp_tag:
        return timestamp_tag
    return source_file


def determine_trial_start_time(event_rows, trial_rows):
    explicit_starts = []
    for row in event_rows:
        time_sec = parse_float(row.get("time_sec"))
        trial_elapsed_sec = parse_float(row.get("trial_elapsed_sec"))
        if math.isfinite(time_sec) and math.isfinite(trial_elapsed_sec) and trial_elapsed_sec >= 0.0:
            explicit_starts.append(time_sec - trial_elapsed_sec)
    if explicit_starts:
        return min(explicit_starts)

    for row in event_rows:
        if row.get("event_type") in {"MANUAL_START", "TARGET_RANGE_STABLE"}:
            time_sec = parse_float(row.get("time_sec"))
            if math.isfinite(time_sec):
                return time_sec

    for row in trial_rows:
        time_sec = parse_float(row.get("time_sec"))
        trial_elapsed_sec = parse_float(row.get("trial_elapsed_sec"))
        if math.isfinite(time_sec) and math.isfinite(trial_elapsed_sec):
            return time_sec - trial_elapsed_sec

    return math.nan


def normalize_event_rows(event_rows, participant_id, condition, formal_experiment, trial_start_time_sec):
    normalized = []
    for row in event_rows:
        new_row = dict(row)
        time_sec = parse_float(new_row.get("time_sec"))
        trial_elapsed_sec = parse_float(new_row.get("trial_elapsed_sec"))

        if not math.isfinite(trial_elapsed_sec):
            if math.isfinite(time_sec) and math.isfinite(trial_start_time_sec):
                trial_elapsed_sec = time_sec - trial_start_time_sec
            else:
                trial_elapsed_sec = math.nan

        if not new_row.get("participant_id"):
            new_row["participant_id"] = participant_id
        if not new_row.get("condition"):
            new_row["condition"] = condition
        if "formal_experiment" not in new_row or new_row.get("formal_experiment", "") == "":
            new_row["formal_experiment"] = int(bool(formal_experiment))

        new_row["time_sec"] = time_sec
        new_row["trial_elapsed_sec"] = trial_elapsed_sec
        new_row["contact_force"] = parse_float(new_row.get("contact_force"))
        new_row["gripper_opening"] = parse_float(new_row.get("gripper_opening"))
        normalized.append(new_row)

    normalized.sort(key=lambda row: parse_float(row.get("time_sec"), math.inf))
    return normalized


def load_trial_run(time_series_path):
    rows = load_csv_rows(time_series_path)
    if not rows:
        return None

    file_participant, file_condition, timestamp_tag = parse_filename_metadata(time_series_path)
    participant_id = rows[0].get("participant_id") or file_participant or "unknown"
    condition = rows[0].get("condition") or file_condition or "unknown"
    formal_experiment = parse_bool(rows[0].get("formal_experiment", "0"))

    trial_rows = extract_trial_rows(rows)
    event_path = find_event_path(time_series_path)
    event_rows = load_csv_rows(event_path) if event_path else []
    trial_start_time_sec = determine_trial_start_time(event_rows, trial_rows)
    event_rows = normalize_event_rows(
        event_rows,
        participant_id=participant_id,
        condition=condition,
        formal_experiment=formal_experiment,
        trial_start_time_sec=trial_start_time_sec,
    )

    return TrialRun(
        source_file=os.path.basename(time_series_path),
        timeseries_path=os.path.abspath(time_series_path),
        event_path=os.path.abspath(event_path) if event_path else "",
        participant_id=participant_id,
        condition=condition,
        formal_experiment=formal_experiment,
        timestamp_tag=timestamp_tag,
        sort_key=current_sort_key(trial_rows, timestamp_tag, os.path.basename(time_series_path)),
        rows=rows,
        trial_rows=trial_rows,
        event_rows=event_rows,
        trial_start_time_sec=trial_start_time_sec,
    )


def assign_trial_ids(trials):
    grouped = defaultdict(list)
    for trial in trials:
        if trial.formal_experiment:
            grouped[(trial.participant_id, trial.condition)].append(trial)

    for (participant_id, condition), grouped_trials in grouped.items():
        grouped_trials.sort(key=lambda trial: trial.sort_key)
        for index, trial in enumerate(grouped_trials, start=1):
            trial.trial_id = f"{participant_id}_{condition}_T{index:02d}"


def trial_token(trial):
    if trial.trial_id:
        return trial.trial_id
    if trial.timestamp_tag:
        return f"{trial.participant_id}_{trial.condition}_debug_{trial.timestamp_tag}"
    stem = os.path.splitext(trial.source_file)[0]
    return f"{trial.participant_id}_{trial.condition}_debug_{stem}"


def build_trial_lookup(rows):
    trial_times = [parse_float(row.get("trial_elapsed_sec")) for row in rows]
    return trial_times


def find_time_series_index(trial_times, event_time):
    if not trial_times:
        return None
    best_index = None
    best_distance = math.inf
    for index, trial_time in enumerate(trial_times):
        if not math.isfinite(trial_time):
            continue
        distance = abs(trial_time - event_time)
        if distance < best_distance:
            best_distance = distance
            best_index = index
    return best_index


def split_segments_by_zone(trial_rows, trial_times):
    target_low = parse_float(trial_rows[0].get("target_low"), 0.0) if trial_rows else 0.0
    target_high = parse_float(trial_rows[0].get("target_high"), 0.0) if trial_rows else 0.0

    zones = []
    for row in trial_rows:
        force = parse_float(row.get("contact_force"))
        if force < target_low:
            zones.append("below")
        elif force > target_high:
            zones.append("above")
        else:
            zones.append("in")

    segments = []
    if not trial_rows:
        return segments

    segment_start = trial_times[0]
    segment_zone = zones[0]
    for index in range(1, len(trial_rows)):
        if zones[index] != segment_zone:
            segments.append((segment_start, trial_times[index], segment_zone))
            segment_start = trial_times[index]
            segment_zone = zones[index]

    segment_end = trial_times[-1]
    if len(trial_times) >= 2:
        segment_end = trial_times[-1] + max(0.0, trial_times[-1] - trial_times[-2])
    segments.append((segment_start, segment_end, segment_zone))
    return segments


def filter_active_event_rows(event_rows):
    return [
        row
        for row in event_rows
        if math.isfinite(parse_float(row.get("trial_elapsed_sec"))) and parse_float(row.get("trial_elapsed_sec")) >= 0.0
    ]


def compute_disturbance_records(trial, include_threshold_reaction, recovery_hold_sec):
    trial_rows = trial.trial_rows
    if not trial_rows:
        return []

    active_events = filter_active_event_rows(trial.event_rows)
    active_events.sort(key=lambda row: parse_float(row.get("trial_elapsed_sec"), math.inf))

    start_events = [
        row
        for row in active_events
        if row.get("event_type") in {EVENT_AUTO_CLOSE, EVENT_AUTO_OPEN}
    ]
    if not start_events:
        return []

    trial_times = [parse_float(row.get("trial_elapsed_sec")) for row in trial_rows]
    target_low = parse_float(trial_rows[0].get("target_low"), 0.0)
    target_high = parse_float(trial_rows[0].get("target_high"), 0.0)
    trial_end_time = max(trial_times) if trial_times else 0.0
    records = []

    for index, start_event in enumerate(start_events):
        start_time = parse_float(start_event.get("trial_elapsed_sec"))
        if not math.isfinite(start_time):
            continue

        next_start_time = trial_end_time
        if index + 1 < len(start_events):
            next_start_time = parse_float(start_events[index + 1].get("trial_elapsed_sec"), trial_end_time)

        direction = "close" if start_event.get("event_type") == EVENT_AUTO_CLOSE else "open"
        correct_user_event = EVENT_USER_OPEN if direction == "close" else EVENT_USER_CLOSE

        response_time = math.nan
        first_correct_user_time = math.nan
        for event_row in active_events:
            event_time = parse_float(event_row.get("trial_elapsed_sec"))
            if not math.isfinite(event_time):
                continue
            if event_time <= start_time:
                continue
            if event_time >= next_start_time:
                break
            if event_row.get("event_type") == correct_user_event:
                first_correct_user_time = event_time
                response_time = event_time - start_time
                break

        threshold_reaction_time = math.nan
        if include_threshold_reaction:
            threshold_cross_time = math.nan
            for row in trial_rows:
                trial_time = parse_float(row.get("trial_elapsed_sec"))
                if not math.isfinite(trial_time) or trial_time < start_time:
                    continue
                if trial_time >= next_start_time:
                    break
                contact_force = parse_float(row.get("contact_force"))
                if direction == "close" and contact_force > target_high:
                    threshold_cross_time = trial_time
                    break
                if direction == "open" and contact_force < target_low:
                    threshold_cross_time = trial_time
                    break

            if math.isfinite(threshold_cross_time):
                for event_row in active_events:
                    event_time = parse_float(event_row.get("trial_elapsed_sec"))
                    if not math.isfinite(event_time):
                        continue
                    if event_time <= threshold_cross_time:
                        continue
                    if event_time >= next_start_time:
                        break
                    if event_row.get("event_type") == correct_user_event:
                        threshold_reaction_time = event_time - threshold_cross_time
                        break

        recovery_time = math.nan
        in_range_start = math.nan
        for row in trial_rows:
            trial_time = parse_float(row.get("trial_elapsed_sec"))
            if not math.isfinite(trial_time) or trial_time < start_time:
                continue
            if trial_time >= next_start_time:
                break

            contact_force = parse_float(row.get("contact_force"))
            in_range = target_low <= contact_force <= target_high
            if in_range:
                if not math.isfinite(in_range_start):
                    in_range_start = trial_time
                if trial_time - in_range_start >= recovery_hold_sec - 1e-9:
                    recovery_time = max(0.0, in_range_start + recovery_hold_sec - start_time)
                    break
            else:
                in_range_start = math.nan

        records.append(
            DisturbanceRecord(
                direction=direction,
                start_time=start_time,
                window_end_time=next_start_time,
                response_time=response_time,
                threshold_reaction_time=threshold_reaction_time,
                recovery_time=recovery_time,
            )
        )

    return records


def build_trial_summary(trial, include_threshold_reaction, recovery_hold_sec):
    trial_rows = trial.trial_rows
    if not trial_rows:
        return None, []

    trial_times = [parse_float(row.get("trial_elapsed_sec")) for row in trial_rows]
    contact_force_values = [parse_float(row.get("contact_force")) for row in trial_rows]
    force_error_values = [parse_float(row.get("force_error")) for row in trial_rows]
    in_target_values = [1.0 if parse_bool(row.get("in_target_range")) else 0.0 for row in trial_rows]
    target_low = parse_float(trial_rows[0].get("target_low"), 0.0)
    target_high = parse_float(trial_rows[0].get("target_high"), 0.0)

    disturbances = compute_disturbance_records(
        trial,
        include_threshold_reaction=include_threshold_reaction,
        recovery_hold_sec=recovery_hold_sec,
    )
    response_times = [item.response_time for item in disturbances]
    recovery_times = [item.recovery_time for item in disturbances]
    threshold_reaction_times = [item.threshold_reaction_time for item in disturbances]

    close_responses = [item.response_time for item in disturbances if item.direction == "close"]
    open_responses = [item.response_time for item in disturbances if item.direction == "open"]
    close_recoveries = [item.recovery_time for item in disturbances if item.direction == "close"]
    open_recoveries = [item.recovery_time for item in disturbances if item.direction == "open"]

    close_thresholds = [
        item.threshold_reaction_time for item in disturbances if item.direction == "close"
    ]
    open_thresholds = [
        item.threshold_reaction_time for item in disturbances if item.direction == "open"
    ]

    active_events = filter_active_event_rows(trial.event_rows)
    manual_open_count = sum(1 for row in active_events if row.get("event_type") == EVENT_USER_OPEN)
    manual_close_count = sum(1 for row in active_events if row.get("event_type") == EVENT_USER_CLOSE)
    manual_operation_count = manual_open_count + manual_close_count
    active_duration = max(trial_times) if trial_times else 0.0
    manual_operation_per_minute = math.nan
    if active_duration > 0.0:
        manual_operation_per_minute = manual_operation_count / (active_duration / 60.0)

    summary = {
        "trial_id": trial.trial_id,
        "source_file": trial.source_file,
        "participant_id": trial.participant_id,
        "condition": trial.condition,
        "formal_experiment": int(bool(trial.formal_experiment)),
        "trial_duration_sec": active_duration,
        "mean_contact_force": safe_mean(contact_force_values),
        "mean_force_error": safe_mean(force_error_values),
        "median_force_error": safe_median(force_error_values),
        "in_target_range_ratio": safe_mean(in_target_values),
        "time_above_high_ratio": safe_mean(
            [1.0 if value > target_high else 0.0 for value in contact_force_values]
        ),
        "time_below_low_ratio": safe_mean(
            [1.0 if value < target_low else 0.0 for value in contact_force_values]
        ),
        "n_auto_close": sum(1 for item in disturbances if item.direction == "close"),
        "n_auto_open": sum(1 for item in disturbances if item.direction == "open"),
        "n_response_events": len(disturbances),
        "n_valid_response_events": len([value for value in response_times if math.isfinite(value)]),
        "response_valid_ratio": safe_mean(
            [1.0 if math.isfinite(value) else 0.0 for value in response_times]
        ),
        "mean_disturbance_response_time": safe_mean(response_times),
        "median_disturbance_response_time": safe_median(response_times),
        "mean_disturbance_response_time_close": safe_mean(close_responses),
        "mean_disturbance_response_time_open": safe_mean(open_responses),
        "n_recovery_events": len(disturbances),
        "n_valid_recovery_events": len([value for value in recovery_times if math.isfinite(value)]),
        "recovery_valid_ratio": safe_mean(
            [1.0 if math.isfinite(value) else 0.0 for value in recovery_times]
        ),
        "mean_recovery_time": safe_mean(recovery_times),
        "median_recovery_time": safe_median(recovery_times),
        "mean_recovery_time_close": safe_mean(close_recoveries),
        "mean_recovery_time_open": safe_mean(open_recoveries),
        "manual_operation_count": manual_operation_count,
        "manual_open_count": manual_open_count,
        "manual_close_count": manual_close_count,
        "manual_operation_per_minute": manual_operation_per_minute,
    }

    if include_threshold_reaction:
        summary.update(
            {
                "n_threshold_events": len(disturbances),
                "n_valid_threshold_reaction_events": len(
                    [value for value in threshold_reaction_times if math.isfinite(value)]
                ),
                "threshold_reaction_valid_ratio": safe_mean(
                    [1.0 if math.isfinite(value) else 0.0 for value in threshold_reaction_times]
                ),
                "mean_threshold_reaction_time": safe_mean(threshold_reaction_times),
                "median_threshold_reaction_time": safe_median(threshold_reaction_times),
                "mean_threshold_reaction_time_close": safe_mean(close_thresholds),
                "mean_threshold_reaction_time_open": safe_mean(open_thresholds),
            }
        )

    return summary, disturbances


def condition_order_key(conditions, requested_order):
    ordered = [condition for condition in requested_order if condition in conditions]
    remaining = sorted(condition for condition in conditions if condition not in ordered)
    return ordered + remaining


def aggregate_participant_condition_metric(summary_rows, metric_key, condition_order):
    grouped = defaultdict(list)
    for row in summary_rows:
        value = parse_float(row.get(metric_key))
        if not math.isfinite(value):
            continue
        grouped[(row["participant_id"], row["condition"])].append(value)

    participant_map = defaultdict(dict)
    for (participant_id, condition), values in grouped.items():
        participant_map[participant_id][condition] = safe_mean(values)

    ordered_conditions = condition_order_key(
        {condition for _, condition in grouped.keys()},
        condition_order,
    )
    return participant_map, ordered_conditions


def aggregate_participant_condition_components(summary_rows, metric_keys, condition_order):
    grouped = defaultdict(lambda: defaultdict(list))
    for row in summary_rows:
        participant_id = row["participant_id"]
        condition = row["condition"]
        for metric_key in metric_keys:
            value = parse_float(row.get(metric_key))
            if math.isfinite(value):
                grouped[(participant_id, condition)][metric_key].append(value)

    participant_map = defaultdict(dict)
    for (participant_id, condition), metrics in grouped.items():
        participant_map[participant_id][condition] = {
            metric_key: safe_mean(values) for metric_key, values in metrics.items()
        }

    ordered_conditions = condition_order_key(
        {condition for _, condition in grouped.keys()},
        condition_order,
    )
    return participant_map, ordered_conditions


def plot_paired_metric(summary_rows, metric_key, title, ylabel, out_base_path, condition_order, paper_mode):
    participant_map, ordered_conditions = aggregate_participant_condition_metric(
        summary_rows, metric_key, condition_order
    )
    if not ordered_conditions:
        return False

    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    x_positions = list(range(len(ordered_conditions)))
    colors = {"visual_only": "tab:gray", "vision_haptic": "tab:cyan"}

    for participant_id, by_condition in sorted(participant_map.items()):
        xs = []
        ys = []
        for index, condition in enumerate(ordered_conditions):
            value = by_condition.get(condition)
            if value is None or not math.isfinite(value):
                continue
            xs.append(index)
            ys.append(value)
        if ys:
            ax.plot(xs, ys, color="0.75", linewidth=1.2, alpha=0.8, zorder=1)
            ax.scatter(xs, ys, color="0.35", s=18, alpha=0.85, zorder=2)

    for index, condition in enumerate(ordered_conditions):
        condition_values = []
        for by_condition in participant_map.values():
            value = by_condition.get(condition)
            if value is not None and math.isfinite(value):
                condition_values.append(value)

        if not condition_values:
            continue

        mean_value = safe_mean(condition_values)
        sem_value = 0.0
        if len(condition_values) > 1:
            sem_value = statistics.stdev(condition_values) / math.sqrt(len(condition_values))

        ax.scatter(
            [index] * len(condition_values),
            condition_values,
            color=colors.get(condition, "tab:blue"),
            edgecolor="black",
            linewidths=0.4,
            alpha=0.7,
            s=36,
            zorder=3,
        )
        ax.errorbar(
            index,
            mean_value,
            yerr=sem_value,
            color="black",
            marker="o",
            markersize=7,
            capsize=4,
            linewidth=1.6,
            zorder=4,
        )

    ax.set_xticks(x_positions)
    ax.set_xticklabels(ordered_conditions)
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(axis="y", linestyle=":", alpha=0.35)
    fig.tight_layout()
    save_figure(fig, out_base_path, paper_mode)
    return True


def plot_target_ratio_stack(summary_rows, out_base_path, condition_order, paper_mode):
    participant_map, ordered_conditions = aggregate_participant_condition_components(
        summary_rows,
        ["time_below_low_ratio", "in_target_range_ratio", "time_above_high_ratio"],
        condition_order,
    )
    if not ordered_conditions:
        return False

    below_means = []
    in_means = []
    above_means = []
    for condition in ordered_conditions:
        below_values = []
        in_values = []
        above_values = []
        for by_condition in participant_map.values():
            metrics = by_condition.get(condition)
            if not metrics:
                continue
            below_values.append(metrics.get("time_below_low_ratio", math.nan))
            in_values.append(metrics.get("in_target_range_ratio", math.nan))
            above_values.append(metrics.get("time_above_high_ratio", math.nan))

        below_means.append(safe_mean(below_values))
        in_means.append(safe_mean(in_values))
        above_means.append(safe_mean(above_values))

    x_positions = list(range(len(ordered_conditions)))
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    ax.bar(x_positions, below_means, color="tab:orange", label="below target")
    ax.bar(x_positions, in_means, bottom=below_means, color="tab:green", label="in target")
    stacked_bottom = [
        (0.0 if math.isnan(below) else below) + (0.0 if math.isnan(in_value) else in_value)
        for below, in_value in zip(below_means, in_means)
    ]
    ax.bar(x_positions, above_means, bottom=stacked_bottom, color="tab:red", label="above target")
    ax.set_xticks(x_positions)
    ax.set_xticklabels(ordered_conditions)
    ax.set_ylim(0.0, 1.0)
    ax.set_ylabel("ratio")
    ax.set_title("Target Range Ratio Stack")
    ax.legend(loc="upper right")
    ax.grid(axis="y", linestyle=":", alpha=0.35)
    fig.tight_layout()
    save_figure(fig, out_base_path, paper_mode)
    return True


def plot_user_workload(summary_rows, out_base_path, condition_order, paper_mode):
    participant_map, ordered_conditions = aggregate_participant_condition_components(
        summary_rows,
        ["manual_operation_per_minute", "manual_open_count", "manual_close_count"],
        condition_order,
    )
    if not ordered_conditions:
        return False

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))
    colors = {"visual_only": "tab:gray", "vision_haptic": "tab:cyan"}
    x_positions = list(range(len(ordered_conditions)))

    for participant_id, by_condition in sorted(participant_map.items()):
        xs = []
        ys = []
        for index, condition in enumerate(ordered_conditions):
            metrics = by_condition.get(condition)
            if not metrics:
                continue
            value = metrics.get("manual_operation_per_minute", math.nan)
            if math.isfinite(value):
                xs.append(index)
                ys.append(value)
        if ys:
            axes[0].plot(xs, ys, color="0.75", linewidth=1.2, alpha=0.8)
            axes[0].scatter(xs, ys, color="0.35", s=18, alpha=0.85)

    for index, condition in enumerate(ordered_conditions):
        values = []
        open_values = []
        close_values = []
        for by_condition in participant_map.values():
            metrics = by_condition.get(condition)
            if not metrics:
                continue
            if math.isfinite(metrics.get("manual_operation_per_minute", math.nan)):
                values.append(metrics["manual_operation_per_minute"])
            if math.isfinite(metrics.get("manual_open_count", math.nan)):
                open_values.append(metrics["manual_open_count"])
            if math.isfinite(metrics.get("manual_close_count", math.nan)):
                close_values.append(metrics["manual_close_count"])

        if values:
            mean_value = safe_mean(values)
            sem_value = 0.0
            if len(values) > 1:
                sem_value = statistics.stdev(values) / math.sqrt(len(values))
            axes[0].scatter(
                [index] * len(values),
                values,
                color=colors.get(condition, "tab:blue"),
                edgecolor="black",
                linewidths=0.4,
                alpha=0.7,
                s=36,
            )
            axes[0].errorbar(
                index,
                mean_value,
                yerr=sem_value,
                color="black",
                marker="o",
                markersize=7,
                capsize=4,
                linewidth=1.6,
            )

        mean_open = safe_mean(open_values)
        mean_close = safe_mean(close_values)
        axes[1].bar(index, mean_close if math.isfinite(mean_close) else 0.0, color="tab:orange")
        axes[1].bar(
            index,
            mean_open if math.isfinite(mean_open) else 0.0,
            bottom=(mean_close if math.isfinite(mean_close) else 0.0),
            color="tab:blue",
        )

    axes[0].set_xticks(x_positions)
    axes[0].set_xticklabels(ordered_conditions)
    axes[0].set_ylabel("manual ops / minute")
    axes[0].set_title("Manual Workload")
    axes[0].grid(axis="y", linestyle=":", alpha=0.35)

    axes[1].set_xticks(x_positions)
    axes[1].set_xticklabels(ordered_conditions)
    axes[1].set_ylabel("mean count / trial")
    axes[1].set_title("Open vs Close Count")
    axes[1].legend(
        handles=[
            Patch(facecolor="tab:orange", label="manual close"),
            Patch(facecolor="tab:blue", label="manual open"),
        ],
        loc="upper right",
    )
    axes[1].grid(axis="y", linestyle=":", alpha=0.35)

    fig.tight_layout()
    save_figure(fig, out_base_path, paper_mode)
    return True


def write_summary_csv(summary_rows, out_path, include_threshold_reaction):
    fieldnames = [
        "trial_id",
        "source_file",
        "participant_id",
        "condition",
        "formal_experiment",
        "trial_duration_sec",
        "mean_contact_force",
        "mean_force_error",
        "median_force_error",
        "in_target_range_ratio",
        "time_above_high_ratio",
        "time_below_low_ratio",
        "n_auto_close",
        "n_auto_open",
        "n_response_events",
        "n_valid_response_events",
        "response_valid_ratio",
        "mean_disturbance_response_time",
        "median_disturbance_response_time",
        "mean_disturbance_response_time_close",
        "mean_disturbance_response_time_open",
        "n_recovery_events",
        "n_valid_recovery_events",
        "recovery_valid_ratio",
        "mean_recovery_time",
        "median_recovery_time",
        "mean_recovery_time_close",
        "mean_recovery_time_open",
        "manual_operation_count",
        "manual_open_count",
        "manual_close_count",
        "manual_operation_per_minute",
    ]
    if include_threshold_reaction:
        fieldnames.extend(
            [
                "n_threshold_events",
                "n_valid_threshold_reaction_events",
                "threshold_reaction_valid_ratio",
                "mean_threshold_reaction_time",
                "median_threshold_reaction_time",
                "mean_threshold_reaction_time_close",
                "mean_threshold_reaction_time_open",
            ]
        )

    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in summary_rows:
            writer.writerow({key: to_csv_value(row.get(key, "")) for key in fieldnames})


def event_legend_handles(include_user=True):
    handles = [
        Line2D([0], [0], color="tab:red", linestyle=":", label="AUTO_PERTURB_CLOSE start"),
        Line2D([0], [0], color="tab:blue", linestyle=":", label="AUTO_PERTURB_OPEN start"),
    ]
    if include_user:
        handles.extend(
            [
                Line2D(
                    [0],
                    [0],
                    marker="v",
                    color="tab:orange",
                    linestyle="None",
                    label="USER_INPUT_Z (close)",
                ),
                Line2D(
                    [0],
                    [0],
                    marker="^",
                    color="tab:green",
                    linestyle="None",
                    label="USER_INPUT_X (open)",
                ),
            ]
        )
    return handles


def add_event_markers(ax, trial):
    for event_row in filter_active_event_rows(trial.event_rows):
        event_type = event_row.get("event_type")
        event_time = parse_float(event_row.get("trial_elapsed_sec"))
        if not math.isfinite(event_time):
            continue
        if event_type == EVENT_AUTO_CLOSE:
            ax.axvline(event_time, color="tab:red", linestyle=":", alpha=0.35)
        elif event_type == EVENT_AUTO_OPEN:
            ax.axvline(event_time, color="tab:blue", linestyle=":", alpha=0.35)


def add_user_scatter(ax, trial):
    for event_row in filter_active_event_rows(trial.event_rows):
        event_type = event_row.get("event_type")
        event_time = parse_float(event_row.get("trial_elapsed_sec"))
        event_force = parse_float(event_row.get("contact_force"))
        if not math.isfinite(event_time) or not math.isfinite(event_force):
            continue
        if event_type == EVENT_USER_CLOSE:
            ax.scatter(event_time, event_force, marker="v", color="tab:orange", s=30, zorder=4)
        elif event_type == EVENT_USER_OPEN:
            ax.scatter(event_time, event_force, marker="^", color="tab:green", s=30, zorder=4)


def title_prefix(trial):
    trial_label = trial.trial_id if trial.trial_id else "debug"
    return f"{trial.participant_id} | {trial.condition} | {trial_label}"


def plot_single_trial_overview(trial, out_dir, paper_mode):
    trial_rows = trial.trial_rows
    if not trial_rows:
        return

    trial_times = [parse_float(row.get("trial_elapsed_sec")) for row in trial_rows]
    contact_force = [parse_float(row.get("contact_force")) for row in trial_rows]
    gripper_opening = [parse_float(row.get("gripper_opening")) for row in trial_rows]
    force_error = [parse_float(row.get("force_error")) for row in trial_rows]
    target_low = parse_float(trial_rows[0].get("target_low"), 0.0)
    target_high = parse_float(trial_rows[0].get("target_high"), 0.0)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9.5), sharex=True)
    fig.suptitle(f"Single Trial Overview | {title_prefix(trial)}")

    axes[0].axhspan(target_low, target_high, color="tab:green", alpha=0.08)
    axes[0].plot(trial_times, contact_force, color="tab:red", linewidth=1.6)
    axes[0].axhline(target_low, color="tab:green", linestyle="--", linewidth=1.0)
    axes[0].axhline(target_high, color="tab:blue", linestyle="--", linewidth=1.0)
    add_event_markers(axes[0], trial)
    add_user_scatter(axes[0], trial)
    axes[0].set_ylabel("contact_force")
    axes[0].legend(handles=event_legend_handles(include_user=True), loc="upper right")
    axes[0].grid(axis="y", linestyle=":", alpha=0.35)

    axes[1].plot(trial_times, gripper_opening, color="tab:purple", linewidth=1.6)
    axes[1].set_ylabel("gripper_opening")
    axes[1].grid(axis="y", linestyle=":", alpha=0.35)

    axes[2].plot(trial_times, force_error, color="tab:orange", linewidth=1.6)
    axes[2].set_xlabel("trial_elapsed_sec")
    axes[2].set_ylabel("force_error")
    axes[2].grid(axis="y", linestyle=":", alpha=0.35)

    fig.tight_layout()
    save_figure(
        fig,
        os.path.join(out_dir, f"single_trial_overview_{trial_token(trial)}"),
        paper_mode,
    )


def plot_force_with_events(trial, out_dir, paper_mode):
    trial_rows = trial.trial_rows
    if not trial_rows:
        return

    trial_times = [parse_float(row.get("trial_elapsed_sec")) for row in trial_rows]
    contact_force = [parse_float(row.get("contact_force")) for row in trial_rows]
    target_low = parse_float(trial_rows[0].get("target_low"), 0.0)
    target_high = parse_float(trial_rows[0].get("target_high"), 0.0)

    fig, ax = plt.subplots(figsize=(12, 4.8))
    fig.suptitle(f"Force With Events | {title_prefix(trial)}")
    ax.axhspan(target_low, target_high, color="tab:green", alpha=0.08, label="target range")
    ax.plot(trial_times, contact_force, color="tab:red", linewidth=1.8, label="contact_force")
    ax.axhline(target_low, color="tab:green", linestyle="--", linewidth=1.0)
    ax.axhline(target_high, color="tab:blue", linestyle="--", linewidth=1.0)
    add_event_markers(ax, trial)
    add_user_scatter(ax, trial)
    ax.set_xlabel("trial_elapsed_sec")
    ax.set_ylabel("contact_force")
    ax.grid(axis="y", linestyle=":", alpha=0.35)
    ax.legend(handles=[Patch(facecolor="tab:green", alpha=0.08, label="target range")] + event_legend_handles())
    fig.tight_layout()
    save_figure(
        fig,
        os.path.join(out_dir, f"force_with_events_{trial_token(trial)}"),
        paper_mode,
    )


def plot_force_gripper_alignment(trial, out_dir, paper_mode):
    trial_rows = trial.trial_rows
    if not trial_rows:
        return

    trial_times = [parse_float(row.get("trial_elapsed_sec")) for row in trial_rows]
    contact_force = [parse_float(row.get("contact_force")) for row in trial_rows]
    gripper_opening = [parse_float(row.get("gripper_opening")) for row in trial_rows]

    fig, ax_force = plt.subplots(figsize=(12, 4.8))
    fig.suptitle(f"Force / Gripper Alignment | {title_prefix(trial)}")
    ax_grip = ax_force.twinx()

    ax_force.plot(trial_times, contact_force, color="tab:red", linewidth=1.8, label="contact_force")
    ax_grip.plot(trial_times, gripper_opening, color="tab:purple", linewidth=1.6, label="gripper_opening")
    add_event_markers(ax_force, trial)

    ax_force.set_xlabel("trial_elapsed_sec")
    ax_force.set_ylabel("contact_force", color="tab:red")
    ax_grip.set_ylabel("gripper_opening", color="tab:purple")
    ax_force.grid(axis="y", linestyle=":", alpha=0.35)

    handles = [
        Line2D([0], [0], color="tab:red", linewidth=1.8, label="contact_force"),
        Line2D([0], [0], color="tab:purple", linewidth=1.6, label="gripper_opening"),
        Line2D([0], [0], color="tab:red", linestyle=":", label="AUTO_PERTURB_CLOSE start"),
        Line2D([0], [0], color="tab:blue", linestyle=":", label="AUTO_PERTURB_OPEN start"),
    ]
    ax_force.legend(handles=handles, loc="upper right")

    fig.tight_layout()
    save_figure(
        fig,
        os.path.join(out_dir, f"force_gripper_alignment_{trial_token(trial)}"),
        paper_mode,
    )


def plot_target_range_band(trial, out_dir, paper_mode):
    trial_rows = trial.trial_rows
    if not trial_rows:
        return

    trial_times = [parse_float(row.get("trial_elapsed_sec")) for row in trial_rows]
    contact_force = [parse_float(row.get("contact_force")) for row in trial_rows]
    target_low = parse_float(trial_rows[0].get("target_low"), 0.0)
    target_high = parse_float(trial_rows[0].get("target_high"), 0.0)
    segments = split_segments_by_zone(trial_rows, trial_times)
    zone_colors = {"below": "tab:orange", "in": "tab:green", "above": "tab:red"}

    fig, axes = plt.subplots(2, 1, figsize=(12, 5.5), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    fig.suptitle(f"Target Range Band | {title_prefix(trial)}")

    axes[0].plot(trial_times, contact_force, color="tab:red", linewidth=1.8)
    axes[0].axhline(target_low, color="tab:green", linestyle="--", linewidth=1.0)
    axes[0].axhline(target_high, color="tab:blue", linestyle="--", linewidth=1.0)
    add_event_markers(axes[0], trial)
    axes[0].set_ylabel("contact_force")
    axes[0].grid(axis="y", linestyle=":", alpha=0.35)

    for start_time, end_time, zone in segments:
        width = max(0.0, end_time - start_time)
        axes[1].broken_barh([(start_time, width)], (0, 1), facecolors=zone_colors[zone])
    axes[1].set_ylim(0, 1)
    axes[1].set_yticks([])
    axes[1].set_xlabel("trial_elapsed_sec")
    axes[1].legend(
        handles=[
            Patch(facecolor="tab:orange", label="below target"),
            Patch(facecolor="tab:green", label="in target"),
            Patch(facecolor="tab:red", label="above target"),
        ],
        loc="upper right",
    )

    fig.tight_layout()
    save_figure(
        fig,
        os.path.join(out_dir, f"target_range_band_{trial_token(trial)}"),
        paper_mode,
    )


def plot_left_right_fingertip_forces(trial, out_dir, paper_mode):
    trial_rows = trial.trial_rows
    if not trial_rows:
        return

    trial_times = [parse_float(row.get("trial_elapsed_sec")) for row in trial_rows]
    tf_force = [parse_float(row.get("tf_force_smooth")) for row in trial_rows]
    if_force = [parse_float(row.get("if_force_smooth")) for row in trial_rows]
    force_diff = [abs(left - right) for left, right in zip(tf_force, if_force)]

    fig, axes = plt.subplots(2, 1, figsize=(12, 6.5), sharex=True)
    fig.suptitle(f"Left / Right Fingertip Forces | {title_prefix(trial)}")

    axes[0].plot(trial_times, tf_force, color="tab:blue", linewidth=1.6, label="tf_force_smooth")
    axes[0].plot(trial_times, if_force, color="tab:orange", linewidth=1.6, label="if_force_smooth")
    axes[0].set_ylabel("fingertip force")
    axes[0].legend(loc="upper right")
    axes[0].grid(axis="y", linestyle=":", alpha=0.35)

    axes[1].plot(trial_times, force_diff, color="tab:purple", linewidth=1.6)
    axes[1].set_xlabel("trial_elapsed_sec")
    axes[1].set_ylabel("|tf - if|")
    axes[1].grid(axis="y", linestyle=":", alpha=0.35)

    fig.tight_layout()
    save_figure(
        fig,
        os.path.join(out_dir, f"left_right_fingertip_forces_{trial_token(trial)}"),
        paper_mode,
    )


def plot_single_trial_figures(trial, out_dir, paper_mode):
    plot_single_trial_overview(trial, out_dir, paper_mode)
    plot_force_with_events(trial, out_dir, paper_mode)
    plot_force_gripper_alignment(trial, out_dir, paper_mode)
    plot_target_range_band(trial, out_dir, paper_mode)
    plot_left_right_fingertip_forces(trial, out_dir, paper_mode)


def build_condition_summary_figures(summary_rows, out_dir, condition_order, paper_mode, include_threshold_reaction):
    created = []
    if plot_paired_metric(
        summary_rows,
        "mean_force_error",
        "Mean Force Error By Condition",
        "mean_force_error",
        os.path.join(out_dir, "mean_force_error_by_condition"),
        condition_order,
        paper_mode,
    ):
        created.append("mean_force_error_by_condition")

    if plot_paired_metric(
        summary_rows,
        "in_target_range_ratio",
        "In-Target Ratio By Condition",
        "in_target_range_ratio",
        os.path.join(out_dir, "in_target_range_ratio_by_condition"),
        condition_order,
        paper_mode,
    ):
        created.append("in_target_range_ratio_by_condition")

    if plot_paired_metric(
        summary_rows,
        "mean_disturbance_response_time",
        "Disturbance Response Time By Condition",
        "disturbance_response_time (s)",
        os.path.join(out_dir, "disturbance_response_time_by_condition"),
        condition_order,
        paper_mode,
    ):
        created.append("disturbance_response_time_by_condition")

    if plot_paired_metric(
        summary_rows,
        "mean_recovery_time",
        "Recovery Time By Condition",
        "recovery_time (s)",
        os.path.join(out_dir, "recovery_time_by_condition"),
        condition_order,
        paper_mode,
    ):
        created.append("recovery_time_by_condition")

    if plot_paired_metric(
        summary_rows,
        "mean_disturbance_response_time_close",
        "Disturbance Response Time By Condition (close-only)",
        "disturbance_response_time (s)",
        os.path.join(out_dir, "disturbance_response_time_close_only_by_condition"),
        condition_order,
        paper_mode,
    ):
        created.append("disturbance_response_time_close_only_by_condition")

    if plot_paired_metric(
        summary_rows,
        "mean_disturbance_response_time_open",
        "Disturbance Response Time By Condition (open-only)",
        "disturbance_response_time (s)",
        os.path.join(out_dir, "disturbance_response_time_open_only_by_condition"),
        condition_order,
        paper_mode,
    ):
        created.append("disturbance_response_time_open_only_by_condition")

    if plot_paired_metric(
        summary_rows,
        "mean_recovery_time_close",
        "Recovery Time By Condition (close-only)",
        "recovery_time (s)",
        os.path.join(out_dir, "recovery_time_close_only_by_condition"),
        condition_order,
        paper_mode,
    ):
        created.append("recovery_time_close_only_by_condition")

    if plot_paired_metric(
        summary_rows,
        "mean_recovery_time_open",
        "Recovery Time By Condition (open-only)",
        "recovery_time (s)",
        os.path.join(out_dir, "recovery_time_open_only_by_condition"),
        condition_order,
        paper_mode,
    ):
        created.append("recovery_time_open_only_by_condition")

    if include_threshold_reaction and plot_paired_metric(
        summary_rows,
        "mean_threshold_reaction_time",
        "Threshold Reaction Time By Condition",
        "threshold_reaction_time (s)",
        os.path.join(out_dir, "threshold_reaction_time_by_condition"),
        condition_order,
        paper_mode,
    ):
        created.append("threshold_reaction_time_by_condition")

    if plot_target_ratio_stack(
        summary_rows,
        os.path.join(out_dir, "target_range_ratio_stack"),
        condition_order,
        paper_mode,
    ):
        created.append("target_range_ratio_stack")

    if plot_user_workload(
        summary_rows,
        os.path.join(out_dir, "user_workload_by_condition"),
        condition_order,
        paper_mode,
    ):
        created.append("user_workload_by_condition")

    return created


def trial_matches_pattern(trial, pattern):
    pattern = pattern.strip()
    if not pattern:
        return False

    if os.path.exists(pattern):
        abs_pattern = os.path.abspath(pattern)
        return abs_pattern in {trial.timeseries_path, trial.event_path}

    return any(
        fnmatch.fnmatch(candidate, pattern)
        for candidate in (
            trial.source_file,
            trial.timeseries_path,
            os.path.basename(trial.event_path) if trial.event_path else "",
        )
    )


def select_single_trials(trials, patterns):
    selected = []
    for trial in trials:
        if any(trial_matches_pattern(trial, pattern) for pattern in patterns):
            selected.append(trial)
    return selected


def build_argument_parser():
    parser = argparse.ArgumentParser(description="Plot and summarize Experiment 2 results.")
    parser.add_argument(
        "--data-dir",
        default="experiment2_data",
        help="Directory containing Experiment 2 time-series and event CSV files.",
    )
    parser.add_argument(
        "--out-dir",
        default="experiment2_figures",
        help="Directory to save generated figures and summary CSV.",
    )
    parser.add_argument(
        "--single",
        action="append",
        default=[],
        help="Generate single-trial figures for a file path or glob-like pattern.",
    )
    parser.add_argument(
        "--summary",
        action="store_true",
        help="Generate summary CSV and condition-level figures.",
    )
    parser.add_argument(
        "--all-trials",
        action="store_true",
        help="Generate single-trial figures for all included trials.",
    )
    parser.add_argument(
        "--recovery-hold-sec",
        type=float,
        default=0.2,
        help="Recovery hold duration used to define recovery_time.",
    )
    parser.add_argument(
        "--include-threshold-reaction",
        action="store_true",
        help="Also compute and plot threshold_reaction_time.",
    )
    parser.add_argument(
        "--condition-order",
        nargs="+",
        default=DEFAULT_CONDITION_ORDER,
        help="Preferred condition order for condition-level figures.",
    )
    parser.add_argument(
        "--paper-mode",
        action="store_true",
        help="Also export SVG and use paper-oriented figure settings.",
    )
    parser.add_argument(
        "--include-nonformal",
        action="store_true",
        help="Include debug / non-formal runs. They will not receive trial_id values.",
    )
    return parser


def main():
    parser = build_argument_parser()
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = args.data_dir
    out_dir = args.out_dir
    if not os.path.isabs(data_dir):
        data_dir = os.path.join(script_dir, data_dir)
    if not os.path.isabs(out_dir):
        out_dir = os.path.join(script_dir, out_dir)

    if not os.path.isdir(data_dir):
        print(f"Data directory not found: {data_dir}")
        return 1

    ensure_dir(out_dir)
    configure_plot_style(args.paper_mode)

    time_series_paths = collect_time_series_paths(data_dir)
    if not time_series_paths:
        print(f"No Experiment 2 time-series CSV files found in: {data_dir}")
        return 1

    all_trials = []
    for path in time_series_paths:
        trial = load_trial_run(path)
        if trial is not None:
            all_trials.append(trial)

    assign_trial_ids(all_trials)

    if args.include_nonformal:
        included_trials = list(all_trials)
    else:
        included_trials = [trial for trial in all_trials if trial.formal_experiment]

    skipped_trials = [trial for trial in all_trials if trial not in included_trials]

    print(f"Loaded time-series files: {len(all_trials)}")
    print(f"Included trials: {len(included_trials)}")
    print(f"Skipped trials: {len(skipped_trials)}")
    if skipped_trials and not args.include_nonformal:
        print("Skipped non-formal trials by default. Use --include-nonformal to include them.")

    if not included_trials:
        print("No trials available after filtering.")
        return 1

    run_summary = args.summary or (not args.single and not args.all_trials)
    single_trials = []
    if args.all_trials:
        single_trials = list(included_trials)
    elif args.single:
        single_trials = select_single_trials(included_trials, args.single)
        if not single_trials:
            print("No included trials matched --single patterns.")

    summary_rows = []
    for trial in included_trials:
        summary_row, _disturbances = build_trial_summary(
            trial,
            include_threshold_reaction=args.include_threshold_reaction,
            recovery_hold_sec=args.recovery_hold_sec,
        )
        if summary_row is not None:
            summary_rows.append(summary_row)

    if run_summary:
        summary_csv_path = os.path.join(out_dir, "experiment2_summary_by_trial.csv")
        write_summary_csv(
            summary_rows,
            summary_csv_path,
            include_threshold_reaction=args.include_threshold_reaction,
        )
        print(f"Saved summary CSV: {summary_csv_path}")

        created_summary_figures = build_condition_summary_figures(
            summary_rows,
            out_dir=out_dir,
            condition_order=args.condition_order,
            paper_mode=args.paper_mode,
            include_threshold_reaction=args.include_threshold_reaction,
        )
        for name in created_summary_figures:
            print(f"Saved figure: {os.path.join(out_dir, name + '.png')}")

    for trial in single_trials:
        plot_single_trial_figures(trial, out_dir=out_dir, paper_mode=args.paper_mode)
        print(f"Saved single-trial figures for: {trial.source_file}")

    print(f"Output directory: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
