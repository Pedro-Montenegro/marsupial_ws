#!/usr/bin/env python3
"""Analyze a recorded 4WD performance run and generate persistent reports."""

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


WHEELS = ("FL", "RL", "FR", "RR")


def finite(value):
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (TypeError, ValueError):
        return None


def mean(values):
    clean = [value for value in values if value is not None]
    return statistics.fmean(clean) if clean else None


def rms(values):
    clean = [value for value in values if value is not None]
    return math.sqrt(statistics.fmean(value * value for value in clean)) if clean else None


def percentile(values, fraction):
    clean = sorted(value for value in values if value is not None)
    if not clean:
        return None
    index = fraction * (len(clean) - 1)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return clean[lower]
    weight = index - lower
    return clean[lower] * (1.0 - weight) + clean[upper] * weight


def unwrap(values):
    output = []
    offset = 0.0
    previous = None
    for value in values:
        if value is None:
            output.append(None)
            continue
        if previous is not None:
            delta = value - previous
            if delta > math.pi:
                offset -= 2.0 * math.pi
            elif delta < -math.pi:
                offset += 2.0 * math.pi
        previous = value
        output.append(value + offset)
    return output


def integrate_body_commands(rows):
    x = y = yaw = 0.0
    previous_t = None
    for row in rows:
        t = finite(row.get("t_s"))
        v = finite(row.get("cmd_v"))
        w = finite(row.get("cmd_w"))
        if t is None or v is None or w is None:
            continue
        if previous_t is not None:
            dt = max(0.0, min(0.2, t - previous_t))
            middle = yaw + 0.5 * w * dt
            x += v * math.cos(middle) * dt
            y += v * math.sin(middle) * dt
            yaw += w * dt
        previous_t = t
    return {"x_m": x, "y_m": y, "yaw_rad": yaw}


def relative_trajectory(rows, prefix):
    points = []
    x0 = y0 = yaw0 = None
    yaws = unwrap([finite(row.get(f"{prefix}_yaw")) for row in rows])
    for row, yaw in zip(rows, yaws):
        x = finite(row.get(f"{prefix}_x"))
        y = finite(row.get(f"{prefix}_y"))
        if x is None or y is None or yaw is None:
            points.append(None)
            continue
        if x0 is None:
            x0, y0, yaw0 = x, y, yaw
        dx = x - x0
        dy = y - y0
        local_x = math.cos(yaw0) * dx + math.sin(yaw0) * dy
        local_y = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
        points.append((local_x, local_y, yaw - yaw0))
    return points


def path_length(points):
    total = 0.0
    previous = None
    for point in points:
        if point is None:
            continue
        if previous is not None:
            total += math.hypot(point[0] - previous[0], point[1] - previous[1])
        previous = point
    return total if previous is not None else None


def final_point(points):
    for point in reversed(points):
        if point is not None:
            return point
    return None


def response_latencies(rows, events):
    results = {wheel: [] for wheel in WHEELS}
    times = [finite(row.get("t_s")) for row in rows]
    for event in events:
        if event.get("event") != "segment_start":
            continue
        event_t = finite(event.get("t_s"))
        if event_t is None:
            continue
        start_index = next(
            (index for index, value in enumerate(times) if value is not None and value >= event_t),
            None,
        )
        if start_index is None or start_index == 0:
            continue
        end_t = event_t + min(2.0, finite(event.get("duration_s")) or 2.0)
        for wheel in WHEELS:
            baseline = finite(rows[start_index - 1].get(f"{wheel}_vel_turns_s"))
            baseline_command = finite(
                rows[start_index - 1].get(f"{wheel}_cmd_turns_s")
            )
            target = None
            for index in range(start_index, min(len(rows), start_index + 50)):
                candidate = finite(rows[index].get(f"{wheel}_cmd_turns_s"))
                if (
                    candidate is not None
                    and baseline_command is not None
                    and abs(candidate - baseline_command) >= 0.02
                ):
                    target = candidate
                    break
            if baseline is None or target is None:
                continue
            delta = target - baseline
            if abs(delta) < 0.05:
                continue
            threshold = baseline + 0.2 * delta
            for index in range(start_index, len(rows)):
                t = times[index]
                measured = finite(rows[index].get(f"{wheel}_vel_turns_s"))
                if t is None or t > end_t:
                    break
                crossed = (
                    measured is not None
                    and (
                        (delta > 0.0 and measured >= threshold)
                        or (delta < 0.0 and measured <= threshold)
                    )
                )
                if crossed:
                    results[wheel].append(t - event_t)
                    break
    return results


def build_summary(rows, events, actual_distance, actual_yaw_deg):
    times = [finite(row.get("t_s")) for row in rows]
    duration = (
        times[-1] - times[0]
        if len(times) >= 2 and times[0] is not None and times[-1] is not None
        else 0.0
    )
    wheel_points = relative_trajectory(rows, "wheel")
    ref_points = relative_trajectory(rows, "ref")
    wheel_final = final_point(wheel_points)
    ref_final = final_point(ref_points)
    command_final = integrate_body_commands(rows)

    moving = [
        row
        for row in rows
        if abs(finite(row.get("cmd_v")) or 0.0) > 0.03
        or abs(finite(row.get("cmd_w")) or 0.0) > 0.05
    ]
    velocity_errors = []
    yaw_rate_errors = []
    ref_velocity_errors = []
    ref_yaw_rate_errors = []
    longitudinal_slip = []
    angular_slip = []
    imu_yaw_errors = []

    for row in moving:
        cmd_v = finite(row.get("cmd_v"))
        cmd_w = finite(row.get("cmd_w"))
        wheel_v = finite(row.get("wheel_v"))
        wheel_w = finite(row.get("wheel_w"))
        ref_v = finite(row.get("ref_v"))
        ref_w = finite(row.get("ref_w"))
        imu_wz = finite(row.get("imu_wz"))
        if cmd_v is not None and wheel_v is not None:
            velocity_errors.append(wheel_v - cmd_v)
        if cmd_w is not None and wheel_w is not None:
            yaw_rate_errors.append(wheel_w - cmd_w)
        if ref_v is not None and wheel_v is not None:
            ref_velocity_errors.append(wheel_v - ref_v)
            if abs(wheel_v) > 0.05:
                longitudinal_slip.append((wheel_v - ref_v) / abs(wheel_v))
        if ref_w is not None and wheel_w is not None:
            ref_yaw_rate_errors.append(wheel_w - ref_w)
            if abs(wheel_w) > 0.05:
                angular_slip.append((wheel_w - ref_w) / abs(wheel_w))
        if imu_wz is not None and wheel_w is not None:
            imu_yaw_errors.append(wheel_w - imu_wz)

    disagreements = {}
    per_wheel_tracking = {}
    for wheel in WHEELS:
        errors = []
        for row in moving:
            measured = finite(row.get(f"{wheel}_vel_turns_s"))
            commanded = finite(row.get(f"{wheel}_cmd_turns_s"))
            if measured is not None and commanded is not None:
                errors.append(measured - commanded)
        per_wheel_tracking[wheel] = {
            "rmse_turns_s": rms(errors),
            "max_abs_error_turns_s": max(
                (abs(value) for value in errors),
                default=None,
            ),
        }

    for first, second, label in (
        ("FL", "RL", "left_front_rear"),
        ("FR", "RR", "right_front_rear"),
    ):
        errors = []
        for row in moving:
            a = finite(row.get(f"{first}_vel_turns_s"))
            b = finite(row.get(f"{second}_vel_turns_s"))
            if a is not None and b is not None:
                errors.append(a - b)
        disagreements[label] = {
            "rmse_turns_s": rms(errors),
            "max_abs_turns_s": max((abs(value) for value in errors), default=None),
        }

    telemetry_samples = []
    for row in rows:
        value = finite(row.get("telemetry_sample_s"))
        if value is not None and (
            not telemetry_samples or value != telemetry_samples[-1]
        ):
            telemetry_samples.append(value)
    telemetry_gaps = [
        current - previous
        for previous, current in zip(telemetry_samples, telemetry_samples[1:])
        if current >= previous
    ]

    latencies = response_latencies(rows, events)
    latency_summary = {
        wheel: {
            "samples": len(values),
            "median_s": percentile(values, 0.5),
            "p95_s": percentile(values, 0.95),
            "max_s": max(values) if values else None,
        }
        for wheel, values in latencies.items()
    }

    reference_available = any(
        int(float(row.get("ref_available", 0) or 0)) == 1 for row in rows
    )
    imu_available = any(
        int(float(row.get("imu_available", 0) or 0)) == 1 for row in rows
    )
    health_samples = 0
    unhealthy_samples = 0
    disconnected_samples = {wheel: 0 for wheel in WHEELS}
    error_samples = {wheel: 0 for wheel in WHEELS}
    for row in rows:
        raw_status = row.get("drive_status_json", "")
        if not raw_status:
            continue
        try:
            axes = json.loads(raw_status).get("axes", {})
        except Exception:
            continue
        health_samples += 1
        unhealthy = False
        for wheel in WHEELS:
            status = axes.get(wheel, {})
            if not status.get("connected", False):
                disconnected_samples[wheel] += 1
                unhealthy = True
            if status.get("has_error", False):
                error_samples[wheel] += 1
                unhealthy = True
        unhealthy_samples += int(unhealthy)

    endpoint_error = None
    yaw_error = None
    if wheel_final is not None and ref_final is not None and reference_available:
        endpoint_error = math.hypot(
            wheel_final[0] - ref_final[0],
            wheel_final[1] - ref_final[1],
        )
        yaw_error = wheel_final[2] - ref_final[2]

    measured_distance_error = None
    if actual_distance is not None and wheel_final is not None:
        wheel_distance = math.hypot(wheel_final[0], wheel_final[1])
        measured_distance_error = wheel_distance - actual_distance

    measured_yaw_error = None
    if actual_yaw_deg is not None and wheel_final is not None:
        measured_yaw_error = wheel_final[2] - math.radians(actual_yaw_deg)

    return {
        "schema_version": 1,
        "duration_s": duration,
        "sample_count": len(rows),
        "effective_sample_hz": len(rows) / duration if duration > 0.0 else None,
        "data_sources": {
            "reference_odom": reference_available,
            "imu": imu_available,
            "four_wheel_telemetry": len(telemetry_samples) > 1,
        },
        "command_integral": command_final,
        "wheel_odom": {
            "final_relative_pose": wheel_final,
            "path_length_m": path_length(wheel_points),
            "linear_tracking_rmse_m_s": rms(velocity_errors),
            "angular_tracking_rmse_rad_s": rms(yaw_rate_errors),
        },
        "reference_comparison": {
            "final_relative_pose": ref_final if reference_available else None,
            "path_length_m": (
                path_length(ref_points) if reference_available else None
            ),
            "endpoint_error_m": endpoint_error,
            "yaw_error_rad": yaw_error,
            "linear_velocity_rmse_m_s": rms(ref_velocity_errors),
            "angular_velocity_rmse_rad_s": rms(ref_yaw_rate_errors),
            "mean_longitudinal_slip_ratio": mean(longitudinal_slip),
            "p95_abs_longitudinal_slip_ratio": percentile(
                [abs(value) for value in longitudinal_slip],
                0.95,
            ),
            "mean_angular_slip_ratio": mean(angular_slip),
        },
        "manual_ground_truth": {
            "actual_distance_m": actual_distance,
            "wheel_distance_error_m": measured_distance_error,
            "actual_yaw_deg": actual_yaw_deg,
            "wheel_yaw_error_rad": measured_yaw_error,
        },
        "wheel_pair_disagreement": disagreements,
        "per_wheel_command_tracking": per_wheel_tracking,
        "command_to_wheel_latency": latency_summary,
        "imu_comparison": {
            "wheel_vs_imu_yaw_rate_rmse_rad_s": rms(imu_yaw_errors),
        },
        "usb_telemetry": {
            "unique_samples": len(telemetry_samples),
            "median_period_s": percentile(telemetry_gaps, 0.5),
            "p95_period_s": percentile(telemetry_gaps, 0.95),
            "max_gap_s": max(telemetry_gaps) if telemetry_gaps else None,
            "gaps_over_100ms": sum(gap > 0.1 for gap in telemetry_gaps),
        },
        "drive_health": {
            "status_samples": health_samples,
            "unhealthy_sample_ratio": (
                unhealthy_samples / health_samples if health_samples else None
            ),
            "disconnected_sample_ratio": {
                wheel: (
                    disconnected_samples[wheel] / health_samples
                    if health_samples
                    else None
                )
                for wheel in WHEELS
            },
            "error_sample_ratio": {
                wheel: (
                    error_samples[wheel] / health_samples
                    if health_samples
                    else None
                )
                for wheel in WHEELS
            },
        },
    }


def format_value(value, digits=4):
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def write_markdown(summary, path):
    sources = summary["data_sources"]
    wheel = summary["wheel_odom"]
    reference = summary["reference_comparison"]
    usb = summary["usb_telemetry"]
    health = summary["drive_health"]
    lines = [
        "# Relatório de desempenho 4WD",
        "",
        f"- Duração: {format_value(summary['duration_s'], 2)} s",
        f"- Amostras: {summary['sample_count']}",
        f"- Taxa efetiva: {format_value(summary['effective_sample_hz'], 2)} Hz",
        f"- Referência externa disponível: {sources['reference_odom']}",
        f"- IMU disponível: {sources['imu']}",
        "",
        "## Rastreamento e deslocamento",
        "",
        f"- Caminho wheel odom: {format_value(wheel['path_length_m'])} m",
        f"- RMSE linear comando→wheel: "
        f"{format_value(wheel['linear_tracking_rmse_m_s'])} m/s",
        f"- RMSE angular comando→wheel: "
        f"{format_value(wheel['angular_tracking_rmse_rad_s'])} rad/s",
        f"- Erro final wheel→referência: "
        f"{format_value(reference['endpoint_error_m'])} m",
        f"- Erro final de yaw: {format_value(reference['yaw_error_rad'])} rad",
        f"- Slip longitudinal médio: "
        f"{format_value(reference['mean_longitudinal_slip_ratio'])}",
        f"- Slip longitudinal absoluto P95: "
        f"{format_value(reference['p95_abs_longitudinal_slip_ratio'])}",
        "",
        "## Comunicação e resposta",
        "",
        f"- Período mediano da telemetria: "
        f"{format_value(usb['median_period_s'])} s",
        f"- Maior lacuna USB: {format_value(usb['max_gap_s'])} s",
        f"- Lacunas >100 ms: {usb['gaps_over_100ms']}",
        f"- Fração do ensaio com eixo desconectado/em erro: "
        f"{format_value(health['unhealthy_sample_ratio'])}",
        "",
        "### Rastreamento por roda",
        "",
        "| Roda | RMSE (turn/s) | Erro máximo (turn/s) |",
        "|---|---:|---:|",
    ]
    for wheel_name, metrics in summary["per_wheel_command_tracking"].items():
        lines.append(
            f"| {wheel_name} | {format_value(metrics['rmse_turns_s'])} | "
            f"{format_value(metrics['max_abs_error_turns_s'])} |"
        )
    lines.extend(
        [
            "",
            "### Latência comando→movimento (20% do degrau)",
            "",
            "| Roda | Mediana (s) | P95 (s) | Máxima (s) |",
            "|---|---:|---:|---:|",
        ]
    )
    for wheel_name, metrics in summary["command_to_wheel_latency"].items():
        lines.append(
            f"| {wheel_name} | {format_value(metrics['median_s'])} | "
            f"{format_value(metrics['p95_s'])} | "
            f"{format_value(metrics['max_s'])} |"
        )
    if not sources["reference_odom"]:
        lines.extend(
            [
                "",
                "> Sem odometria externa independente: slip absoluto, erro real "
                "de trajetória e deslocamento real permanecem N/A. Use ZED/SLAM, "
                "mocap, RTK ou informe medidas manuais ao analisador.",
            ]
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def make_plots(rows, run_dir):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False

    times = [finite(row.get("t_s")) for row in rows]
    figure, axes = plt.subplots(3, 1, figsize=(12, 11), sharex=True)
    axes[0].plot(times, [finite(row.get("cmd_v")) for row in rows], label="cmd v")
    axes[0].plot(times, [finite(row.get("wheel_v")) for row in rows], label="wheel v")
    axes[0].plot(times, [finite(row.get("ref_v")) for row in rows], label="ref v")
    axes[0].set_ylabel("m/s")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(times, [finite(row.get("cmd_w")) for row in rows], label="cmd w")
    axes[1].plot(times, [finite(row.get("wheel_w")) for row in rows], label="wheel w")
    axes[1].plot(times, [finite(row.get("ref_w")) for row in rows], label="ref w")
    axes[1].plot(times, [finite(row.get("imu_wz")) for row in rows], label="imu wz")
    axes[1].set_ylabel("rad/s")
    axes[1].legend()
    axes[1].grid(True)

    for wheel in WHEELS:
        axes[2].plot(
            times,
            [finite(row.get(f"{wheel}_vel_turns_s")) for row in rows],
            label=wheel,
        )
    axes[2].set_ylabel("turn/s")
    axes[2].set_xlabel("tempo (s)")
    axes[2].legend()
    axes[2].grid(True)
    figure.tight_layout()
    figure.savefig(run_dir / "timeseries.png", dpi=140)
    plt.close(figure)

    wheel_points = relative_trajectory(rows, "wheel")
    ref_points = relative_trajectory(rows, "ref")
    figure, axis = plt.subplots(figsize=(8, 8))
    wheel_xy = [point for point in wheel_points if point is not None]
    ref_xy = [point for point in ref_points if point is not None]
    if wheel_xy:
        axis.plot(
            [point[0] for point in wheel_xy],
            [point[1] for point in wheel_xy],
            label="wheel odom",
        )
    if ref_xy:
        axis.plot(
            [point[0] for point in ref_xy],
            [point[1] for point in ref_xy],
            label="referência",
        )
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x relativo (m)")
    axis.set_ylabel("y relativo (m)")
    axis.grid(True)
    axis.legend()
    figure.tight_layout()
    figure.savefig(run_dir / "trajectory.png", dpi=140)
    plt.close(figure)
    return True


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--actual-distance-m", type=float)
    parser.add_argument("--actual-yaw-deg", type=float)
    parsed = parser.parse_args(args)

    run_dir = parsed.run_dir.expanduser().resolve()
    samples_path = run_dir / "samples.csv"
    events_path = run_dir / "events.csv"
    if not samples_path.exists():
        raise SystemExit(f"Arquivo não encontrado: {samples_path}")

    with samples_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    events = []
    if events_path.exists():
        with events_path.open(newline="", encoding="utf-8") as handle:
            events = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit("O ensaio não contém amostras.")

    summary = build_summary(
        rows,
        events,
        actual_distance=parsed.actual_distance_m,
        actual_yaw_deg=parsed.actual_yaw_deg,
    )
    summary["plots_generated"] = make_plots(rows, run_dir)
    (run_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    write_markdown(summary, run_dir / "report.md")
    print(f"Relatório: {run_dir / 'report.md'}")
    print(f"Resumo JSON: {run_dir / 'summary.json'}")


if __name__ == "__main__":
    main()
