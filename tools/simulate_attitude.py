import math
import json
import argparse
from pathlib import Path


def rc_to_rate(channel: int) -> float:
    centered = channel - 992
    if centered > 250:
        centered = 250
    elif centered < -250:
        centered = -250
    if -20 < centered < 20:
        centered = 0
    return centered * 0.25


class PID:
    def __init__(self, kp, ki, kd, max_output, max_integral):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.max_output = max_output
        self.max_integral = max_integral

    def update(self, setpoint, measurement, dt_s):
        error = setpoint - measurement
        self.integral += error * dt_s
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))

        derivative = (error - self.prev_error) / (dt_s if dt_s > 0.0 else 0.001)
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(-self.max_output, min(self.max_output, output))
        self.prev_error = error
        return output


class AttitudeSimulator:
    def __init__(self):
        self.roll_pid = PID(0.25, 0.01, 0.002, 500.0, 2000.0)
        self.pitch_pid = PID(0.25, 0.01, 0.002, 500.0, 2000.0)
        self.yaw_pid = PID(0.18, 0.005, 0.001, 300.0, 1500.0)
        self.prev_filtered = [0.0, 0.0, 0.0]

    def apply_notch_filter(self, raw):
        alpha = 0.18
        filtered = []
        for i, value in enumerate(raw):
            self.prev_filtered[i] = self.prev_filtered[i] + alpha * (value - self.prev_filtered[i])
            filtered.append(self.prev_filtered[i])
        return filtered

    def step(self, rc_channels, gyro_rates):
        dt_s = 0.005
        filtered = self.apply_notch_filter(gyro_rates)

        desired_roll = rc_to_rate(rc_channels[0])
        desired_pitch = -rc_to_rate(rc_channels[1])
        desired_yaw = rc_to_rate(rc_channels[3])

        roll_out = self.roll_pid.update(desired_roll, filtered[0], dt_s)
        pitch_out = self.pitch_pid.update(desired_pitch, filtered[1], dt_s)
        yaw_out = self.yaw_pid.update(desired_yaw, filtered[2], dt_s)
        throttle = 0 if rc_channels[2] < 172 else int((rc_channels[2] - 172) * 1000 / 1639)
        enabled = True
        return {
            "roll_out": roll_out,
            "pitch_out": pitch_out,
            "yaw_out": yaw_out,
            "throttle": throttle,
            "enabled": enabled,
            "filtered": filtered,
        }


def load_scenario(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def run_scenario(scenario, steps=100):
    sim = AttitudeSimulator()
    results = []
    for i in range(steps):
        sample = scenario[i % len(scenario)]
        rc_channels = sample["rc_channels"]
        gyro_rates = sample["gyro_rates"]
        result = sim.step(rc_channels, gyro_rates)
        results.append({
            "step": i,
            "rc_channels": rc_channels,
            "gyro_rates": gyro_rates,
            **result,
        })
    return results


def main():
    parser = argparse.ArgumentParser(description="Simula PID y attitude fuera del hardware")
    parser.add_argument("--scenario", default=None, help="Ruta a un JSON con escenarios")
    parser.add_argument("--steps", type=int, default=100, help="Cantidad de pasos a simular")
    parser.add_argument("--json-out", default=None, help="Ruta para guardar resultados en JSON")
    args = parser.parse_args()

    if args.scenario:
        scenario = load_scenario(args.scenario)
    else:
        scenario = [
            {"rc_channels": [992, 992, 1000, 992], "gyro_rates": [0.0, 0.0, 0.0]},
            {"rc_channels": [1200, 992, 1000, 992], "gyro_rates": [0.0, 0.0, 0.0]},
            {"rc_channels": [992, 1200, 1000, 992], "gyro_rates": [0.0, 0.0, 0.0]},
            {"rc_channels": [992, 992, 1000, 992], "gyro_rates": [5.0, -3.0, 2.0]},
            {"rc_channels": [1200, 800, 1000, 1100], "gyro_rates": [3.0, 2.0, -4.0]},
        ]

    results = run_scenario(scenario, args.steps)

    if args.json_out:
        Path(args.json_out).write_text(json.dumps(results, indent=2), encoding="utf-8")

    for item in results[:20]:
        print(f"step={item['step']:02d} roll={item['roll_out']:+7.2f} pitch={item['pitch_out']:+7.2f} yaw={item['yaw_out']:+7.2f} throttle={item['throttle']} filtered={item['filtered']}")

    if len(results) > 20:
        print("...")


if __name__ == "__main__":
    main()
