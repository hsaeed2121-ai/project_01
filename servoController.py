#!/usr/bin/env python3
"""
--------------------------------------------------------------------------
Robot Hand Desk Accessory
--------------------------------------------------------------------------
License:
Copyright 2021-2024 - Hamza Saeed

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------

robot_hand.py

Controls an InMoov-based robot hand:
 - 5 pots connected to AIN pins (PocketBeagle / BeagleBone style)
 - PCA9685 (I2C) drives servos
 - Reads pots, maps to angles, updates servos if change > tolerance
 - Saves last known pose to a JSON file on exit
"""

import time
import json
import os
from typing import Dict, List

# Hardware libraries (Beagle/PocketBeagle)
try:
    import Adafruit_BBIO.ADC as ADC
except Exception as e:
    raise RuntimeError("Adafruit_BBIO.ADC not available. Run on PocketBeagle/BeagleBone or install Adafruit_BBIO.") from e

try:
    from Adafruit_PCA9685 import PCA9685
except Exception as e:
    raise RuntimeError("Adafruit_PCA9685 not available. Install Adafruit_PCA9685.") from e


# ---------------------------
# Utility functions
# ---------------------------
def clamp(x, a, b):
    return max(a, min(b, x))


def map_range(x, in_min, in_max, out_min, out_max):
    # linear map and clamp
    if in_max == in_min:
        return out_min
    val = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return clamp(val, out_min, out_max)


# ---------------------------
# AnalogIn wrapper
# ---------------------------
class AnalogIn:
    """Simple wrapper around Adafruit_BBIO.ADC to return normalized 0.0-1.0 values."""

    def __init__(self, pin_name: str):
        """
        pin_name example: "P1_19" or "P9_39" depending on your board mapping.
        For PocketBeagle use the AIN pin names you have configured.
        """
        self.pin = pin_name
        ADC.setup()

    def read_value(self) -> float:
        """Return float in range [0.0, 1.0]."""
        # ADC.read returns 0.0-1.0 on Adafruit_BBIO
        raw = ADC.read(self.pin)
        # sometimes ADC.read returns None if not available; guard
        if raw is None:
            return 0.0
        return float(raw)


# ---------------------------
# PCA9685 driver wrapper
# ---------------------------
class PCA9685Driver:
    """
    Wrapper over Adafruit_PCA9685.PCA9685
    Converts angle to pulse (12-bit resolution)
    """

    def __init__(self, bus=1, address=0x40, freq_hz=50):
        """
        bus parameter is unused by this Adafruit_PCA9685 python lib (it uses default i2c).
        address default 0x40.
        """
        self.pwm = PCA9685(address=address, busnum=bus)
        self.set_pwm_freq(freq_hz)
        self.freq = freq_hz
        # Standard constants for mapping
        self._resolution = 4096  # 12-bit

    def set_pwm_freq(self, freq_hz: int):
        self.pwm.set_pwm_freq(freq_hz)
        self.freq = freq_hz

    def angle_to_pwm(self, angle: float, min_pulse: int = 150, max_pulse: int = 600) -> int:
        """
        Convert angle (0..180) to PCA9685 off value (0..4095).
        min_pulse/max_pulse are pulse ticks (0..4095) representing ~1ms..2ms
        Defaults tuned for common servos: 150..600 ticks at 50Hz.
        """
        angle = clamp(angle, 0.0, 180.0)
        # Linear map angle to pulse ticks
        pwm_val = int(map_range(angle, 0.0, 180.0, min_pulse, max_pulse))
        pwm_val = clamp(pwm_val, 0, self._resolution - 1)
        return pwm_val

    def set_servo_angle(self, channel: int, angle: float, min_pulse: int = 150, max_pulse: int = 600):
        pwm_val = self.angle_to_pwm(angle, min_pulse=min_pulse, max_pulse=max_pulse)
        # on=0, off=pwm_val
        self.pwm.set_pwm(channel, 0, pwm_val)


# ---------------------------
# RobotHand main class
# ---------------------------
class RobotHand:
    """
    Main program controlling 5 fingers via pots -> PCA9685 -> servos.
    """

    def __init__(self,
                 pot_pins: List[str],
                 servo_channels: List[int],
                 pose_file: str = "last_pose.json",
                 tolerance_deg: float = 2.0,
                 servo_calibration: Dict[int, Dict] = None):
        """
        pot_pins: list of ADC pin names in same order as fingers [thumb, index, middle, ring, pinky]
        servo_channels: PCA9685 channels in same order [thumb_channel, index_channel, ...]
        tolerance_deg: only update servo when angle change > tolerance
        servo_calibration: optional dict {channel: {"min": int, "max": int}} with min/max PWM ticks
        """
        assert len(pot_pins) == len(servo_channels), "pots/channels length mismatch"
        self.pot_pins = pot_pins
        self.servo_channels = servo_channels
        self.n_fingers = len(pot_pins)
        self.pose_file = pose_file
        self.tolerance = tolerance_deg

        # instantiate hardware
        self.analog_inputs = [AnalogIn(pin) for pin in pot_pins]
        self.servo_driver = PCA9685Driver()
        # default calibration: same for all channels
        default_cal = {"min": 150, "max": 600}
        self.servo_calibration = servo_calibration or {ch: default_cal.copy() for ch in servo_channels}

        # internal state: angles per finger
        # try to load last pose
        self.angles = [90.0] * self.n_fingers  # default neutral
        loaded = self.load_pose()
        if loaded:
            print("Loaded pose from", self.pose_file)
        else:
            print("No previous pose found, using default (90deg)")

        # Setup hardware initial state
        self._setup()

    def _setup(self):
        """Set initial servo PWM frequency and move to last-known pose."""
        # set frequency (already set in driver init, but keep for clarity)
        self.servo_driver.set_pwm_freq(50)
        # move servos to stored angles
        for idx, ch in enumerate(self.servo_channels):
            angle = self.angles[idx]
            calib = self.servo_calibration[ch]
            self.servo_driver.set_servo_angle(ch, angle, min_pulse=calib["min"], max_pulse=calib["max"])
        # short pause to let servos move
        time.sleep(0.3)

    def load_pose(self) -> bool:
        """Load last pose JSON into self.angles if available."""
        if not os.path.isfile(self.pose_file):
            return False
        try:
            with open(self.pose_file, "r") as f:
                data = json.load(f)
            angles = data.get("angles", None)
            if angles and len(angles) == self.n_fingers:
                self.angles = [float(a) for a in angles]
                return True
        except Exception as e:
            print("Failed loading pose:", e)
        return False

    def save_pose(self):
        """Save current pose to JSON."""
        data = {"angles": self.angles, "timestamp": time.time()}
        try:
            with open(self.pose_file, "w") as f:
                json.dump(data, f, indent=2)
            print("Saved pose to", self.pose_file)
        except Exception as e:
            print("Failed saving pose:", e)

    def calibrate_servo(self, channel: int, min_pulse: int, max_pulse: int):
        """Adjust calibration for a single servo channel."""
        if channel not in self.servo_calibration:
            print("Channel not present in calibration map.")
            return
        self.servo_calibration[channel] = {"min": int(min_pulse), "max": int(max_pulse)}
        print(f"Calibrated channel {channel}: min={min_pulse} max={max_pulse}")

    def _map_pot_to_angle(self, pot_value: float) -> float:
        """Map pot [0.0..1.0] to angle [0..180]. Can be adjusted (invert, deadzone, etc.)."""
        adjusted_value = pot_value * (1.7/1.8)
        return float(map_range(adjusted_value, 0.0, 1.0, 0.0, 180.0))

    def run(self, loop_delay: float = 0.02):
        """Main loop: read pots, map to angles, update servo if change > tolerance."""
        print("Starting main loop. Ctrl-C to exit.")
        try:
            while True:
                updated = False
                for i in range(self.n_fingers):
                    pot = self.analog_inputs[i]
                    raw = pot.read_value()
                    angle = self._map_pot_to_angle(raw)

                    # check tolerance
                    if abs(angle - self.angles[i]) >= self.tolerance:
                        self.angles[i] = angle
                        # write servo
                        ch = self.servo_channels[i]
                        calib = self.servo_calibration[ch]
                        self.servo_driver.set_servo_angle(ch, angle, min_pulse=calib["min"], max_pulse=calib["max"])
                        print(f"[DEBUG] Finger {i} -> Channel {ch}, PWM {self.servo_driver.angle_to_pwm(angle, min_pulse=calib['min'], max_pulse=calib['max'])}")
                        updated = True
                if updated:
                    # optionally save continuously or periodically; here we simply update in-memory
                    pass

                time.sleep(loop_delay)
        except KeyboardInterrupt:
            print("\nInterrupted by user. Saving pose and exiting.")
            self.save_pose()
        except Exception as e:
            print("Unexpected exception:", e)
            print("Saving pose before exit.")
            self.save_pose()


# ---------------------------
# Example usage (edit pins/channels as needed)
# ---------------------------
if __name__ == "__main__":
    # Order: [thumb, index, middle, ring, pinky]
    POT_PINS = ["P1_19", "P1_21", "P1_23", "P1_25", "P1_27"]
    SERVO_CHANNELS = [11, 12, 13, 14, 15]  # PCA9685 channels mapped to fingers

    # Optional: per-channel calibration table (pwm ticks)
    # Tune min/max for each servo to match mechanical extremes
    SERVO_CAL = {
        11: {"min": 150, "max": 600},
        12: {"min": 150, "max": 600},
        13: {"min": 150, "max": 600},
        14: {"min": 150, "max": 600},
        15: {"min": 150, "max": 600},
    }

    hand = RobotHand(pot_pins=POT_PINS, servo_channels=SERVO_CHANNELS, pose_file="last_pose.json",
                     tolerance_deg=2.0, servo_calibration=SERVO_CAL)
    hand.run(loop_delay=0.02)