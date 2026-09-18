#!/usr/bin/env python3
"""
calibrate_axis1_only.py

axis0 da ODrive 3678387D3333 ja calibrou com sucesso (perfil direita).
Este script calibra so o axis1 (perfil esquerda) e depois finaliza
(pre_calibrated=True + save) nos dois eixos.
"""

import math
import time
import odrive
from odrive.enums import (
    ENCODER_MODE_HALL,
    MOTOR_TYPE_HIGH_CURRENT,
    CONTROL_MODE_VELOCITY_CONTROL,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_IDLE,
)

SERIAL = "3678387D3333"
POLE_PAIRS = 15
CPR = 90
CALIBRATION_CURRENT = 10
RESISTANCE_CALIB_MAX_VOLTAGE = 7
CURRENT_LIM = 20
VEL_LIMIT = 15.0
VEL_GAIN = 3.0
VEL_INTEGRATOR_GAIN = 1.0
HOVERBOARD_KV = 16.0
TORQUE_CONSTANT = 8.27 / HOVERBOARD_KV


def wait_for_odrive(timeout: float = 30.0):
    print(f"Aguardando ODrive serial={SERIAL}...")
    start = time.time()
    while time.time() - start < timeout:
        try:
            odrv = odrive.find_any(serial_number=SERIAL, timeout=5)
            print(f"  Conectado: serial = {hex(odrv.serial_number)}")
            return odrv
        except Exception:
            pass
    raise RuntimeError("ODrive não encontrado.")


def configure_axis1(ax):
    print("  Configurando axis1 com perfil esquerda...")
    ax.motor.config.pole_pairs = POLE_PAIRS
    ax.motor.config.current_control_bandwidth = 100
    ax.motor.config.torque_constant = TORQUE_CONSTANT
    ax.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    ax.motor.config.calibration_current = CALIBRATION_CURRENT
    ax.motor.config.resistance_calib_max_voltage = RESISTANCE_CALIB_MAX_VOLTAGE
    ax.motor.config.current_lim = CURRENT_LIM
    ax.motor.config.requested_current_range = 25

    ax.encoder.config.mode = ENCODER_MODE_HALL
    ax.encoder.config.cpr = CPR
    ax.encoder.config.calib_scan_distance = 64 * math.pi
    ax.encoder.config.calib_scan_omega = 2 * math.pi
    ax.encoder.config.calib_range = 0.05
    ax.encoder.config.bandwidth = 100

    ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    ax.controller.config.input_mode = 1
    ax.controller.config.vel_limit = VEL_LIMIT
    ax.controller.config.vel_limit_tolerance = 3.0
    ax.controller.config.pos_gain = 1
    ax.controller.config.vel_gain = VEL_GAIN
    ax.controller.config.vel_integrator_gain = VEL_INTEGRATOR_GAIN

    ax.motor.config.pre_calibrated = False
    ax.encoder.config.pre_calibrated = False
    ax.config.startup_encoder_offset_calibration = False
    ax.config.startup_closed_loop_control = False


def dump_errors(ax, label: str):
    print(f"  [{label}] axis.error       = {hex(ax.error)}")
    print(f"  [{label}] motor.error      = {hex(ax.motor.error)}")
    print(f"  [{label}] encoder.error    = {hex(ax.encoder.error)}")
    print(f"  [{label}] controller.error = {hex(ax.controller.error)}")
    print(f"  [{label}] calib_scan_response = {ax.encoder.calib_scan_response}")


def calibrate_axis(ax, label: str, timeout: float = 120.0):
    print(f"  Calibrando {label} (FULL_CALIBRATION_SEQUENCE, timeout={timeout}s)...")
    try:
        ax.clear_errors()
    except Exception:
        pass
    ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    start = time.time()
    started = False
    while time.time() - start <= timeout:
        state = ax.current_state
        if state != AXIS_STATE_IDLE:
            started = True
        elif started:
            break
        if ax.error != 0:
            dump_errors(ax, label)
            raise RuntimeError(f"{label}: erro durante a calibracao.")
        time.sleep(0.1)
    else:
        dump_errors(ax, label)
        raise RuntimeError(f"{label}: timeout na calibracao (started={started}).")

    if ax.error != 0:
        dump_errors(ax, label)
        raise RuntimeError(f"{label}: erro apos calibracao.")

    motor_ok = bool(ax.motor.is_calibrated)
    encoder_ok = bool(ax.encoder.is_ready)
    if not motor_ok or not encoder_ok:
        dump_errors(ax, label)
        raise RuntimeError(
            f"{label}: sequencia terminou, mas resultado invalido "
            f"(motor.is_calibrated={motor_ok}, encoder.is_ready={encoder_ok})."
        )
    print(f"  {label}: calibração OK (motor_ok={motor_ok}, encoder_ok={encoder_ok}).")


def main():
    odrv = wait_for_odrive()

    print("\n[0/3] Confirmando axis0 já calibrado (não mexe nele)...")
    print(f"  axis0: motor.is_calibrated={odrv.axis0.motor.is_calibrated} "
          f"encoder.is_ready={odrv.axis0.encoder.is_ready} error={hex(odrv.axis0.error)}")

    print("\n[1/3] Configurando axis1...")
    configure_axis1(odrv.axis1)
    odrv.save_configuration()
    time.sleep(3)
    odrv = wait_for_odrive()

    print("\n[2/3] Calibrando axis1...")
    calibrate_axis(odrv.axis1, "axis1")

    print("\n[3/3] Marcando os dois eixos como pré-calibrados e salvando...")
    for ax in (odrv.axis0, odrv.axis1):
        ax.motor.config.pre_calibrated = True
        ax.encoder.config.pre_calibrated = True
        ax.config.startup_encoder_offset_calibration = False
        ax.config.startup_closed_loop_control = False
    odrv.save_configuration()
    time.sleep(3)
    odrv = wait_for_odrive()

    print("\nODrive híbrida (3678387D3333) totalmente calibrada.")
    print(f"  axis0: motor.is_calibrated={odrv.axis0.motor.is_calibrated} encoder.is_ready={odrv.axis0.encoder.is_ready}")
    print(f"  axis1: motor.is_calibrated={odrv.axis1.motor.is_calibrated} encoder.is_ready={odrv.axis1.encoder.is_ready}")


if __name__ == "__main__":
    main()
