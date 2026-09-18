#!/usr/bin/env python3
"""
calibrate_right_odrive.py

Configura e calibra um ODrive do robô 4WD do zero.

Uso:
    python3 calibrate_right_odrive.py [serial] [direction]

    serial    : hex sem 0x (ex: 3553346E3034). Vazio = primeiro encontrado.
    direction : 1 (padrão, lado direito) ou -1 (lado esquerdo)

Exemplos:
    python3 calibrate_right_odrive.py 3678387D3333 -1   # esquerdo
    python3 calibrate_right_odrive.py 3553346E3034 1    # direito
"""

import sys
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

# ── Parâmetros do motor (hoverboard) ─────────────────────────────────────────
HOVERBOARD_KV                = 16.0
# Os quatro motores de hoverboard usam Hall com 15 pares de polos e 90 CPR.
# Uma resposta de varredura 96 em vez de 144 NAO identifica 10 pares: como
# CPR = 6 * pole_pairs, a resposta esperada da calibracao independe do numero
# de pares quando os dois valores mudam juntos. Os seis estados Hall da ODrive
# direita foram observados em 2026-07-22 na sequencia valida 1,3,2,6,4,5.
LEFT_POLE_PAIRS              = 15
RIGHT_POLE_PAIRS             = 15
CALIBRATION_CURRENT          = 10
RESISTANCE_CALIB_MAX_VOLTAGE = 7
CURRENT_LIM                  = 20
VEL_LIMIT                    = 15.0
VEL_GAIN                     = 1.0
VEL_INTEGRATOR_GAIN          = 0.5
TORQUE_CONSTANT              = 8.27 / HOVERBOARD_KV


def wait_for_odrive(serial: str = "", timeout: float = 30.0):
    print(f"Aguardando ODrive{'  serial=' + serial if serial else ''}...")
    kw = {"serial_number": serial} if serial else {}
    start = time.time()
    while time.time() - start < timeout:
        try:
            odrv = odrive.find_any(timeout=5, **kw)
            print(f"  Conectado: serial = {hex(odrv.serial_number)}")
            return odrv
        except Exception:
            pass
    raise RuntimeError("ODrive não encontrado.")


def configure_axis(ax, label: str, direction: int):
    pole_pairs = LEFT_POLE_PAIRS if direction == -1 else RIGHT_POLE_PAIRS
    encoder_cpr = 6 * pole_pairs
    print(
        f"  Configurando {label} (direction={direction}, "
        f"pole_pairs={pole_pairs}, cpr={encoder_cpr})..."
    )

    # pole_pairs deve ser setado ANTES do cpr — ODrive 0.5.1 valida cpr == 6*pole_pairs na hora
    ax.motor.config.pole_pairs               = pole_pairs
    ax.motor.config.current_control_bandwidth = 100
    ax.motor.config.torque_constant          = TORQUE_CONSTANT
    ax.motor.config.motor_type               = MOTOR_TYPE_HIGH_CURRENT
    ax.motor.config.calibration_current      = CALIBRATION_CURRENT
    ax.motor.config.resistance_calib_max_voltage = RESISTANCE_CALIB_MAX_VOLTAGE
    ax.motor.config.current_lim              = CURRENT_LIM
    ax.motor.config.requested_current_range  = 25

    ax.encoder.config.mode               = ENCODER_MODE_HALL
    ax.encoder.config.cpr                = encoder_cpr
    # Perfis de varredura distintos por placa (ver ODRIVE_CALIBRATION_LOG_2026-07-22.md,
    # recalibração 2026-07-23): a esquerda (3678387D3333) NAO passa com o perfil
    # da direita apos ter a config apagada — precisa de 64*pi / 2*pi / range 0.05.
    if direction == -1:
        ax.encoder.config.calib_scan_distance = 64 * math.pi
        ax.encoder.config.calib_scan_omega    = 2 * math.pi
        ax.encoder.config.calib_range         = 0.05
    else:
        ax.encoder.config.calib_scan_distance = 48 * math.pi
        ax.encoder.config.calib_scan_omega    = math.pi
    ax.encoder.config.bandwidth          = 100
    try:
        ax.encoder.config.direction = direction
    except AttributeError:
        print(f"    AVISO: encoder.config.direction não disponível neste firmware — usando default.")

    ax.controller.config.control_mode        = CONTROL_MODE_VELOCITY_CONTROL
    ax.controller.config.input_mode          = 1  # INPUT_MODE_VEL_RAMP
    ax.controller.config.vel_limit           = VEL_LIMIT
    ax.controller.config.vel_limit_tolerance = 3.0
    ax.controller.config.pos_gain            = 1
    ax.controller.config.vel_gain            = VEL_GAIN
    ax.controller.config.vel_integrator_gain = VEL_INTEGRATOR_GAIN

    # Garante que esta execucao realmente refaca toda a calibracao. Esses
    # flags podem ter ficado gravados por uma tentativa anterior incompleta.
    ax.motor.config.pre_calibrated = False
    ax.encoder.config.pre_calibrated = False
    ax.config.startup_encoder_offset_calibration = False
    ax.config.startup_closed_loop_control = False


def dump_errors(ax, label: str):
    print(f"  [{label}] axis.error    = {hex(ax.error)}")
    try:
        print(f"  [{label}] motor.error   = {hex(ax.motor.error)}")
    except Exception:
        pass
    try:
        print(f"  [{label}] encoder.error = {hex(ax.encoder.error)}")
    except Exception:
        pass
    try:
        print(f"  [{label}] controller.error = {hex(ax.controller.error)}")
    except Exception:
        pass


def calibrate_axis(ax, label: str, timeout: float = 90.0):
    print(f"  Calibrando {label} (FULL_CALIBRATION_SEQUENCE)...")
    try:
        ax.clear_errors()
    except Exception:
        pass
    ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    # A mudanca de requested_state nao e sincrona. Se consultarmos o estado
    # logo em seguida ele ainda pode ser IDLE, causando um falso "OK" sem que
    # a calibracao sequer tenha iniciado.
    start = time.time()
    started = False
    while time.time() - start <= timeout:
        state = ax.current_state
        if state != AXIS_STATE_IDLE:
            started = True
        elif started:
            break

        # Um erro pode devolver o eixo para IDLE muito rapidamente. Nao espere
        # o timeout para mostrar o diagnostico.
        if ax.error != 0:
            dump_errors(ax, label)
            raise RuntimeError(f"{label}: erro durante a calibracao.")
        time.sleep(0.1)
    else:
        if not started:
            dump_errors(ax, label)
            raise RuntimeError(
                f"{label}: o ODrive nao iniciou a calibracao solicitada."
            )
        dump_errors(ax, label)
        raise RuntimeError(f"{label}: timeout na calibracao.")

    if not started:
        dump_errors(ax, label)
        raise RuntimeError(f"{label}: calibracao nao iniciou.")

    if ax.error != 0:
        dump_errors(ax, label)
        raise RuntimeError(f"{label}: erro apos calibracao (veja sub-erros acima).")

    motor_ok = bool(ax.motor.is_calibrated)
    encoder_ok = bool(ax.encoder.is_ready)
    if not motor_ok or not encoder_ok:
        dump_errors(ax, label)
        raise RuntimeError(
            f"{label}: sequencia terminou, mas resultado invalido "
            f"(motor.is_calibrated={motor_ok}, encoder.is_ready={encoder_ok})."
        )
    print(f"  {label}: calibração OK.")


def main():
    serial    = sys.argv[1] if len(sys.argv) > 1 else ""
    direction = int(sys.argv[2]) if len(sys.argv) > 2 else 1

    print(f"direction={direction}  ({'esquerdo' if direction == -1 else 'direito'})")

    odrv = wait_for_odrive(serial)

    print("\n[1/4] Configurando parâmetros...")
    odrv.config.dc_max_negative_current       = -24
    odrv.config.max_regen_current             = 24.0
    odrv.config.dc_bus_overvoltage_trip_level = 42

    configure_axis(odrv.axis0, "axis0", direction)
    configure_axis(odrv.axis1, "axis1", direction)

    print("\n[2/4] Salvando config antes da calibração...")
    odrv.save_configuration()
    time.sleep(3)
    odrv = wait_for_odrive(serial)

    # save_configuration() nem sempre reboota este firmware/placa de forma
    # confiavel (ver ODRIVE_CPR_POLEPAIRS_MISMATCH.md) — sem este reboot()
    # explicito o axis0 volta a falhar com ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH
    # mesmo com o perfil de calib_scan_distance correto.
    print("  Reboot explícito para limpar estado latched...")
    try:
        odrv.reboot()
    except Exception:
        pass
    time.sleep(6)
    odrv = wait_for_odrive(serial)

    print("\n[3/4] Rodando calibração...")
    calibrate_axis(odrv.axis0, "axis0")
    calibrate_axis(odrv.axis1, "axis1")

    print("\n[4/4] Marcando como pré-calibrado e salvando...")
    for ax in (odrv.axis0, odrv.axis1):
        ax.motor.config.pre_calibrated              = True
        ax.encoder.config.pre_calibrated            = True
        # O offset ja foi obtido e validado acima. Recalibrar a cada boot pode
        # falhar com o robo apoiado no chao e invalidar somente um dos lados.
        ax.config.startup_encoder_offset_calibration = False
        ax.config.startup_closed_loop_control        = False

    odrv.save_configuration()
    time.sleep(3)
    odrv = wait_for_odrive(serial)

    serial_hex = hex(odrv.serial_number).replace("0x", "").upper()
    side = "esquerdo" if direction == -1 else "direito"
    param = "left_odrive_serial" if direction == -1 else "right_odrive_serial"
    print(f"""
╔══════════════════════════════════════════════════════╗
  ODrive {side} configurado com sucesso!

  Serial number: {serial_hex}

  Confirme no robot_4wd.yaml:
    {param}: "{serial_hex}"
    right_disabled: false
╚══════════════════════════════════════════════════════╝
""")


if __name__ == "__main__":
    main()
