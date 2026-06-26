import odrive
from odrive.enums import *
import time

# Hoverboard Kv
HOVERBOARD_KV = 16.0
    
# Min/Max phase inductance of motor
MIN_PHASE_INDUCTANCE = 0
MAX_PHASE_INDUCTANCE = 0.001

# Min/Max phase resistance of motor
MIN_PHASE_RESISTANCE = 0
MAX_PHASE_RESISTANCE = 0.5

# Tolerance for encoder offset float
ENCODER_OFFSET_FLOAT_TOLERANCE = 0.05

# Timeout handling for reconnection
def wait_for_odrive(timeout=30):
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            odrv0 = odrive.find_any()
            print("ODrive found!")
            return odrv0
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(1)
    print("ODrive not found after waiting")
    return None

# Connect to ODrive
odrv0 = wait_for_odrive()

if odrv0:
    
    left_motor = odrv0.axis1
    right_motor = odrv0.axis0

    calibration_current = 10
    resistance_calib_max_voltage = 7

    current_lim = 20
    odrv0.config.dc_max_negative_current = -24
    odrv0.config.max_regen_current = 24.0
    odrv0.config.dc_bus_overvoltage_trip_level = 42


    left_motor.encoder.config.mode = ENCODER_MODE_HALL
    left_motor.encoder.config.cpr = 90
    left_motor.encoder.config.calib_scan_distance = 150
    left_motor.encoder.config.bandwidth = 100
    left_motor.encoder.config.direction = -1

    left_motor.motor.config.pole_pairs = 15
    left_motor.motor.config.current_control_bandwidth = 100
    left_motor.motor.config.torque_constant = 8.27 / HOVERBOARD_KV
    left_motor.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    left_motor.motor.config.calibration_current = calibration_current
    left_motor.motor.config.resistance_calib_max_voltage = resistance_calib_max_voltage
    left_motor.motor.config.current_lim = current_lim
    left_motor.motor.config.requested_current_range = 25

    left_motor.controller.config.vel_limit_tolerance = 3.0
    left_motor.controller.config.vel_limit = 15
    left_motor.controller.config.pos_gain = 1
    #left_motor.controller.config.vel_gain = 0.035 * left_motor.motor.config.torque_constant * left_motor.encoder.config.cpr
    #left_motor.controller.config.vel_integrator_gain = 0.06 * left_motor.motor.config.torque_constant * left_motor.encoder.config.cpr
    left_motor.controller.config.vel_gain = 3.0
    left_motor.controller.config.vel_integrator_gain = 1.0
    #left_motor.controller.config.vel_gain = 5.8
    #left_motor.controller.config.vel_integrator_gain = 6.4
    left_motor.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    left_motor.controller.config.input_mode = 1
    left_motor.config.can.node_id = 1

    
    # odrv0.axis0.config.can.node_id = 31
    # odrv0.axis1.config.can.node_id = 4
    
    right_motor.encoder.config.mode = ENCODER_MODE_HALL
    right_motor.encoder.config.cpr = 90
    right_motor.encoder.config.calib_scan_distance = 150
    right_motor.encoder.config.bandwidth = 100
    right_motor.encoder.config.direction = 1

    right_motor.motor.config.pole_pairs = 15
    right_motor.motor.config.current_control_bandwidth = 100
    right_motor.motor.config.torque_constant = 8.27 / HOVERBOARD_KV # 0.516875
    right_motor.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    right_motor.motor.config.calibration_current = calibration_current
    right_motor.motor.config.resistance_calib_max_voltage = resistance_calib_max_voltage
    right_motor.motor.config.current_lim = current_lim
    right_motor.motor.config.requested_current_range = 25

    right_motor.controller.config.vel_limit_tolerance = 3.0
    right_motor.controller.config.vel_limit = 10.0
    right_motor.controller.config.pos_gain = 1
    #right_motor.controller.config.vel_gain = 0.035 * right_motor.motor.config.torque_constant * right_motor.encoder.config.cpr
    #right_motor.controller.config.vel_integrator_gain = 0.06 * right_motor.motor.config.torque_constant * right_motor.encoder.config.cpr
    right_motor.controller.config.vel_gain = 3.0
    right_motor.controller.config.vel_integrator_gain = 1.0
    #right_motor.controller.config.vel_gain = 5.8
    #right_motor.controller.config.vel_integrator_gain = 6.4
    right_motor.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    right_motor.controller.config.input_mode = 1
    right_motor.config.can.node_id = 0



    try:
        # Save and Reboot
        print("Saving configuration and rebooting ODrive...")
        odrv0.save_configuration()
        print("Configuration saved successfully. Rebooting...")
        wait_for_odrive()
        
    except Exception as e:
        print(f"Error during configuration save: {e}")
else:
    print("Failed to connect to ODrive.")


"""
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

dump_errors(odrv0)

odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.config.startup_encoder_offset_calibration = False
odrv0.axis0.config.startup_closed_loop_control = False

odrv0.axis1.motor.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True
odrv0.axis1.config.startup_encoder_offset_calibration = False
odrv0.axis1.config.startup_closed_loop_control = False

odrv0.save_configuration()

odrv0.axis1.controller.input_vel = 3

odrv0.vbus_voltage # Verifica velocidade

# Configurar portas - caso de problema
odrv0.config.enable_uart_a = True
odrv0.config.gpio1_mode = GPIO_MODE_UART_A
odrv0.config.gpio2_mode = GPIO_MODE_UART_A

Roda Direita:
In [1]: odrv0.serial_number
Out[1]: 315F32623431

Roda Esquerda:
In [2]: odrv1.serial_number
Out[2]: 316332743431

"""

"""
odrv0.axis0.config.can.node_id = 0
odrv0.axis1.config.can.node_id = 1
odrv0.axis0.controller.config.vel_integrator_gain = 3.2
odrv0.axis1.controller.config.vel_integrator_gain = 3.2
odrv0.axis0.controller.config.vel_gain = 2.5
odrv0.axis1.controller.config.vel_gain = 2.5


odrv0.axis0.controller.config.vel_integrator_gain = 2.791125
odrv0.axis1.controller.config.vel_integrator_gain = 2.791125
odrv0.axis0.controller.config.vel_gain = 1.62815625
odrv0.axis1.controller.config.vel_gain = 1.62815625

odrv0.axis0.controller.input_vel = 0.3
odrv0.axis1.controller.input_vel = 0.3

odrv0.axis0.controller.input_vel = 0
odrv0.axis1.controller.input_vel = 0


"""