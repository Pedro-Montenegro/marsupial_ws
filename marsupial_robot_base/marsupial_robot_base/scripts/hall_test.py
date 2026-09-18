import time
from odrive import find_any

odrv0 = find_any()
axis = odrv0.axis0

while True:
    print("----------------------------")
    print(f"Shadow Count : {axis.encoder.shadow_count}")
    print(f"Count in CPR : {axis.encoder.count_in_cpr}")
    print(f"Pos Estimate : {axis.encoder.pos_estimate:.4f}")
    print(f"Vel Estimate : {axis.encoder.vel_estimate:.4f}")
    print(f"Error        : {axis.encoder.error}")
    print()
    time.sleep(0.1)