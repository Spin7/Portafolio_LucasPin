# 6_Gripper_3D_Model — 3D-Printable Gripper

CAD and STL files for the custom servo-driven gripper (end-effector) mounted on the R1 robot.

## Contents

| Folder | Description |
|---|---|
| `Editable_3D_Model/gripper.zip` | Editable source model (compressed) |
| `Editable_3D_Model/grass-gripper-1.snapshot.2/` | CAD project snapshot folder |
| `For_Printing/` | Ready-to-slice STL files |

### STL files in `For_Printing/`

| File | Part description |
|---|---|
| `Ban Xoay-1.STL` | Rotating base |
| `Ga Tay Kep Duoi-1.STL` | Lower jaw |
| `Ga Tay Kep Tren Nua-1.STL` | Upper jaw (half) |
| `Ga Tay Kep Tren-1.STL` | Upper jaw (full) |
| `Tru Dong 28-1.STL` | 28 mm drive rod |
| `Tru Dong 50-1.STL` | 50 mm drive rod |
| `servo horn-1.STL` | Servo horn adapter |
| `Tay Kep.STL` | Gripper arm |

## Gripper Operation

- Actuated by a **MG995 servo** mounted on the Z-axis head
- Controlled from the PC via the **Arduino MEGA** (serial at 115200 baud):
  - `$on`  → servo rotates to **20°** → **closes** (grips egg)
  - `$off` → servo rotates to **90°** → **opens** (releases egg)
- Firmware: `3_Code/2_Arduino_Cpp/2_Secondary_Arduino_Gripper_Control/Serial_Control_Servo_Gripper/Serial_Control_Servo_Gripper.ino`

← [Back to root](../README.md)
