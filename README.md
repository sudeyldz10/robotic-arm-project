# 4-DOF Cardboard Robot Arm (Arduino Uno, 6× SG90)

A button-controlled robot arm with an automatic pick-and-place task. Servo angles are computed from rotation matrices (Rx/Ry/Rz → atan2).

- **Servos:** base (pin 3), shoulder: 2 mirrored servos (5, 6), elbow (9), gripper: 2 mirrored jaws (10, 11)
- **Buttons:** base 2/4, shoulder 7/8, elbow 12/13, gripper A0/A1, automatic task A2 (INPUT_PULLUP, other leg to GND)
- **Task:** picks up 3 cubes (red, green, blue) and places each on its own target (A, B, C).
- **Serial monitor (9600):** `j <base|shoulder|elbow|grip> <degrees>` · `fk` · `home` · `task` · `stop` · `setstart` (save the current pose as the power-on pose) · `resetstart`

## Folders
- `src/main.cpp` – Arduino / PlatformIO code
- `diagram.json`, `wokwi.toml` – Wokwi circuit (works after a PlatformIO build)
- `simulasyon/robot_kol_simulasyon.html` – 3D simulation that opens in the browser (cubes, targets, automatic task)
- `test_donanim/buton_testi.ino` – small sketch that tests the buttons over the serial monitor (open it in its own folder)
- `karton_sablonlar.pdf` – 1:1 cardboard cutting templates
- `arduino-denemeler/` – earlier experiment recordings

> The position constants (shoulder height, grasp point, cube/target angles) have to be calibrated to your own arm.
