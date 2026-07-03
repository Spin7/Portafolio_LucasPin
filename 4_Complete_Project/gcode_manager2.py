import serial 
import time
import re

class GcodeManager:
    def __init__(self, port, baud=115200, scale=[130.5, 134.8, 132]):
        self.scale = [scale[0], scale[1], scale[2]]
        self.port = port
        self.baud = baud
        self.ser = None
        self._error = None
        self.current_state = "Unknown"
        self.current_position = (0, 0, 0)
        self._last_position = (0, 0, 0)
        self._moving = False
        self._last_query_time = 0
        self.serial_lock = None  # No longer using lock since there are no threads

    def start(self):
        print(f"Connecting to port {self.port} at {self.baud} baud...")
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)  # Initial wait for safety

            print("Waiting for GRBL initialization...")
            start_time = time.time()

            while True:
                if time.time() - start_time > 5:
                    raise TimeoutError("Did not receive exact GRBL welcome message.")
                raw = self.ser.readline()
                if raw:
                    try:
                        decoded = raw.decode('latin-1').rstrip()
                    except:
                        decoded = raw.decode('latin-1', errors='ignore').rstrip()
                    print(f"Received: {decoded}")
                    if decoded == "Grbl 0.9j ['$' for help]":
                        print("GRBL ready.")
                        self.current_position = (0.0, 0.0, 0.0)
                        break
        except Exception as e:
            print(f"Unexpected error while connecting: {e}")
            raise

    def send_command(self, cmd, wait_for_ok=True):
        line = (cmd + '\n').encode('ascii')
        self.ser.write(line)

        if wait_for_ok:
            start_time = time.time()
            while True:
                if time.time() - start_time > 3:
                    raise TimeoutError(f"Did not receive 'ok' after sending: {cmd}")
                raw = self.ser.readline()
                if raw:
                    try:
                        decoded = raw.decode('latin-1').rstrip()
                    except:
                        decoded = raw.decode('latin-1', errors='ignore').rstrip()
                    if decoded.strip().lower() == "ok":
                        break
                    elif "error" in decoded.lower():
                        raise Exception(f"Error sending command: {cmd} => {decoded}")

    def calibrate(self):
        print("CALIBRATION: Setting steps per mm for each axis...")
        self.send_command("$100=7000")
        self.send_command("$101=7000")
        self.send_command("$102=7000")

    def takeEgg(self, activate=True):
        action = "takingEgg" if activate else "freeEgg"
        print(f"{action}...")
        angle = 90 if activate else 0
        self.send_command(f"M280 P0 S{angle}")

    def isMoving(self):
        return self._moving

    def move(self, x_raw, y_raw):
        x_scaled = round(x_raw / self.scale[0], 4)
        y_scaled = round(y_raw / self.scale[1], 4)
        print(f"Moving to position in mm X={x_raw:.2f}, Y={y_raw:.2f}")
        self._moving = True
        self.send_command(f"G21 G90 G1 X{x_scaled} Y{y_scaled} F10")
    
    def move_cm(self, x_raw, y_raw):
        self.move(x_raw*10, y_raw*10)
    
    def moveZ(self, z_raw):
        z_scaled = round(z_raw / self.scale[2], 4)
        print(f"Moving to position in mm Z={z_raw:.2f}")
        self._moving = True
        print(z_scaled) #/////////////////////////////////////////////////////////////////////////////////////////////////
        self.send_command(f"G21 G90 G1 Z{z_scaled} F10")
    
    def moveZ_cm(self, z_raw):
        self.moveZ(z_raw*10)

    def arc(self, center, endpoint, v, a):
        print(f"Executing arc to {endpoint} with center {center}...")
        cx, cy = center
        x, y = endpoint
        self.send_command(f"G2 X{x:.3f} Y{y:.3f} I{cx:.3f} J{cy:.3f} F{int(v)}")

    def update(self):
        try:
            current_time = time.time()
            if current_time - self._last_query_time >= 0.2:  # Every 200 ms
                self.ser.write(b"?\n")
                self._last_query_time = current_time

            if self.ser.in_waiting > 0:
                raw = self.ser.readline()
            else:
                return  # Nothing to do if no data

            if raw:
                try:
                    decoded = raw.decode('latin-1').rstrip()
                except:
                    decoded = raw.decode('latin-1', errors='ignore').rstrip()

                if decoded.startswith('<') and 'MPos' in decoded:
                    self._parse_position(decoded)
                elif 'ok' in decoded.lower():
                    # Could update something here if needed
                    pass
                elif 'error' in decoded.lower():
                    print(f"Unexpected error: {decoded}")
                    self._error = 'error'

        except Exception as e:
            print(f"Error reading serial: {e}")

    def _parse_position(self, line):
        try:
            line = line.strip('<>')

            match = re.match(r'(\w+),', line)
            if match:
                self.current_state = match.group(1)

            mpos_match = re.search(r'MPos:([-\d.]+),([-\d.]+),([-\d.]+)', line)
            if mpos_match:
                x, y, z = map(float, mpos_match.groups())
                self.current_position = (x, y, z)

            self._moving = self.current_state != "Idle"

            # Show position only if it changed
            if self.current_position != self._last_position:
                print(f"State: {self.current_state}, Position in mm: ({self.current_position[0]*self.scale[0]:.2f}, {self.current_position[1]*self.scale[1]:.2f}, {self.current_position[2]*self.scale[2]:.2f})")
                self._last_position = self.current_position

        except Exception as e:
            print(f"Error parsing position: {e}")

    def stop(self):
        print("Robot stopped. Sending feed hold and pause...")
        self.send_command("!")
        self.send_command("M0")

    def close(self):
        print("Closing serial connection...")
        if self.ser:
            self.ser.close()
        print("Connection closed.")

    def get_status(self):
        return self.current_state

    def get_position(self):
        return self.current_position
    
    def get_mm_position(self):
        return self.current_position[0]*self.scale[0], self.current_position[1]*self.scale[1]
