import serial
import time

class ESPManager:
    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self._last_state = None

    def start(self):
        print(f"Connecting to {self.port} at {self.baud} baud...")
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(2)  # Give time for ESP32 to boot
        print("Connection established.")

    def update(self):
        """Must be called regularly from the main loop to read data from the ESP."""
        if self.ser and self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"[ESP] {line}")
                if "ACTIVATED" in line:
                    self._last_state = "ON"
                elif "DEACTIVATED" in line:
                    self._last_state = "OFF"

    def send_command(self, cmd: str):
        if not self.ser or not self.ser.is_open:
            raise Exception("Serial port not open")
        print(f"Sending: {cmd}")
        self.ser.write((cmd + "\n").encode())

    def turn_on(self):
        self.send_command("$on")

    def turn_off(self):
        self.send_command("$off")

    def get_state(self):
        return self._last_state

    def close(self):
        print("Closing connection with ESP...")
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("Connection closed.")
