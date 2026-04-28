#!/usr/bin/env python3
"""
Mock Arduino simulator for bridge_hardware testing.

Usage:
  1. Start the simulator:
     python3 tools/mock_arduino.py /dev/pts/1

  2. In another terminal, run the bridge with the simulated port:
     ros2 run bridge_hardware bridge_proteus_node --ros-args \
       -p arduino_port:=/dev/pts/2 \
       -p arduino_baud_rate:=115200

Note: socat creates virtual serial port pairs. Adjust /dev/pts/X paths as needed.
Create the pair with:
  socat -d -d pty,raw,echo=0 pty,raw,echo=0
"""

import serial
import re
import sys
import time

PULSE_WIDTH_MIN = 1400
PULSE_WIDTH_MAX = 1600
PULSE_WIDTH_NEUTRAL = 1500
PINS_COUNT = 6
PINS = [7, 6, 3, 2, 4, 5]

def simulate_arduino(port, baudrate=115200):
    """Simulate Arduino behavior on the given serial port."""
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"[MOCK ARDUINO] Connected on {port} @ {baudrate} baud")
    except Exception as e:
        print(f"[ERROR] Cannot open {port}: {e}")
        sys.exit(1)
    
    # Send initialization message
    time.sleep(1)
    for i in range(PINS_COUNT):
        time.sleep(0.05)
    
    print("[MOCK ARDUINO] Initialization complete, awaiting commands...\n")
    
    # Thruster state tracking
    thruster_state = [PULSE_WIDTH_NEUTRAL] * PINS_COUNT
    
    try:
        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if not line:
                    continue
                
                print(f"[MOCK ARDUINO] Received: {repr(line)}")
                # Parse command format: T<index>:<value>
                match = re.match(r'^T(\d+):(.*)$', line)
                if match:
                    try:
                        thruster_index = int(match.group(1))
                        value = float(match.group(2))
                        
                        # Validate index
                        if thruster_index < 0 or thruster_index >= PINS_COUNT:
                            ser.write(b"Invalid Thruster Index\n")
                            continue
                        
                        # Map value (-1.0 to 1.0) to pulse width (1100-1900)
                        pulse_width = PULSE_WIDTH_NEUTRAL + 400 * value
                        
                        # Clamp to valid range
                        pulse_width = max(PULSE_WIDTH_MIN, min(PULSE_WIDTH_MAX, pulse_width))
                        
                        # Store state
                        thruster_state[thruster_index] = pulse_width
                        
                        print(f"  → Thruster {thruster_index}: value={value:.2f}, pulse={pulse_width:.0f}µs")
                        
                    except ValueError as e:
                        print(f"  → Parse error: {e}")
                else:
                    if line:
                        print(f"  → Unrecognized format")
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\n[MOCK ARDUINO] Shutting down...")
    finally:
        ser.close()
        print("[MOCK ARDUINO] Port closed")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 mock_arduino.py <port> [baudrate]")
        print("Example: python3 mock_arduino.py /dev/pts/1 115200")
        sys.exit(1)
    
    port = sys.argv[1]
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    simulate_arduino(port, baudrate)
