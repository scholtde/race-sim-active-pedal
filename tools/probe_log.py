import serial, sys, time
s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.2)
time.sleep(2.5); s.reset_input_buffer()
end = time.time() + float(sys.argv[1])
with open("probe_raw.txt", "w") as f:
    while time.time() < end:
        l = s.readline().decode(errors="replace").strip()
        if l: f.write(f"{time.time():.2f},{l}\n"); f.flush()
