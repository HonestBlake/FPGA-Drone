"""View Basys3 8x8 lidar depth packets (millimetres). Requires pyserial.
python depth_viewer.py --port COM4 [--headless] [--frames 5] [--csv scans.csv]
"""
import argparse
import csv
import struct
import time


class Decoder:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(b"L8")
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer.endswith(b"L") else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 136:
                break
            packet = self.buffer[:136]
            if sum(packet[:134]) & 65535 != struct.unpack_from("<H", packet, 134)[0]:
                del self.buffer[0]
                continue
            frames.append((struct.unpack_from("<I", packet, 2)[0],
                           struct.unpack_from("<64H", packet, 6)))
            del self.buffer[:136]
        return frames


def self_test():
    body = b"L8" + struct.pack("<I64H", 7, *range(1000, 1064))
    packet = body + struct.pack("<H", sum(body) & 65535)
    d = Decoder()
    assert not d.feed(b"garbageL")
    assert not d.feed(packet[:31])
    assert d.feed(packet[31:]) == [(7, tuple(range(1000, 1064)))]
    bad = bytearray(packet)
    bad[20] ^= 1
    assert d.feed(bad + packet) == [(7, tuple(range(1000, 1064)))]
    assert d.feed(packet + packet) == [(7, tuple(range(1000, 1064)))] * 2
    print("PASS: depth parser chunking, resynchronization, checksum and pixel ordering")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--csv")
    parser.add_argument("--timeout", type=float, default=20, help="seconds without a frame before exiting")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    import serial
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not args.port:
        for port in ports:
            print(port.device, port.description)
        parser.error("Choose the Basys3 serial port with --port COMx")
    root = None
    if not args.headless:
        import tkinter as tk
        root = tk.Tk()
        root.title("Basys3 lidar: 8x8 depth (mm), sensor row order")
        canvas = tk.Canvas(root, width=640, height=680)
        canvas.pack()
    output = open(args.csv, "w", newline="") if args.csv else None
    writer = csv.writer(output) if output else None
    if writer:
        writer.writerow(["time", "frame"] + [f"r{r}c{c}" for r in range(8) for c in range(8)])
    decoder = Decoder()
    count = 0
    last = time.monotonic()
    try:
        with serial.Serial(args.port, 115200, timeout=0.05) as connection:
            while not args.frames or count < args.frames:
                for sequence, pixels in decoder.feed(connection.read(4096)):
                    last = time.monotonic()
                    count += 1
                    print(f"Frame {sequence}: distances in mm")
                    for row in range(8):
                        print(" ".join(f"{v:5}" for v in pixels[row*8:row*8+8]))
                    if writer:
                        writer.writerow([time.time(), sequence, *pixels])
                        output.flush()
                    if root:
                        canvas.delete("all")
                        for i, value in enumerate(pixels):
                            x, y = (i % 8)*80, (i // 8)*80
                            shade = max(0, min(255, int(value*255/4000)))
                            color = f"#{255-shade:02x}40{shade:02x}" if value < 4000 else "#666666"
                            canvas.create_rectangle(x, y, x+80, y+80, fill=color)
                            canvas.create_text(x+40, y+40, text=str(value), fill="white")
                        canvas.create_text(320, 660, text=f"Frame {sequence}; red near / blue far; gray >=4000 (may be invalid)")
                if root:
                    root.update()
                if time.monotonic()-last > args.timeout:
                    raise TimeoutError("No depth frames: check LD9, UART mode, sensor power/common ground and JC7/JC8")
    except KeyboardInterrupt:
        pass
    finally:
        if output:
            output.close()
        if root:
            root.destroy()


if __name__ == "__main__":
    main()
