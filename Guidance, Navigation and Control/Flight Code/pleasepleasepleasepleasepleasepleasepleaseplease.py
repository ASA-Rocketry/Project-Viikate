import asyncio
import struct
from bleak import BleakClient, BleakScanner
from datetime import datetime

# --- Config ---
DEVICE_ADDRESS = "00:15:83:00:6E:F7"  # replace with your HC-06 address
CHAR_UUID = "B8CC4750-0C67-86C5-DF99-30F17A288FB8"  
# --- Packet format ---
PACKET_FORMAT = "<5fI"
PACKET_SIZE   = struct.calcsize(PACKET_FORMAT)  # 24 bytes

buffer = bytearray()

def parse_packet(raw_bytes):
    if len(raw_bytes) != PACKET_SIZE:
        return None
    try:
        unpacked = struct.unpack(PACKET_FORMAT, raw_bytes)
        return {
            "altitude":         unpacked[0],
            "verticalVelocity": unpacked[1],
            "accZ":             unpacked[2],
            "rotZ":             unpacked[3],
            "error":            unpacked[4],
            "timestamp":        unpacked[5],
        }
    except struct.error:
        return None

def print_data(data):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(
        f"[{ts}] "
        f"Alt: {data['altitude']:.3f}m  "
        f"VVel: {data['verticalVelocity']:.3f}m/s  "
        f"AccZ: {data['accZ']:.3f}  "
        f"RotZ: {data['rotZ']:.3f}  "
        f"Err: {data['error']:.3f}  "
        f"T: {data['timestamp']}ms"
    )

def notification_handler(sender, raw):
    global buffer
    buffer.extend(raw)

    while len(buffer) >= PACKET_SIZE:
        packet = parse_packet(bytes(buffer[:PACKET_SIZE]))
        if packet:
            print_data(packet)
            buffer = buffer[PACKET_SIZE:]
        else:
            print("Sync error — resyncing...")
            buffer = buffer[1:]

async def main():

    devices = await BleakScanner.discover()
    target = None
    for device in devices:
        print(f"{device.name} - {device.address}")
        if device.name == "=flightcontroller":  # or whatever your device name is
            target = device
            break

    print(type(target.address))

    print(f"Connecting to {target.address}...")
    async with BleakClient(target.address) as client:
        print(f"Connected!")
        await client.start_notify(str(target.address), notification_handler)
        print("Receiving data — press Ctrl+C to stop\n")
        while True:
            await asyncio.sleep(0.1)

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("\nStopped")