import asyncio
import threading
from bleak import BleakClient, BleakScanner
from KeyboardControl2 import start_listener, payload_dict, serialize

TEST = False
DEVICE_CONNECTED = TEST
RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"


async def print_loop(stop_event: asyncio.Event):
    while not stop_event.is_set():
        printString = getUIString(payload_dict["mode"], payload_dict["direction"])
        print(printString, end="\r")
        await asyncio.sleep(0.1)


def getUIString(mode, direction):
    if mode == 0:
        direction_str = "Stopped"
    else:
        match direction:
            case 0:
                direction_str = "Backward / Negative"
            case 1:
                direction_str = "Forward / Positive"
            case _:
                direction_str = "ERROR"

    match mode:
        case 0:
            mode_str = "Idle"
        case 1:
            mode_str = "Elbow Flexion/Extension"
        case 2:
            mode_str = "Elbow Pronation/Supination"
        case 3:
            mode_str = "Wrist Flexion/Extension"
        case 4:
            mode_str = "Wrist Pronation/Supination"
        case _:
            mode_str = "ERROR"

    ui_string = "\n" * 50
    ui_string += """
    ==========================================================
                    ROBOTIC ARM INTERFACE v1.0                 
    ==========================================================

    [ INSTRUCTIONS ]
    TOGGLE MODE ON/OFF:
        - [1]: Elbow Flexion/Extension
        - [2]: Elbow Pronation/Supination
        - [3]: Wrist Flexion/Extension
        - [4]: Wrist Pronation/Supination

    MOVEMENT: 
        - HOLD [UP ARROW]    : Forward / Positive Direction
        - HOLD [DOWN ARROW]  : Backward / Negative Direction

    EXIT:
        - [ESC]              : Emergency Stop & Close

    ----------------------------------------------------------
    [ CURRENT STATE ]
    """
    if DEVICE_CONNECTED:
        ui_string += f"""
        MODE        : {mode_str}
        DIRECTION   : {direction_str}
        ----------------------------------------------------------
        """
    else:
        ui_string += "    CONNECT DEVICE TO BLUETOOTH\n    ----------------------------------------------------------\n"
    return ui_string


async def main():
    global DEVICE_CONNECTED
    loop = asyncio.get_event_loop()
    stop_event = asyncio.Event()
    disconnect_event = asyncio.Event()
    active_client: BleakClient | None = None

    def on_disconnect(client: BleakClient):
        print("\nDevice disconnected")
        loop.call_soon_threadsafe(disconnect_event.set)

    async def send_emergency(payload: str):
        if active_client and active_client.is_connected:
            try:
                await active_client.write_gatt_char(RX_UUID, payload.encode())
            except Exception as e:
                print(f"\nEmergency send failed: {e}")

    def emergency_callback(payload: str):
        asyncio.run_coroutine_threadsafe(send_emergency(payload), loop)

    threading.Thread(
        target=start_listener, args=(loop, stop_event, emergency_callback), daemon=True
    ).start()

    if TEST:
        asyncio.create_task(print_loop(stop_event))

    while not stop_event.is_set():
        try:
            disconnect_event.clear()
            print("\nScanning...")
            devices = await BleakScanner.discover()
            target = next((d for d in devices if d.name == "Nano_ESP32"), None)

            if target is None:
                print("Device not found, retrying...")
                await asyncio.sleep(2)
                continue

            async with BleakClient(
                target.address, disconnected_callback=on_disconnect
            ) as client:
                active_client = client
                DEVICE_CONNECTED = True
                print("Connected")
                asyncio.create_task(print_loop(stop_event))
                while not disconnect_event.is_set() and not stop_event.is_set():
                    await client.write_gatt_char(
                        RX_UUID, serialize(payload_dict).encode()
                    )
                    await asyncio.sleep(0.1)
                active_client = None
                if not TEST:
                    DEVICE_CONNECTED = False

        except Exception as e:
            print(f"Error: {e}, reconnecting...")
            active_client = None
            await asyncio.sleep(2)

    print("\nStopped")


asyncio.run(main())

