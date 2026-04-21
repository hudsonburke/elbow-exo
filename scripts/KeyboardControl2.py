from pynput import keyboard
from asyncio import AbstractEventLoop, Event
from typing import Callable

payload_dict = {"mode": 0, "direction": 0}
current_keys = set()

def serialize(payload):
    return ",".join(str(k) for k in payload.values())

def start_listener(loop: AbstractEventLoop, stop_event: Event, emergency_callback: Callable):
    def on_press(key):
        if key not in current_keys:
            current_keys.add(key)
            getPayloadFromKeys(current_keys)

    def on_release(key):
        if key in current_keys:
            current_keys.discard(key)
            getPayloadFromKeys(current_keys)
        if key == keyboard.Key.esc:
            payload_dict["mode"] = 0
            payload_dict["direction"] = 0
            emergency_callback(serialize(payload_dict))
            loop.call_soon_threadsafe(stop_event.set)
            return False

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

def getPayloadFromKeys(keys):
    global payload_dict

    if not keys or len(keys) != 1:
        payload_dict["direction"] = 0
        return

    if keyboard.Key.up in keys and payload_dict["mode"] != 0:
        payload_dict["direction"] = 1
    elif keyboard.Key.down in keys and payload_dict["mode"] != 0:
        payload_dict["direction"] = 0
    else:
        payload_dict["direction"] = 0

    for number in ['1', '2', '3', '4']:
        if keyboard.KeyCode.from_char(number) in keys:
            if payload_dict["mode"] == 0:
                payload_dict["mode"] = int(number)
            elif payload_dict["mode"] == int(number):
                payload_dict["mode"] = 0