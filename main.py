# Copyright (C) 2025  Koutaro Mukai
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import argparse
import contextlib
import io
import platform
import threading
import time
import typing

import fastmcp
import PIL.Image
import serial.tools
import serial.tools.list_ports

# https://stackoverflow.com/a/51470016
with contextlib.redirect_stdout(None):
    import pygame
import pygame.camera
import pygame.image
import serial

mcp = fastmcp.FastMCP("GameCubeBridge")


def _get_camera_backend():
    """
    https://www.pygame.org/docs/ref/camera.html#pygame.camera.get_backends
    """
    pl = platform.system()
    if pl == "Windows":
        return "_camera (MSMF)"
    elif pl == "Linux":
        return "_camera (V4L2)"
    elif pl == "Darwin":
        return "OpenCV-Mac"
    else:
        raise NotImplementedError(f"{pl} is not supported.")


pygame.init()
pygame.camera.init(_get_camera_backend())

ser: serial.Serial | None = None
PORTS = [port.device for port in serial.tools.list_ports.comports()]

cam: pygame.camera.Camera | None = None
CAMERAS = pygame.camera.list_cameras()
cam_ready = threading.Event()


def _remap_m1_to_1_to_0_255(n: float):
    return max(0, min(255, int(round((n + 1) * 127.5))))


def _send(
    ser: serial.Serial,
    a: bool,
    b: bool,
    x: bool,
    y: bool,
    l: bool,
    r: bool,
    z: bool,
    start_pause: bool,
    control_pad: (
        typing.Literal["up"]
        | typing.Literal["up_right"]
        | typing.Literal["right"]
        | typing.Literal["down_right"]
        | typing.Literal["down"]
        | typing.Literal["down_left"]
        | typing.Literal["left"]
        | typing.Literal["up_left"]
        | typing.Literal["neutral"]
    ),
    control_stick: tuple[float, float],
    c_stick: tuple[float, float],
    reset: bool,
):
    """
    https://github.com/u1f992/jiangtun/blob/main/lib/jiangtun-core/src/nxmc2.c
    """
    byte0 = 0xAB
    byte1 = (
        z << 7
        # | zl << 6
        | r << 5
        | l << 4
        | x << 3
        | a << 2
        | b << 1
        | y
    )
    byte2 = (
        # capture << 5
        reset << 4
        # | r_click << 3
        # | l_click << 2
        | start_pause << 1
        # | minus
    )
    if control_pad == "up":
        byte3 = 0
    elif control_pad == "up_right":
        byte3 = 1
    elif control_pad == "right":
        byte3 = 2
    elif control_pad == "down_right":
        byte3 = 3
    elif control_pad == "down":
        byte3 = 4
    elif control_pad == "down_left":
        byte3 = 5
    elif control_pad == "left":
        byte3 = 6
    elif control_pad == "up_left":
        byte3 = 7
    elif control_pad == "neutral":
        byte3 = 8
    else:
        byte3 = 8
    byte4 = _remap_m1_to_1_to_0_255(control_stick[0])
    byte5 = _remap_m1_to_1_to_0_255(control_stick[1])
    byte6 = _remap_m1_to_1_to_0_255(c_stick[0])
    byte7 = _remap_m1_to_1_to_0_255(c_stick[1])

    ser.write([byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7, 0, 0, 0])


@mcp.tool()
def send_controller_input(
    a: bool = False,
    b: bool = False,
    x: bool = False,
    y: bool = False,
    l: bool = False,
    r: bool = False,
    z: bool = False,
    start_pause: bool = False,
    control_pad: (
        typing.Literal["up"]
        | typing.Literal["up_right"]
        | typing.Literal["right"]
        | typing.Literal["down_right"]
        | typing.Literal["down"]
        | typing.Literal["down_left"]
        | typing.Literal["left"]
        | typing.Literal["up_left"]
        | typing.Literal["neutral"]
    ) = "neutral",
    control_stick: tuple[float, float] = (0, 0),
    c_stick: tuple[float, float] = (0, 0),
    reset: bool = False,
    hold_time: float = 0.0,
    wait_time: float = 0.0,
):
    if ser is not None:
        _send(
            ser,
            a,
            b,
            x,
            y,
            l,
            r,
            z,
            start_pause,
            control_pad,
            control_stick,
            c_stick,
            reset,
        )
    time.sleep(max(0, hold_time))

    if ser is not None:
        _send(
            ser,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            "neutral",
            (0, 0),
            (0, 0),
            False,
        )
    time.sleep(max(0, wait_time))

    if cam is not None:
        cam_ready.wait()
        cam_ready.clear()
        pygame_image = cam.get_image()
        cam_ready.set()
        raw_bytes = pygame.image.tobytes(pygame_image, "RGB")
        pil_image = PIL.Image.frombytes("RGB", pygame_image.get_size(), raw_bytes)
    else:
        pil_image = PIL.Image.new("RGB", (640, 480), (0, 0, 0))

    # https://github.com/jlowin/fastmcp?tab=readme-ov-file#images
    buffer = io.BytesIO()
    pil_image.save(buffer, format="PNG")
    return fastmcp.Image(data=buffer.getvalue(), format="png")


def _thread_cam_get_image():
    while True:
        try:
            if cam is not None:
                cam_ready.wait()
                cam_ready.clear()
                cam.get_image()
                cam_ready.set()
        except:  # noqa: E722
            pass
        time.sleep(1 / 120)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial-port", type=str)
    parser.add_argument("--camera", type=str)
    parser.add_argument("--skip-frames", type=int, default=0)
    parser.add_argument_group("available ports", f"{PORTS}")
    parser.add_argument_group("available cameras", f"{CAMERAS}")
    args = parser.parse_args()

    if args.serial_port is None or args.camera is None:
        parser.print_help()
        exit(0)

    ser = serial.Serial(args.serial_port)
    cam = pygame.camera.Camera(args.camera)
    cam.start()

    # Some inexpensive cameras return a test pattern for a few seconds after connection
    for _ in range(args.skip_frames):
        cam.get_image()
    cam_ready.set()
    threading.Thread(target=_thread_cam_get_image).start()

    mcp.run()
