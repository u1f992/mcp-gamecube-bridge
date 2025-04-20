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
import atexit
import contextlib
import ctypes
import io
import multiprocessing
import multiprocessing.synchronize
import multiprocessing.sharedctypes
import platform
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


def _get_list_cameras():
    pygame.init()
    pygame.camera.init(_get_camera_backend())
    ret = pygame.camera.list_cameras()
    pygame.camera.quit()
    pygame.quit()
    return ret


def _remap_m1_to_1_to_0_255(n: float):
    return max(0, min(255, int(round((n + 1) * 127.5))))


_ControlPad = (
    typing.Literal["up"]
    | typing.Literal["up_right"]
    | typing.Literal["right"]
    | typing.Literal["down_right"]
    | typing.Literal["down"]
    | typing.Literal["down_left"]
    | typing.Literal["left"]
    | typing.Literal["up_left"]
    | typing.Literal["neutral"]
)


def _send_gamecube_controller_input(
    ser: serial.Serial,
    a: bool,
    b: bool,
    x: bool,
    y: bool,
    l: bool,  # noqa: E741
    r: bool,
    z: bool,
    start_pause: bool,
    control_pad: _ControlPad,
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
    byte5 = 255 - _remap_m1_to_1_to_0_255(control_stick[1])
    byte6 = _remap_m1_to_1_to_0_255(c_stick[0])
    byte7 = 255 - _remap_m1_to_1_to_0_255(c_stick[1])

    ser.write([byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7, 0, 0, 0])


def _process(
    camera: str,
    buffer: ctypes.Array[ctypes.c_uint8],
    ready: multiprocessing.synchronize.Event,
    cancel: multiprocessing.synchronize.Event,
):
    pygame.init()
    pygame.camera.init(_get_camera_backend())
    
    atexit.register(pygame.camera.quit)
    atexit.register(pygame.quit)
    
    cam = pygame.camera.Camera(camera)
    cam.start()

    first = True

    try:
        while not cancel.is_set():
            image = cam.get_image()
            if first:
                first = False
            elif not ready.wait(timeout=1):
                raise TimeoutError("_process: ready.wait")
            ready.clear()
            try:
                memoryview(buffer).cast("B")[:] = memoryview(
                    pygame.image.tobytes(image, "RGB")
                ).cast("B")[:]
            finally:
                ready.set()
    finally:
        cam.stop()
        pygame.camera.quit()
        pygame.quit()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial-port", type=str)
    parser.add_argument("--camera", type=str)
    parser.add_argument_group(
        "available ports",
        f"{[port.device for port in serial.tools.list_ports.comports()]}",
    )
    parser.add_argument_group("available cameras", f"{_get_list_cameras()}")
    args = parser.parse_args()

    if args.serial_port is None or args.camera is None:
        parser.print_help()
        exit(0)

    ser = serial.Serial(args.serial_port)

    pygame.init()
    pygame.camera.init(_get_camera_backend())
    cam = pygame.camera.Camera(args.camera)
    cam.start()
    image = cam.get_image()
    image_size = image.get_size()
    buffer_size = len(pygame.image.tobytes(image, "RGB"))
    cam.stop()
    pygame.camera.quit()
    pygame.quit()

    with multiprocessing.Manager() as manager:
        buffer = multiprocessing.sharedctypes.RawArray(ctypes.c_uint8, buffer_size)
        image_buffer = buffer
        ready = manager.Event()
        cancel = manager.Event()
        process = multiprocessing.Process(
            target=_process, args=(args.camera, buffer, ready, cancel), daemon=True
        )
        process.start()

        @mcp.tool()
        def send_gamecube_controller_input(
            a: bool = False,
            b: bool = False,
            x: bool = False,
            y: bool = False,
            l: bool = False,  # noqa: E741
            r: bool = False,
            z: bool = False,
            start_pause: bool = False,
            control_pad: _ControlPad = "neutral",
            control_stick: tuple[float, float] = (0, 0),
            c_stick: tuple[float, float] = (0, 0),
            reset: bool = False,
            hold_time: float = 0.0,
            wait_time: float = 0.0,
        ):
            _send_gamecube_controller_input(
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

            _send_gamecube_controller_input(
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

            if not ready.wait(timeout=1):
                raise TimeoutError("send_gamecube_controller_input: ready.wait")
            ready.clear()
            try:
                pil_image = PIL.Image.frombytes(
                    "RGB", image_size, memoryview(image_buffer).cast("B")
                )
            finally:
                ready.set()

            # https://github.com/jlowin/fastmcp?tab=readme-ov-file#images
            buffer = io.BytesIO()
            pil_image.save(buffer, format="PNG")
            return fastmcp.Image(data=buffer.getvalue(), format="png")

        try:
            mcp.run()
        finally:
            cancel.set()
            process.join()


if __name__ == "__main__":
    main()
