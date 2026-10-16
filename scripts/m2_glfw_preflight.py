#!/usr/bin/env python3
"""Minimal system-GLFW context check for the authorized M2 batch environment."""
import ctypes
import json
import os
import sys
import time
from pathlib import Path


def main() -> int:
    output = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    result = {"started_unix_ns": time.time_ns(), "python": sys.executable,
        "uid": os.getuid(), "euid": os.geteuid(),
        "environment": {key: os.environ.get(key) for key in
            ("DISPLAY", "XAUTHORITY", "MUJOCO_GL", "LD_LIBRARY_PATH", "LIBGL_DRIVERS_PATH")},
        "xauthority_readable": bool(os.environ.get("XAUTHORITY") and os.access(os.environ["XAUTHORITY"], os.R_OK)),
        "glfw_init": False, "window_created": False}
    try:
        glfw = ctypes.CDLL("libglfw.so.3")
        glfw.glfwInit.restype = ctypes.c_int
        glfw.glfwWindowHint.argtypes = [ctypes.c_int, ctypes.c_int]
        glfw.glfwCreateWindow.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_char_p,
                                         ctypes.c_void_p, ctypes.c_void_p]
        glfw.glfwCreateWindow.restype = ctypes.c_void_p
        glfw.glfwMakeContextCurrent.argtypes = [ctypes.c_void_p]
        glfw.glfwDestroyWindow.argtypes = [ctypes.c_void_p]
        if glfw.glfwInit() != 1:
            raise RuntimeError("glfwInit returned false")
        result["glfw_init"] = True
        glfw.glfwWindowHint(0x00020004, 0)  # GLFW_VISIBLE false
        window = glfw.glfwCreateWindow(16, 16, b"M2 preflight", None, None)
        if not window:
            raise RuntimeError("GLFW window/context creation failed")
        result["window_created"] = True
        glfw.glfwMakeContextCurrent(window)
        gl = ctypes.CDLL("libGL.so.1")
        gl.glGetString.argtypes = [ctypes.c_uint]
        gl.glGetString.restype = ctypes.c_char_p
        for name, enum in (("vendor", 0x1F00), ("renderer", 0x1F01), ("version", 0x1F02)):
            value = gl.glGetString(enum)
            result[name] = value.decode("utf-8", "replace") if value else None
        glfw.glfwDestroyWindow(window)
        glfw.glfwTerminate()
        result["window_closed"] = True
        result["status"] = "PASS"
    except Exception as error:
        result.update({"status": "FAIL", "error": f"{type(error).__name__}: {error}"})
    result["finished_unix_ns"] = time.time_ns()
    encoded = json.dumps(result, indent=2, ensure_ascii=False) + "\n"
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
