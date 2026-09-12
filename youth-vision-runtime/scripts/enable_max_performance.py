#!/usr/bin/env python3
import time

from jtop import jtop


def enable_max_performance() -> None:
    last_error = None

    for attempt in range(5):
        try:
            with jtop(interval=0.5) as jetson:
                jetson.nvpmodel = 0
                jetson.jetson_clocks = True

                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline:
                    time.sleep(0.5)
                    jetson.ok()
                    if jetson.jetson_clocks:
                        print(f"max performance enabled: {jetson.nvpmodel}")
                        return

                raise RuntimeError("jetson_clocks did not become active")
        except Exception as exc:
            last_error = exc
            if attempt < 4:
                time.sleep(1.0)

    raise RuntimeError(f"failed to enable max performance: {last_error}")


if __name__ == "__main__":
    enable_max_performance()
