"""One high-resolution monotonic domain for control, I/O and local inputs.

Windows Python 3.12's GetTickCount64 can quantize monotonic() at 15.625 ms.
QPC is monotonic too, and does not quantize the trajectory time step that way.
The facade leaves Python's global time module and event-loop clocks untouched.
"""
import time as _time
import asyncio

_native_sleep=_time.sleep


class ControlClock:
    @staticmethod
    def monotonic():return _time.perf_counter()

    @staticmethod
    def monotonic_ns():return _time.perf_counter_ns()

    @staticmethod
    def get_clock_info(name):return _time.get_clock_info('perf_counter' if name=='monotonic' else name)

    def __getattr__(self,name):return getattr(_time,name)


clock=ControlClock()


async def sleep_until(deadline):
    """Do not run the next control tick early on coarse asyncio clocks.

    Native sleep runs outside the event loop; QPC remains the deadline authority.
    Cancelling this wait cannot schedule any device operations.
    """
    while True:
        remaining=deadline-clock.monotonic()
        if remaining<=0:
            # Overruns must still let stop/input callbacks and cancellation run.
            await asyncio.sleep(0)
            return
        await asyncio.to_thread(_native_sleep,remaining)
