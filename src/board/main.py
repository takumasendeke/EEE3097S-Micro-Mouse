# Board launcher: runs the precompiled Submission 4 code (s4.mpy).
# s4.mpy = workspace/final_task/main.py compiled with mpy-cross, so the
# board never has to compile the 38 KB source in its own RAM.
import gc
gc.collect()
import uct_mouse
try:
    import s4
    gc.collect()
    s4.mission()
except Exception as e:
    try:
        uct_mouse.set_motors(0, 0)
    except Exception:
        pass
    raise e
