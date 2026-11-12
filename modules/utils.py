from __future__ import annotations
import threading
import asyncio

sim_lock = threading.Lock()
"""Общий мьютекс и обертки для асинхронного вызова функций API"""

def _sync_sim_call(func, *args, **kwargs):
    with sim_lock:
        return func(*args, **kwargs)

async def async_sim_call(func, *args, **kwargs):
    return await asyncio.to_thread(_sync_sim_call, func, *args, **kwargs)
