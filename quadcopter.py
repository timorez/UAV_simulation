from __future__ import annotations
import math
import asyncio
from enum import Enum
from time import monotonic


class State(Enum):
    IDLE = 0
    MOVING = 1


class TaskStatus(Enum):
    FREE = 0
    IN_PROGRESS = 1
    FINISHED = 2


class Task:
    def __init__(self, poi):
        self.index = poi[0]
        self.pos = poi[1]
        self.status = TaskStatus.FREE

    def __repr__(self):
        return f'{self.index}: {self.pos}, {self.status}'


class Quadrocopter:
    def __init__(self, path: str, all_pois: list, sim_proxy, async_sim_call):
        self.sim = sim_proxy
        self.async_sim_call = async_sim_call

        try:
            self.handle = self.sim.getObject(path)
            self.tgt_handle = self.sim.getObject(f'{path}/target')
        except Exception as e:
            raise RuntimeError('Не удалось подключиться к дрону') from e

        self.pos = self.sim.getObjectPosition(self.tgt_handle, self.sim.handle_world)
        self.state = State.IDLE
        self.speed = 4
        self.step_time = 0.1
        self.tolerance = 0.25

        self.known_pois = {
            poi.index: {'pos': poi.pos, 'status': poi.status, 'by_whom': None}
            for poi in all_pois
        }

        self.claimed_tgt = None
        self.network: list[Quadrocopter] = []

    def set_network(self, all_drones: list):
        self.network = [d for d in all_drones if d != self]

    async def recieve_claim(self, sender_handle, claimed_idx, claimed_distance):
        if self.known_pois[claimed_idx]['status'] == TaskStatus.FINISHED:
            return 'REJECTED'
        
        my_dist = await self.get_distance(*self.known_pois[claimed_idx]['pos'])
        
        if self.claimed_tgt == claimed_idx:
            if (claimed_distance + sender_handle) < ((my_dist + self.handle) * 0.8):
                print(f'[{self.handle}]: уступаю  цель {self.claimed_tgt} [{sender_handle}]')
                self.known_pois[claimed_idx]['status'] = TaskStatus.IN_PROGRESS
                self.known_pois[claimed_idx]['by_whom'] = sender_handle
                self.claimed_tgt = None
                return 'ACCEPTED'
            else:
                print(f'[{self.handle}]: забираю  цель {self.claimed_tgt} у [{sender_handle}]')
                return 'REJECTED'

        print(f'[{self.handle}]: получил обновление по цели {claimed_idx} от [{sender_handle}]')
        self.known_pois[claimed_idx]['status'] = TaskStatus.IN_PROGRESS
        self.known_pois[claimed_idx]['by_whom'] = sender_handle
        return 'UPDATED'

    def receive_done_signal(self, poi_idx):
        """Уведомление от соседа, что точка выполнена."""
        self.known_pois[poi_idx]['status'] = TaskStatus.FINISHED

    async def get_position(self):
        return await self.async_sim_call(self.sim.getObjectPosition, self.handle, self.sim.handle_world)

    async def move_to(self, x: float, y: float, z: float, timeout: float = 15.0):
        target_pos = [x, y, z]
        start_pos = await self.async_sim_call(self.sim.getObjectPosition, self.tgt_handle, self.sim.handle_world)
        distance = math.sqrt(sum((target_pos[i] - start_pos[i]) ** 2 for i in range(3)))
        steps = max(1, math.ceil(distance / (self.speed * self.step_time)))

        self.state = State.MOVING
        try:
            for step in range(1, steps + 1):
                if self.claimed_tgt is None:
                    return False

                ratio = step / steps
                current_pos = [
                    start_pos[i] + (target_pos[i] - start_pos[i]) * ratio
                    for i in range(3)
                ]
                await self.async_sim_call(self.sim.setObjectPosition, self.tgt_handle, self.sim.handle_world, current_pos)
                self.pos = current_pos
                await asyncio.sleep(self.step_time)

            deadline = monotonic() + timeout
            while True:
                if self.claimed_tgt is None:
                    return False

                current_pos = await self.get_position()
                remaining = math.sqrt(sum((target_pos[i] - current_pos[i]) ** 2 for i in range(3)))
                if remaining <= self.tolerance:
                    return True
                if monotonic() >= deadline:
                    return False
                await asyncio.sleep(self.step_time)
        finally:
            self.state = State.IDLE

    async def move_vector(self, x: float = 0, y: float = 0, z: float = 0):
        """Перемещает коптер на заданный вектор относительно target."""
        await self.move_to(self.pos[0] + x, self.pos[1] + y, self.pos[2] + z)

    async def change_alt(self, alt: float):
        await self.move_to(self.pos[0], self.pos[1], self.pos[2] + alt)

    async def get_distance(self, x: float, y: float, z: float):
        target_pos = [x, y, z]
        self_pos = await self.get_position()
        distance = math.sqrt(sum((target_pos[i] - self_pos[i]) ** 2 for i in range(3)))
        return distance

    async def fly_to_pois(self):
        if not self.known_pois:
            print(f'[{self.handle}] В сцене не найдены точки интереса')
            return

        while any(info['status'] == TaskStatus.FREE for info in self.known_pois.values()):
            if self.claimed_tgt is not None:
                await asyncio.sleep(0.2)
                continue

            free_pois = {
                idx: info
                for idx, info in self.known_pois.items()
                if info['status'] == TaskStatus.FREE and idx != self.claimed_tgt
            }

            if not free_pois:
                await asyncio.sleep(0.5)
                continue

            distances = {}
            for idx in free_pois.keys():
                distances[idx] = await self.get_distance(*free_pois[idx]['pos'])
            
            best_idx = min(distances, key=distances.get)
            my_dist = distances[best_idx]

            self.claimed_tgt = best_idx
            conflict = False

            for drone in self.network:
                response = await drone.recieve_claim(self.handle, best_idx, my_dist)
                if response == 'REJECTED':
                    conflict = True

            if conflict:
                self.claimed_tgt = None
                await asyncio.sleep(0.1)
                continue

            poi_pos = self.known_pois[best_idx]['pos']

            success = await self.move_to(*poi_pos)
            if success and self.claimed_tgt == best_idx:
                self.known_pois[best_idx]['status'] = TaskStatus.FINISHED
                for drone in self.network:
                    drone.receive_done_signal(best_idx)
                print(f'[{self.handle}] Завершил POI_{best_idx}')

            self.claimed_tgt = None
            await asyncio.sleep(0.1)
