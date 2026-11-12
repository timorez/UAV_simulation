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


class Quadcopter:
    def __init__(self, path: str, all_pois: list, sim_proxy, async_sim_call):
        self.sim = sim_proxy
        self.async_sim_call = async_sim_call
        try:
            self.handle = self.sim.getObject(path)
            self.tgt_handle = self.sim.getObject(f'{path}/target')
            self.altimeter = self.sim.getObject(f'{path}/proximitySensor')
            self.front_camera = self.sim.getObject(f'{path}/visionSensor')
            self.accelerometer = self.sim.getObject(f'{path}/Accelerometer')
            self.gyro = self.sim.getObject(f'{path}/GyroSensor')
        except Exception as e:
            raise RuntimeError('Не удалось подключиться к дрону') from e

        self.pos = self.sim.getObjectPosition(self.tgt_handle, self.sim.handle_world)
        self.state = State.IDLE
        self.speed = 15
        self.rotation_speed = 5
        self.step_time = 0.005
        self.tolerance = 0.25

        self.known_pois = {
            poi.index: {'pos': poi.pos, 'status': poi.status, 'by_whom': None}
            for poi in all_pois
        }

        self.claimed_tgt = None
        self.network: list[Quadcopter] = []


    def set_network(self, all_drones: list):
        self.network = [d for d in all_drones if d != self]


    async def recieve_claim(self, sender_handle, claimed_idx, claimed_distance):
        if self.known_pois[claimed_idx]['status'] == TaskStatus.FINISHED:
            return 'REJECTED'
        
        my_dist = await self.get_distance(*self.known_pois[claimed_idx]['pos'])
        
        if self.claimed_tgt == claimed_idx:
            if (claimed_distance) < ((my_dist) * 0.8):
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
    

    async def angle_to_target(self, target_pos):
        drone_pos = await self.get_position()
        dx = target_pos[0] - drone_pos[0]
        dy = target_pos[1] - drone_pos[1]
        return math.atan2(dy, dx)
    

    async def yaw(self, yaw_target: float, timeout: float = 10.0, wait_for_drone: bool = True, tol: float = 0.02) -> bool:
        tquat = await self.async_sim_call(
            self.sim.getObjectQuaternion, self.tgt_handle, self.sim.handle_world)
        yaw = 2.0 * math.atan2(tquat[2], tquat[3])    # qz, qw

        w_step = self.rotation_speed * self.step_time
        t0 = await self.async_sim_call(self.sim.getSimulationTime)

        while True:
            if (await self.async_sim_call(self.sim.getSimulationTime)) - t0 > timeout:
                return False

            dyaw = math.atan2(math.sin(yaw_target - yaw),
                            math.cos(yaw_target - yaw))
            if abs(dyaw) < 1e-3:
                break

            yaw += max(-w_step, min(w_step, dyaw))

            half = yaw * 0.5
            await self.async_sim_call(
                self.sim.setObjectQuaternion,
                self.tgt_handle, self.sim.handle_world,
                [0.0, 0.0, math.sin(half), math.cos(half)])

            await asyncio.sleep(self.step_time)

        if wait_for_drone:
            while True:
                if (await self.async_sim_call(self.sim.getSimulationTime)) - t0 > timeout:
                    return False

                dquat = await self.async_sim_call(
                    self.sim.getObjectQuaternion, self.handle, self.sim.handle_world)
                drone_yaw = 2.0 * math.atan2(dquat[2], dquat[3])

                dyaw = math.atan2(math.sin(yaw_target - drone_yaw),
                                math.cos(yaw_target - drone_yaw))

                if abs(dyaw) < tol:
                    return True

                await asyncio.sleep(self.step_time)

        return True
    

    async def move_to(self, x: float, y: float, z: float, timeout: float = 15.0, yaw: bool = True):
        target_pos = [x, y, z]
        start_pos = await self.async_sim_call(self.sim.getObjectPosition, self.tgt_handle, self.sim.handle_world)
        distance = math.sqrt(sum((target_pos[i] - start_pos[i]) ** 2 for i in range(3)))
        steps = max(1, math.ceil(distance / (self.speed * self.step_time)))

        self.state = State.MOVING
        if yaw:
            await self.yaw(await self.angle_to_target(target_pos))
        try:
            for step in range(1, steps + 1):
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

    def get_alt(self):
        detected, distance, *_ = self.sim.readProximitySensor(self.altimeter)
        return distance if detected else None

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
