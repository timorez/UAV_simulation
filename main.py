from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import os
import asyncio
from enum import Enum
from time import monotonic, sleep

client = RemoteAPIClient()
sim = client.require('sim')


class State(Enum):
    IDLE = 0
    MOVING = 1


class Quadrocopter():
    def __init__(self, path: str):
        try:
            self.handle = sim.getObject(path)
            self.tgt_handle = sim.getObject(f'{path}/target')
            # self.alt_handler = sim.getObject(f'{path}/proximitySensor')
        except Exception as e:
            raise RuntimeError('Не удалось подключиться к дрону') from e

        self.pos = sim.getObjectPosition(self.tgt_handle, sim.handle_world)
        self.state = State.IDLE
        self.speed = 2
        self.step_time = 0.1
        self.tolerance = 0.25

    # def get_alt(self):
    #     detected, distance, *_ = sim.readProximitySensor(self.alt_handler)
    #     return distance if detected else None

    def get_position(self):
        return sim.getObjectPosition(self.handle, sim.handle_world)

    async def move_to(self, x: float, y: float, z: float, timeout: float = 60.0):
        """Плавно перемещает target и ждёт, пока коптер достигнет точки."""
        target_pos = [x, y, z]
        start_pos = sim.getObjectPosition(self.tgt_handle, sim.handle_world)
        distance = math.sqrt(sum((target_pos[i] - start_pos[i]) ** 2 for i in range(3)))
        steps = max(1, math.ceil(distance / (self.speed * self.step_time)))

        self.state = State.MOVING
        try:
            for step in range(1, steps + 1):
                ratio = step / steps
                current_pos = [
                    start_pos[i] + (target_pos[i] - start_pos[i]) * ratio
                    for i in range(3)
                ]
                sim.setObjectPosition(self.tgt_handle, sim.handle_world, current_pos)
                self.pos = current_pos
                await asyncio.sleep(self.step_time)

            deadline = monotonic() + timeout
            while True:
                current_pos = self.get_position()
                remaining = math.sqrt(sum((target_pos[i] - current_pos[i]) ** 2 for i in range(3)))
                if remaining <= self.tolerance:
                    break
                if monotonic() >= deadline:
                    raise TimeoutError(f'Коптер не достиг точки {target_pos}')
                await asyncio.sleep(self.step_time)
        finally:
            self.state = State.IDLE

    async def move_vector(self, x: float = 0, y: float = 0, z: float = 0):
        """Перемещает коптер на заданный вектор относительно target."""
        await self.move_to(self.pos[0] + x, self.pos[1] + y, self.pos[2] + z)

    async def change_alt(self, alt: float):
        await self.move_to(self.pos[0], self.pos[1], self.pos[2] + alt)

    def get_distance(self, x: float, y: float, z: float):
        target_pos = [x, y, z]
        self_pos = self.get_position()
        distance = 0

        for idx in range(len(self_pos)):
            distance += (target_pos[idx] - self_pos[idx]) ** 2

        distance **= 0.5
        
        return distance

    async def fly_to_pois(self, pool):
        pois = get_poi_positions()
        if not pois:
            print('В сцене не найдены точки интереса')
            return

        while pool:
            closest_poi = min(
                pool, key=lambda poi: self.get_distance(*poi[1])
            )

            pool.remove(closest_poi)
            index, position = closest_poi

            print(f'[{self.handle}] Лечу к POI_{index}: {position}')
            await self.move_to(*position)


def get_poi_positions():
    """Возвращает позиции POI в порядке их числовых индексов."""
    pois = []
    handles = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
    for handle in handles:
        alias = sim.getObjectAlias(handle, 1).rsplit('/', 1)[-1]
        if not alias.startswith('POI_'):
            continue
        try:
            index = int(alias.split('_')[-1])
        except ValueError:
            continue
        pois.append((index, sim.getObjectPosition(handle, sim.handle_world)))
    return sorted(pois, key=lambda poi: poi[0])


async def main():
    name = input('Введите имя сцены для загрузки: ')
    path = os.path.join(os.getcwd(), f'{name}.ttt')

    try:
        sim.stopSimulation()
    except Exception:
        pass

    while sim.getSimulationState() != sim.simulation_stopped:
         await asyncio.sleep(0.1)
    sim.loadScene(path)

    targets_pool = get_poi_positions()

    quad0 = Quadrocopter('/Quadcopter[0]')
    quad1 = Quadrocopter('/Quadcopter[1]')

    sim.startSimulation()
    try:
        await asyncio.gather(
            quad0.fly_to_pois(targets_pool), quad1.fly_to_pois(targets_pool)
        )
    finally:
        sim.stopSimulation()


if __name__ == '__main__':
    asyncio.run(main())
