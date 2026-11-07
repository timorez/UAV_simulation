from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import os
import asyncio
import threading

from quadcopter import Quadrocopter, Task

client = RemoteAPIClient()
sim = client.require('sim')

"""Общий мьютекс и обертки для асинхронного вызова функций API"""
sim_lock = threading.Lock()

def _sync_sim_call(func, *args, **kwargs):
    with sim_lock:
        return func(*args, **kwargs)

async def async_sim_call(func, *args, **kwargs):
    return await asyncio.to_thread(_sync_sim_call, func, *args, **kwargs)


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
        await async_sim_call(sim.stopSimulation)
    except Exception:
        pass

    while await async_sim_call(sim.getSimulationState) != sim.simulation_stopped:
         await asyncio.sleep(0.1)
         
    await async_sim_call(sim.loadScene, path)

    targets_pool = [Task(_) for _ in await async_sim_call(get_poi_positions)]

    quad0 = Quadrocopter('/Quadcopter[0]', targets_pool, sim, async_sim_call)
    quad1 = Quadrocopter('/Quadcopter[1]', targets_pool, sim, async_sim_call)
    quad2 = Quadrocopter('/Quadcopter[2]', targets_pool, sim, async_sim_call)
    quad3 = Quadrocopter('/Quadcopter[3]', targets_pool, sim, async_sim_call)

    all_drones = [quad0, quad1, quad2, quad3]
    for quad in all_drones:
        quad.set_network(all_drones)

    await async_sim_call(sim.startSimulation)
    try:
        await asyncio.gather(
            quad0.fly_to_pois(), 
            quad1.fly_to_pois(), 
            quad2.fly_to_pois(), 
            quad3.fly_to_pois()
        )
    finally:
        await async_sim_call(sim.stopSimulation)


if __name__ == '__main__':
    asyncio.run(main())   