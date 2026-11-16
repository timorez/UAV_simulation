from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import os
import asyncio
import threading
from modules import quadcopter
from modules import utils as utl


client = RemoteAPIClient()
sim = client.require('sim')


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
        await utl.async_sim_call(sim.stopSimulation)
    except Exception:
        pass

    while await utl.async_sim_call(sim.getSimulationState) != sim.simulation_stopped:
         await asyncio.sleep(0.1)
         
    await utl.async_sim_call(sim.loadScene, path)

    targets_pool = [quadcopter.POI(_) for _ in await utl.async_sim_call(get_poi_positions)]

    quad0 = quadcopter.Quadcopter('/Quadcopter[0]', targets_pool, sim, utl.async_sim_call, rotation_speed=10)
    quad1 = quadcopter.Quadcopter('/Quadcopter[1]', targets_pool, sim, utl.async_sim_call, rotation_speed=10)
    quad2 = quadcopter.Quadcopter('/Quadcopter[2]', targets_pool, sim, utl.async_sim_call, rotation_speed=10)
    quad3 = quadcopter.Quadcopter('/Quadcopter[3]', targets_pool, sim, utl.async_sim_call, rotation_speed=10)

    all_drones = [quad0, quad1, quad2, quad3]
    for quad in all_drones:
        quad.set_network(all_drones)

    await utl.async_sim_call(sim.startSimulation)
    try:
        await quad0.start()
        await quad1.start()
        await quad2.start()
        await quad3.start()

        await asyncio.sleep(55)

    finally:
        await quad0.stop()
        await quad1.stop()
        await quad2.stop()
        await quad3.stop()
        await utl.async_sim_call(sim.stopSimulation)


if __name__ == '__main__':
    asyncio.run(main())   