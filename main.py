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

    targets_pool = [quadcopter.Task(_) for _ in await utl.async_sim_call(get_poi_positions)]

    quad0 = quadcopter.Quadcopter('/Quadcopter[0]', targets_pool, sim, utl.async_sim_call)
    quad1 = quadcopter.Quadcopter('/Quadcopter[1]', targets_pool, sim, utl.async_sim_call)
    quad2 = quadcopter.Quadcopter('/Quadcopter[2]', targets_pool, sim, utl.async_sim_call)
    quad3 = quadcopter.Quadcopter('/Quadcopter[3]', targets_pool, sim, utl.async_sim_call)

    all_drones = [quad0, quad1, quad2, quad3]
    for quad in all_drones:
        quad.set_network(all_drones)

    await utl.async_sim_call(sim.startSimulation)
    try:
        await asyncio.gather(
            quad0.fly_to_pois(), 
            quad1.fly_to_pois(), 
            quad2.fly_to_pois(), 
            quad3.fly_to_pois()
        )
    finally:
        await utl.async_sim_call(sim.stopSimulation)


if __name__ == '__main__':
    asyncio.run(main())   