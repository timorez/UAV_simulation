from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from modules import quadcopter
from modules import utils as utl
from main import get_poi_positions
from time import sleep
import asyncio


client = RemoteAPIClient()
sim = client.require('sim')


async def main():

    targets_pool = [quadcopter.POI(_) for _ in await utl.async_sim_call(get_poi_positions)]

    quad0 = quadcopter.Quadcopter('/Quadcopter[0]', targets_pool, sim, utl.async_sim_call, 5)
    quad1 = quadcopter.Quadcopter('/Quadcopter[1]', targets_pool, sim, utl.async_sim_call, 5)


    drones = [quad0, quad1]

    for d in drones:
        d.set_network(drones)

    await utl.async_sim_call(sim.startSimulation)


    try:
        await quad0.start()
        await quad1.start()
        # print(f'true pos: {list(map(lambda x: round(x, 3), await quad0.get_position()))}\nins_pos: {list(map(lambda x: round(x, 3), quad0.get_ins_position()))}') 

        # await quad0.move_vector(1.5, -1.5, 0)
        # print(f'true pos: {list(map(lambda x: round(x, 3), await quad0.get_position()))}\nins_pos: {list(map(lambda x: round(x, 3), quad0.get_ins_position()))}') 

        # await quad0.move_vector(0, 0, 3)
        # print("Команда движения отправлена")

        # print(f'true pos: {list(map(lambda x: round(x, 3), await quad0.get_position()))}\nins_pos: {list(map(lambda x: round(x, 3), quad0.get_ins_position()))}') 

        await asyncio.sleep(25)
            

    finally:
        await quad0.stop()
        await quad1.stop()
        await utl.async_sim_call(sim.stopSimulation)
    

if __name__ == '__main__':
    asyncio.run(main())
