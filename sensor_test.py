from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from modules import quadcopter
from modules import utils as utl
from time import sleep
import asyncio


client = RemoteAPIClient()
sim = client.require('sim')


async def main():

    quad0 = quadcopter.Quadcopter('/Quadcopter[0]', [], sim, utl.async_sim_call)

    await utl.async_sim_call(sim.startSimulation)


    try:
        await quad0.start()
        print(f'true pos: {list(map(lambda x: round(x, 3), await quad0.get_position()))}\nins_pos: {list(map(lambda x: round(x, 3), quad0.get_ins_position()))}') 

        await quad0.move_vector(1.5, -1.5, 0)
        print(f'true pos: {list(map(lambda x: round(x, 3), await quad0.get_position()))}\nins_pos: {list(map(lambda x: round(x, 3), quad0.get_ins_position()))}') 

        await quad0.move_vector(0, 0, 3)
        print("Команда движения отправлена")

        print(f'true pos: {list(map(lambda x: round(x, 3), await quad0.get_position()))}\nins_pos: {list(map(lambda x: round(x, 3), quad0.get_ins_position()))}') 

        await asyncio.sleep(3)
            

    finally:
        await quad0.stop()
        await utl.async_sim_call(sim.stopSimulation)
    

if __name__ == '__main__':
    asyncio.run(main())
