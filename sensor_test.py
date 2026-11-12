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
        await quad0.move_vector(1.5, -1.5, 0)
        alt1 = quad0.get_alt() 

        await quad0.move_vector(0, 0, 3)
        print("Команда движения отправлена")

        alt2 = quad0.get_alt()

        print(alt1, alt2) 
        await asyncio.sleep(3)
            

    finally:
        await utl.async_sim_call(sim.stopSimulation)
    

if __name__ == '__main__':
    asyncio.run(main())
