from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from modules import quadcopter
from modules import utils as utl
from main import get_poi_positions
from time import sleep
import asyncio
import cv2
import numpy as np


client = RemoteAPIClient()
sim = client.require('sim')


async def save_image(quad):
    img = await quad.get_camera_image()
        
    if img is None:
        print("ОШИБКА: get_camera_image() вернул None")
    elif img.size == 0:
        print(f"ОШИБКА: Изображение пустое, shape: {img.shape}")
    else:
        print(f"Успех! Получено изображение shape: {img.shape}, dtype: {img.dtype}")
        img = cv2.flip(img, 0)
        img = np.rot90(img, k=1)
        cv2.imwrite("result.png", img)
        print("Изображение сохранено в result.png")

    await quad.move_vector(0, -0.5, 0)

    await asyncio.sleep(1)

    img = await quad.get_camera_image()
        
    if img is None:
        print("ОШИБКА: get_camera_image() вернул None")
    elif img.size == 0:
        print(f"ОШИБКА: Изображение пустое, shape: {img.shape}")
    else:
        print(f"Успех! Получено изображение shape: {img.shape}, dtype: {img.dtype}")
        img = cv2.flip(img, 0)
        img = np.rot90(img, k=1)
        cv2.imwrite("result1.png", img)
        print("Изображение сохранено в result.png")


async def main():

    targets_pool = [quadcopter.POI(_) for _ in await utl.async_sim_call(get_poi_positions)]

    quad0 = quadcopter.Quadcopter('/Quadcopter', targets_pool, sim, utl.async_sim_call, 1)
    # quad1 = quadcopter.Quadcopter('/Quadcopter[1]', targets_pool, sim, utl.async_sim_call, 5)


    drones = [quad0]

    for d in drones:
        d.set_network(drones)

    await utl.async_sim_call(sim.startSimulation)


    try:
        await quad0.start()
        # await quad1.start()

        await asyncio.sleep(2)
        
        # await quad0.move_vector(3, 0, 0)
        

        await asyncio.sleep(25)

    except KeyboardInterrupt:
        await quad0.stop()
        # await quad1.stop()
        await utl.async_sim_call(sim.stopSimulation)
    
    finally:
        await quad0.stop()
        # await quad1.stop()
        await utl.async_sim_call(sim.stopSimulation)
    

if __name__ == '__main__':
    asyncio.run(main())
