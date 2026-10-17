from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import os
from time import sleep

client = RemoteAPIClient()
sim = client.require('sim')

class Quadrocopter():
    def __init__(self, handle, tgt_handle):
        self.handle = handle
        self.tgt = tgt_handle
        self.pos = sim.getObjectPosition(handle, sim.handle_world)
        self.speed = 0.5

    def climb(self, alt):
        requested_pos = [self.pos[0], self.pos[1], self.pos[2] + alt]

        try:
            while self.pos[2] != requested_pos[2]:
                if self.pos[2] + self.speed > requested_pos[2]:
                    current_pos = [self.pos[0], self.pos[1], self.pos[2] + (requested_pos[2] - self.pos[1])]
                else:
                    current_pos = [self.pos[0], self.pos[1], self.pos[2] + self.speed]
                    sim.setObjectPosition(self.tgt, sim.handle_world, current_pos)

                self.pos = current_pos

                sleep(0.1)


        except Exception as e:
            print(f'Ошибка {e}')



def get_quadro_handles():
    try:
        drone_handle = sim.getObject('/Quadcopter')
        drone_tgt_handle = sim.getObject('/Quadcopter/target')
    except Exception as e:
        print('Не удалось подключиться к дрону')

    return drone_handle, drone_tgt_handle

def main():
    name = input('Введите имя имя сцены для загрузки: ')
    path = os.path.join(os.getcwd(), f'{name}.ttt')
    print(path)
    sim.loadScene(path)

    drone, tgt = get_quadro_handles()

    quad = Quadrocopter(drone, tgt)

    sim.startSimulation()

    quad.climb(5)
    sleep(10)

    sim.pauseSimulation()



if __name__ == '__main__':
    main()
