from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from PIL import Image, ImageDraw
import random
import os
import math
import time
import argparse

# ---------- НАСТРОЙКИ СЦЕНЫ ----------
AREA_SIZE = 20.0          # размер области (м), от -10 до +10 по X и Y
NUM_BUILDINGS = 8         # количество зданий
NUM_WALLS = 6             # количество стен
NUM_POLES = 10            # количество столбов/деревьев
NUM_POI = 5               # количество точек интереса
RANDOM_SEED = 42          # для воспроизводимости; None — случайно

parser = argparse.ArgumentParser(description="Scene generator")
parser.add_argument("-n", "--name", type=str, required=True, help="File name")
parser.add_argument("-s", "--size", type=int, default=AREA_SIZE, help="Area size")
parser.add_argument("-b", "--buildings", type=int, default=NUM_BUILDINGS, help="Number of buildings")
parser.add_argument("-w", "--walls", type=int, default=NUM_WALLS, help="Number of walls")
parser.add_argument("-p", "--poles", type=int, default=NUM_POLES, help="Number of poles")
parser.add_argument("-P", "--POI", type=int, default=NUM_POI, help="Number of POIs")
args = parser.parse_args()

# ---------- ПОДКЛЮЧЕНИЕ ----------
client = RemoteAPIClient()
sim = client.require('sim')

if RANDOM_SEED is not None:
    random.seed(RANDOM_SEED)


# ---------- ОЧИСТКА СЦЕНЫ ----------
def clear_scene():
    """Удаляет пол и все объекты, созданные предыдущими запусками скрипта."""
    objects_to_remove = []
    
    # 1. Ищем стандартный пол
    for name in ['/Floor', '/ResizableFloor_5_25', '/infiniteFloor', '/box']:
        try:
            h = sim.getObject(name)
            objects_to_remove.append(h)
        except:
            pass
            
    # 2. Ищем все объекты с нашими префиксами
    all_objects = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
    for h in all_objects:
        try:
            alias = sim.getObjectAlias(h, 1)
            if alias.startswith(('Obstacle_', 'POI_', 'Ground')):
                objects_to_remove.append(h)
        except:
            pass

    # 3. Удаляем всё разом (если список не пуст)
    if objects_to_remove:
        sim.removeObjects(objects_to_remove)

# ---------- ПОЛ С КЛЕТКАМИ ----------
def create_checkered_floor(size: int):
    # Генерируем текстуру (как у вас)
    img_size = 256
    cell = 32
    img = Image.new('RGB', (img_size, img_size), 'white')
    draw = ImageDraw.Draw(img)
    for i in range(0, img_size, cell):
        for j in range(0, img_size, cell):
            if (i // cell + j // cell) % 2 == 0:
                draw.rectangle([i, j, i + cell, j + cell], fill='#c8c8c8')
    texture_path = os.path.join(os.getcwd(), 'checker_floor.png')
    img.save(texture_path)

    # 1. Создаём текстуру (плоскость-посредник)
    #    Указываем planeSizes, чтобы плоскость была квадратной
    tex_shape, tex_id, _ = sim.createTexture(
        texture_path,
        4 + 8,               # options: повтор по U и V
        [1.0, 1.0],          # planeSizes: размер плоскости
        None, None, 0, None
    )

    # 2. Создаём кубоид-пол
    floor = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [size, size, 0.1], 0)
    sim.setObjectAlias(floor, 'Ground')
    sim.setObjectPosition(floor, sim.handle_world, [0, 0, -0.05])

    # 3. Применяем текстуру к полу (пока плоскость-посредник ещё существует)
    sim.setShapeTexture(
        floor,
        tex_id,
        0,                   # mappingMode: проекция
        4 + 8,               # options: повтор по U и V
        [size, size],  # uvScaling: 1 клетка = 1 метр
        None, None
    )

    # 4. Теперь можно удалить плоскость-посредник
    sim.removeObjects([tex_shape])

    # Делаем пол статическим
    sim.setObjectInt32Param(floor, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(floor, sim.shapeintparam_respondable, 1)

    return floor

# ---------- ГЕНЕРАЦИЯ ПРЕПЯТСТВИЙ ----------
def random_position(size=args.size, min_dist_from_origin=3.0):
    """Случайная позиция в области, не ближе min_dist от центра (место старта БПЛА)."""
    while True:
        x = random.uniform(-size/2 + 1, size/2 - 1)
        y = random.uniform(-size/2 + 1, size/2 - 1)
        if math.hypot(x, y) >= min_dist_from_origin:
            return x, y

def create_building():
    """Создаёт здание — вертикальный кубоид."""
    w = random.uniform(0.8, 2.0)
    d = random.uniform(0.8, 2.0)
    h = random.uniform(2.0, 6.0)
    x, y = random_position()

    handle = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid,
        [w, d, h],
        0
    )
    sim.setObjectPosition(handle, sim.handle_world, [x, y, h/2])
    # Случайный поворот вокруг Z
    sim.setObjectOrientation(handle, sim.handle_world, [0, 0, random.uniform(0, math.pi)])
    sim.setObjectAlias(handle, f'Obstacle_Building_{random.randint(1000,9999)}')
    sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse,
                      [random.uniform(0.4, 0.9),
                       random.uniform(0.4, 0.9),
                       random.uniform(0.4, 0.9)])
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 1)
    return handle

def create_wall():
    """Создаёт стену — длинный тонкий кубоид."""
    length = random.uniform(2.0, 5.0)
    thickness = random.uniform(0.15, 0.3)
    height = random.uniform(1.5, 3.5)
    x, y = random_position()
    angle = random.uniform(0, math.pi)

    handle = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid,
        [length, thickness, height],
        0
    )
    sim.setObjectPosition(handle, sim.handle_world, [x, y, height/2])
    sim.setObjectOrientation(handle, sim.handle_world, [0, 0, angle])
    sim.setObjectAlias(handle, f'Obstacle_Wall_{random.randint(1000,9999)}')
    sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, [0.7, 0.7, 0.6])
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 1)
    return handle

def create_pole():
    """Создаёт столб — вертикальный цилиндр."""
    radius = random.uniform(0.1, 0.25)
    height = random.uniform(2.0, 5.0)
    x, y = random_position()

    handle = sim.createPrimitiveShape(
        sim.primitiveshape_cylinder,
        [radius, radius, height],
        0
    )
    sim.setObjectPosition(handle, sim.handle_world, [x, y, height/2])
    sim.setObjectAlias(handle, f'Obstacle_Pole_{random.randint(1000,9999)}')
    sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, [0.3, 0.5, 0.3])
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 1)
    return handle

def create_obstacles(num_buildings: int, num_walls: int, num_poles: int):
    obstacles = []
    for _ in range(num_buildings):
        obstacles.append(create_building())
    for _ in range(num_walls):
        obstacles.append(create_wall())
    for _ in range(num_poles):
        obstacles.append(create_pole())
    return obstacles

# ---------- ТОЧКИ ИНТЕРЕСА ----------
def create_poi(size: int, idx):
    """Создаёт точку интереса — сферу-маркер на высоте полёта БПЛА."""
    # Точки интереса размещаем по краям области, на высоте 2-5 м
    x = random.uniform(-size/2 + 2, size/2 - 2)
    y = random.uniform(-size/2 + 2, size/2 - 2)
    z = random.uniform(2.0, 5.0)

    handle = sim.createPrimitiveShape(
        sim.primitiveshape_spheroid,
        [0.2, 0.2, 0.2],
        0
    )
    sim.setObjectPosition(handle, sim.handle_world, [x, y, z])
    sim.setObjectAlias(handle, f'POI_{idx}')
    sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, [1.0, 0.1, 0.1])
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 0)
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
    return handle

def create_pois(num_poi: int):
    return [create_poi(args.size, i) for i in range(num_poi)]

# ---------- СОХРАНЕНИЕ СЦЕНЫ ----------
def save_scene(path):
    try:
        sim.saveScene(path)
        print(f"Сцена сохранена: {path}")
    except Exception as e:
        print(f"Ошибка сохранения сцены: {e}")



# ---------- MAIN ----------


def main():
    clear_scene()
    create_checkered_floor(args.size)
    create_obstacles(args.buildings, args.walls, args.poles)
    create_pois(args.POI)

    print(f"Сцена сгенерирована: площадь: {args.size}, {args.buildings} зданий, {args.walls} стен, "
          f"{args.poles} столбов, {args.POI} точек интереса.")

    # Сохраняем сцену
    save_scene(os.path.join(os.getcwd(), f'{args.name}.ttt'))

if __name__ == '__main__':
    main()
