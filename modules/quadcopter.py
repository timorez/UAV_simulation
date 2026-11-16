from __future__ import annotations
import math
import asyncio
from enum import Enum
from time import monotonic


class State(Enum):
    IDLE = 0
    MOVING = 1
    NOT_RESPONDING = 3


class TaskStatus(Enum):
    FREE = 0
    IN_PROGRESS = 1
    FINISHED = 2


class POI:
    def __init__(self, poi):
        self.index = poi[0]
        self.pos = poi[1]
        self.status = TaskStatus.FREE

    def __repr__(self):
        return f'{self.index}: {self.pos}, {self.status}'


class Quadcopter:
    def __init__(self, path: str, all_pois: list[POI], sim_proxy, async_sim_call, speed: float = 15, rotation_speed: float = 5):
        self.sim = sim_proxy
        self.async_sim_call = async_sim_call
        try:
            self.handle = self.sim.getObject(path)
            self.tgt_handle = self.sim.getObject(f'{path}/target')
            self.altimeter = self.sim.getObject(f'{path}/proximitySensor')
            self.front_camera = self.sim.getObject(f'{path}/visionSensor')
            self.accelerometer = self.sim.getObject(f'{path}/Accelerometer/forceSensor')
            self.gyro = self.sim.getObject(f'{path}/GyroSensor')
        except Exception as e:
            raise RuntimeError('Не удалось подключиться к дрону') from e

        self.accel_bias = [0.0, 0.0, 0.0]
        self.gyro_bias = [0.0, 0.0, 0.0]
        
        self.gravity = [0.0, 0.0, -9.81]

        self.pos = self.sim.getObjectPosition(self.tgt_handle, self.sim.handle_world)
        self.state = State.IDLE
        self.speed = speed
        self.rotation_speed = rotation_speed
        self.step_time = 0.005
        self.tolerance = 0.25
        self.ping_timestamp = monotonic()

        self.known_pois = {
            poi.index: {'position': poi.pos, 'status': poi.status, 'by_whom': None}
            for poi in all_pois
        }

        self.claimed_tgt = None
        self.network: dict[int, dict[str, State | list | int | Quadcopter | float | None]] = {}
        """Инфа о соседях {handle: {state, position, claimed_tgt, inst, timestamp}}"""

        self._sim_lock = asyncio.Lock()


    def set_network(self, all_drones: list[Quadcopter]):
        """Инициализация сети"""
        for d in all_drones:
            if d.handle != self.handle:
                self.network[d.handle] = {
                    'state': d.state,
                    'position': d.pos,
                    'claimed_tgt': d.claimed_tgt,
                    'inst': d,
                    'timestamp': monotonic()
                }

    async def create_message(self):
        """Формирование сообщения для пинга"""
        return {
                'sender': self.handle,
                'state': self.state,
                'position': await self.get_position(),
                'claimed_tgt': self.claimed_tgt,
                'timestamp': self.ping_timestamp
            }


    async def start(self):
        """Запуск фоновых задач."""
        init_pos = await self.async_sim_call(
            self.sim.getObjectPosition, self.handle, self.sim.handle_world)
        init_quat = await self.async_sim_call(
            self.sim.getObjectQuaternion, self.handle, self.sim.handle_world)

        self.ins_pos = list(init_pos)
        self.ins_vel = [0.0, 0.0, 0.0]
        self.ins_quat = list(init_quat)

        self._tasks = [
            asyncio.create_task(self._ins_loop(),       name='ins'),
            asyncio.create_task(self._ping_loop(),      name='ping'),
            asyncio.create_task(self._targeting_loop(), name='tgt'),
            asyncio.create_task(self._action_loop(),    name='action')
        ]
        print(f'[{self.handle}] Дрон запущен, ИНС активна')


    async def stop(self):
        """Остановка дрона и всех фоновых задач."""
        for t in getattr(self, '_tasks', []):
            if not t.done():
                t.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        print(f'[{self.handle}] Дрон остановлен')


    async def _ins_loop(self):
        """Фоновый цикл обновления ИНС с частотой симуляции."""
        while True:
            await self.update_ins()
            await asyncio.sleep(self.step_time)


    async def _ping_loop(self):
        """Фоновый цикл с пингами"""
        while True:
            self.ping_timestamp = monotonic()
            msg = await self.create_message()
            tasks = [d['inst'].recieve_ping(msg) for d in self.network.values()]
            await asyncio.gather(*tasks, return_exceptions=True)

            for handle, info in self.network.items():
                if info['state'] != State.NOT_RESPONDING:
                    if self.ping_timestamp - info['timestamp'] > 5:
                        info['state'] = State.NOT_RESPONDING
                        print(f'[{self.handle}]: дрон {handle} помечен как вышедший из строя т.к не обновлял свой статус в течении 15 секунд')
                    else:
                        print(f'[{self.handle}]: Активные дроны: {self.network.keys()}')

            
            await asyncio.sleep(2)


    async def _targeting_loop(self):
        """Цикл выбора целей"""
        while True:
            if self.state == State.IDLE:
                if len(self.known_pois) == 0:
                    print(f'[{self.handle}]: Нет известных целей, завершаю цикл поиска')
                    break

                claimed_idx = None
                distance = None
                for tgt_idx, tgt_info in self.known_pois.items():
                    if tgt_info['status'] == TaskStatus.FREE:
                        d = await self.get_distance(*tgt_info['position'])
                        if claimed_idx is None:
                            claimed_idx = tgt_idx
                            distance = d
                        elif d < distance:
                            claimed_idx = tgt_idx
                            distance = d

                response = []
                
                for handle, info in self.network.items():
                    if info['state'] != State.NOT_RESPONDING:
                        response.append(await info['inst'].recieve_claim(self.handle, claimed_idx, distance))

                if 'REJECTED' in response:
                    print(f'[{self.handle}]: не удалось занять цель {claimed_idx}')
                    continue
                else:
                    print(f'[{self.handle}]: Занимаю цель {claimed_idx}')
                    self.claimed_tgt = claimed_idx

                    self.known_pois[self.claimed_tgt]['status'] = TaskStatus.IN_PROGRESS
                    self.known_pois[self.claimed_tgt]['by_whom'] = self.handle

                    msg = await self.create_message()
                    tasks = [d['inst'].recieve_ping(msg) for d in self.network.values()]
                    await asyncio.gather(*tasks, return_exceptions=True)

            await asyncio.sleep(3)


    async def _action_loop(self):
        """Цикл действий"""
        while True:
            if self.claimed_tgt is not None:
                x, y, z = self.known_pois[self.claimed_tgt]['position']
                arrival = await self.move_to(x, y, z)
                msg = await self.create_message()
                
                tasks = [d['inst'].recieve_ping(msg) for d in self.network.values()]
                await asyncio.gather(*tasks, return_exceptions=True)

                if arrival:
                    print(f'[{self.handle}]: Выполнил цель {self.claimed_tgt}')
                    msg = await self.create_message()
                    tasks = [d['inst'].recieve_done(self.claimed_tgt) for d in self.network.values()]
                    await asyncio.gather(*tasks, return_exceptions=True)

                    tasks = [d['inst'].recieve_ping(msg) for d in self.network.values()]
                    await asyncio.gather(*tasks, return_exceptions=True)
                    self.claimed_tgt = None
                    
            await asyncio.sleep(1)
                

    async def recieve_done(self, tgt):
        """Получение уведомления о выполненой задаче"""
        self.known_pois[tgt]['status'] = TaskStatus.FINISHED

    async def recieve_ping(self, msg: dict):
        """Получение обновлений от дронов из сети"""
        self.network[msg['sender']]['state'] = msg['state']
        self.network[msg['sender']]['position'] = msg['position']
        self.network[msg['sender']]['claimed_tgt'] = msg['claimed_tgt']
        self.network[msg['sender']]['timestamp'] = msg['timestamp']
        if msg['claimed_tgt'] is not None and msg['claimed_tgt'] in self.known_pois:
            self.known_pois[msg['claimed_tgt']]['by_whom'] = msg['sender']
            self.known_pois[msg['claimed_tgt']]['status'] = TaskStatus.IN_PROGRESS


    async def recieve_claim(self, sender_handle, claimed_idx, claimed_distance):
        """Получение заявки на цель"""
        distance = await self.get_distance(*self.known_pois[claimed_idx]['position'])

        if self.state == State.IDLE and distance < claimed_distance:
            return 'REJECTED'
        else:
            return 'ACCEPTED'
    

    async def get_position(self):
        return await self.async_sim_call(
            self.sim.getObjectPosition, self.handle, self.sim.handle_world)

    async def angle_to_target(self, target_pos):
        drone_pos = await self.get_position()
        dx = target_pos[0] - drone_pos[0]
        dy = target_pos[1] - drone_pos[1]
        return math.atan2(dy, dx)
    

    async def yaw(self, yaw_target: float, timeout: float = 10.0, wait_for_drone: bool = True, tol: float = 0.02) -> bool:
        tquat = await self.async_sim_call(
            self.sim.getObjectQuaternion, self.tgt_handle, self.sim.handle_world)
        yaw = 2.0 * math.atan2(tquat[2], tquat[3])    # qz, qw

        w_step = self.rotation_speed * self.step_time
        t0 = await self.async_sim_call(self.sim.getSimulationTime)

        while True:
            if (await self.async_sim_call(self.sim.getSimulationTime)) - t0 > timeout:
                return False

            dyaw = math.atan2(math.sin(yaw_target - yaw),
                            math.cos(yaw_target - yaw))
            if abs(dyaw) < 1e-3:
                break

            yaw += max(-w_step, min(w_step, dyaw))

            half = yaw * 0.5
            await self.async_sim_call(
                self.sim.setObjectQuaternion,
                self.tgt_handle, self.sim.handle_world,
                [0.0, 0.0, math.sin(half), math.cos(half)])

            await asyncio.sleep(self.step_time)

        if wait_for_drone:
            while True:
                if (await self.async_sim_call(self.sim.getSimulationTime)) - t0 > timeout:
                    return False

                dquat = await self.async_sim_call(
                    self.sim.getObjectQuaternion, self.handle, self.sim.handle_world)
                drone_yaw = 2.0 * math.atan2(dquat[2], dquat[3])

                dyaw = math.atan2(math.sin(yaw_target - drone_yaw),
                                math.cos(yaw_target - drone_yaw))

                if abs(dyaw) < tol:
                    return True

                await asyncio.sleep(self.step_time)

        return True
    

    async def move_to(self, x: float, y: float, z: float, timeout: float = 15.0, yaw: bool = True):
        target_pos = [x, y, z]
        start_pos = await self.async_sim_call(self.sim.getObjectPosition, self.tgt_handle, self.sim.handle_world)
        distance = math.sqrt(sum((target_pos[i] - start_pos[i]) ** 2 for i in range(3)))
        steps = max(1, math.ceil(distance / (self.speed * self.step_time)))

        self.state = State.MOVING
        if yaw:
            await self.yaw(await self.angle_to_target(target_pos))
        try:
            for step in range(1, steps + 1):
                ratio = step / steps
                current_pos = [
                    start_pos[i] + (target_pos[i] - start_pos[i]) * ratio
                    for i in range(3)
                ]
                await self.async_sim_call(self.sim.setObjectPosition, self.tgt_handle, self.sim.handle_world, current_pos)
                self.pos = current_pos
                await asyncio.sleep(self.step_time)

            deadline = monotonic() + timeout
            while True:
                current_pos = await self.get_position()
                remaining = math.sqrt(sum((target_pos[i] - current_pos[i]) ** 2 for i in range(3)))
                if remaining <= self.tolerance:
                    return True
                if monotonic() >= deadline:
                    return False
                await asyncio.sleep(self.step_time)
        finally:
            self.state = State.IDLE


    async def move_vector(self, x: float = 0, y: float = 0, z: float = 0):
        """Перемещает коптер на заданный вектор относительно target."""
        await self.move_to(self.pos[0] + x, self.pos[1] + y, self.pos[2] + z)
        

    async def change_alt(self, alt: float):
        await self.move_to(self.pos[0], self.pos[1], self.pos[2] + alt)


    async def get_distance(self, x: float, y: float, z: float):
        target_pos = [x, y, z]
        self_pos = await self.get_position()
        distance = math.sqrt(sum((target_pos[i] - self_pos[i]) ** 2 for i in range(3)))
        return distance

    def get_alt(self):
        detected, distance, *_ = self.sim.readProximitySensor(self.altimeter)
        return distance if detected else None


    @staticmethod
    def _quat_multiply(q1, q2):
        """Умножение кватернионов [x, y, z, w]."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return [
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
        ]

    @staticmethod
    def _quat_normalize(q):
        n = math.sqrt(sum(c*c for c in q))
        if n < 1e-9:
            return [0.0, 0.0, 0.0, 1.0]
        return [c / n for c in q]

    @staticmethod
    def _quat_rotate_vector(q, v):
        """Поворот вектора v кватернионом q [x,y,z,w]."""
        x, y, z, w = q
        # q * v * q_conjugate
        vx, vy, vz = v
        # t = 2 * cross(q.xyz, v)
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return [
            vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx),
        ]


    async def update_ins(self):
        dt = self.step_time

        async with self._sim_lock:
            ax = await self.async_sim_call(self.sim.getFloatSignal, 'accelX')
            ay = await self.async_sim_call(self.sim.getFloatSignal, 'accelY')
            az = await self.async_sim_call(self.sim.getFloatSignal, 'accelZ')
            
            gx = await self.async_sim_call(self.sim.getFloatSignal, 'gyroX')
            gy = await self.async_sim_call(self.sim.getFloatSignal, 'gyroY')
            gz = await self.async_sim_call(self.sim.getFloatSignal, 'gyroZ')

            true_pos = await self.async_sim_call(
                self.sim.getObjectPosition, self.handle, self.sim.handle_world)

        # Если сигналы еще не инициализированы (None), пропускаем шаг
        if ax is None or gx is None:
            return

        accel_raw = [ax, ay, az]
        gyro_raw = [gx, gy, gz]

        # Математика
        accel = [accel_raw[i] - self.accel_bias[i] for i in range(3)]
        gyro = [gyro_raw[i] - self.gyro_bias[i] for i in range(3)]

        wx, wy, wz = gyro
        angle = math.sqrt(wx*wx + wy*wy + wz*wz) * dt
        if angle > 1e-9:
            half_angle = angle * 0.5
            s = math.sin(half_angle) / (angle * 0.5)
            delta_q = [wx * dt * 0.5 * s,
                       wy * dt * 0.5 * s,
                       wz * dt * 0.5 * s,
                       math.cos(half_angle)]
        else:
            delta_q = [wx * dt * 0.5, wy * dt * 0.5, wz * dt * 0.5, 1.0]

        self.ins_quat = self._quat_normalize(
            self._quat_multiply(self.ins_quat, delta_q)
        )

        accel_world = self._quat_rotate_vector(self.ins_quat, accel)
        accel_world = [accel_world[i] + self.gravity[i] for i in range(3)]

        for i in range(3):
            self.ins_vel[i] += accel_world[i] * dt
            self.ins_pos[i] += self.ins_vel[i] * dt

        true_pos = await self.async_sim_call(
            self.sim.getObjectPosition, self.handle, self.sim.handle_world)

        alpha_pos = 0.3
        alpha_vel = 0.5
        for i in range(3):
            pos_err = true_pos[i] - self.ins_pos[i]
            self.ins_pos[i] += pos_err * alpha_pos
            self.ins_vel[i] += pos_err * alpha_vel

    def get_ins_position(self):
        """Возвращает текущую оценку позиции от ИНС."""
        return list(self.ins_pos)
