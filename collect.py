import numpy as np
from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Accelerometer, State
import random
from time import sleep
from beamngpy.sensors import Electrics
import pandas as pd
from beamngpy.sensors import Timer

SIZE = 1024


def save_acc_csv(Adata, Sdata, Eata, u, tims):
    datas = {"t_b": u, "ver": np.sqrt(Sdata["vel"][1] ** 2 + Sdata["vel"][0] ** 2), "accel": Adata["axis1"],
             "x1": Sdata["pos"][0], "x2": Sdata["pos"][1], "x3": Sdata["pos"][2],
             "d1": Sdata["dir"][0], "d2": Sdata["dir"][1], "d3": Sdata["dir"][2],
             "v1": Sdata["vel"][0], "v2": Sdata["vel"][1], "v3": Sdata["vel"][2],
             "fuel": Eata["fuel"], "ws": Eata["wheelspeed"], "times": tims['time']}
    path = ".\\Data\\Itera.csv"
    df = pd.DataFrame(datas, index=[0])
    df.to_csv(path, encoding="gbk", mode='a', index=False, header=False)


def main():
    for j in range(5, 30):
        # set_up_simple_logging()
        # beamng = BeamNGpy('localhost', 64256)
        beamng = BeamNGpy('', 64256, home='',
                          user='')
        bng = beamng.open(launch=True)

        scenario = Scenario('italy_snowy', 'accelerometer_demo',
                            description='Spanning the map with an accelerometer sensor')  # driver_training smallgrid

        state = State()
        ele = Electrics()
        tim = Timer()
        vehicle = Vehicle('ego_vehicle', model='Model3', licence='RED', color='Red')  # c63sedan etk800
        vehicle.attach_sensor('newstate', state)
        vehicle.attach_sensor('electrics', ele)
        vehicle.attach_sensor('timer', tim)

        # scenario.add_vehicle(vehicle, pos=(240.46867867177207, 64.39849100223728, 50), rot_quat=(0, 0, 0, 0))# 大圆圈位置
        # scenario.add_vehicle(vehicle, pos=(0, 0, 100.8), rot_quat=(0, 0, 0, 0))
        scenario.add_vehicle(vehicle, pos=(-1312.383516797796, 1499.5895585417747, 164),
                             rot_quat=(0, 0, 0, 1))

        scenario.make(bng)

        bng.set_deterministic()
        bng.set_steps_per_second(50)  # Set simulator to 60hz temporal resolution

        bng.load_scenario(scenario)
        # bng.hide_hud()
        bng.start_scenario()

        # NOTE: Create sensor after scenario has started.
        accel = Accelerometer('accel1', bng, vehicle, requested_update_time=0.02)

        # accel = Accelerometer('accel1', bng, vehicle, requested_update_time=0.01, pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1)) # if we want to specify a local frame

        # vehicle.ai_set_mode('span')
        print('Driving around, polling the accelerometer sensor at regular intervals...')
        while True:
            # sleep(0.4)
            # # i = j * 0.01
            # vehicle.control(throttle=i)
            # vehicle.control(brake=0)
            vehicle.poll_sensors()
            data = accel.poll()  # Fetch the latest readings from the sensor.
            # print(str(ele.data["fuel"]))
            # print(str(tim.data))
            save_acc_csv(data, state.data, ele.data, 0, tim.data)

if __name__ == '__main__':
    main()
