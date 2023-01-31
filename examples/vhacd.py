import pybullet as p
import pybullet_data as pd
import os

p.connect(p.DIRECT)
name_in = "../models/office/office.obj"
name_out = "office_vhacd2.obj"
name_log = "log.txt"
p.vhacd(name_in, name_out, name_log)