#!/usr/bin/env python

import pandas as pd
import numpy as np
import rospy

csv = pd.read_csv("../resources/reward_table.csv")
print(csv)

array1 = csv.values
print(array1[1,2])