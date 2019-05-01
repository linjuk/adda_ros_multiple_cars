#! /usr/bin/python
from tabulate import tabulate
from functions import all_prediction_processing, distance_formula

result_set_A = all_prediction_processing(number_point=10)
result_set_B = all_prediction_processing(number_point=100, apply_power=True, power=0.1)  # 0.1 --> 1/(100/10) = 1/10 = 0.1


