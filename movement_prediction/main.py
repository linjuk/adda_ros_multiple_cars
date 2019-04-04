#! /usr/bin/python
from tabulate import tabulate
from functions import all_prediction_processing, distance_formula

result_set_A = all_prediction_processing(number_point=10)
result_set_B = all_prediction_processing(number_point=100, apply_power=True, power=0.1)  # 0.1 --> 1/(100/10) = 1/10 = 0.1

print("\nresult_set_A")
print tabulate(result_set_A, headers=['Time step', 'Coordinates [x,y]', 'Likelihood(pdf) [left, right, straight]', 'Updated belief [left, right, straight]'])

print("\nresult_set_B")
print tabulate(result_set_B, headers=['Time step', 'Coordinates [x,y]', 'Likelihood(pdf) [left, right, straight]', 'Updated belief [left, right, straight]'])

# nested loop to compare each value of result set A with each value of result set B
# and use that comparison to test out different conditions like same coordinates, similar coordinates etc.
for time_step_A in range(len(result_set_A)):
    for time_step_B in range(len(result_set_B)):

        # testing condition 1: find same coordinates
        matching_coordinates = set(result_set_A[time_step_A][1]).intersection(result_set_B[time_step_B][1])
        if len(matching_coordinates) > 0:
            print("\nMatch found, result_set_A point {} and result_set_B point {}".format(time_step_A, time_step_B))
            print tabulate([result_set_A[time_step_A], result_set_B[time_step_B]], headers=['Time step', 'Coordinates [x,y]', 'Likelihood(pdf) [left, right, straight]', 'Updated belief [left, right, straight]'])
            break

        # testing condition 2: find similar coordinates i.e distance < 0.05, 0.009, 0.1 etc.
        distance_between_coordinates = distance_formula(result_set_A[time_step_A][1], result_set_B[time_step_B][1])
        if len(matching_coordinates) == 0 and distance_between_coordinates < 0.1:
            print("\nSimilar coordinates found, result_set_A point {} and result_set_B point {} with a distance of {}".format(time_step_A, time_step_B, distance_between_coordinates))
            print tabulate([result_set_A[time_step_A], result_set_B[time_step_B]], headers=['Time step', 'Coordinates [x,y]', 'Likelihood(pdf) [left, right, straight]', 'Updated belief [left, right, straight]'])
