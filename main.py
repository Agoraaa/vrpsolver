from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from ModelCreator import *
import GoogleSolver
import json
import GraphAlgos
import VrpSolver
import time

def main():
    model = VrpModel.from_json(json.load(open("model.json")))
    solver = VrpSolver.VrpSolver(model)
    #GoogleSolver.GoogleSolver.solve(model)
    #for _ in range(100):
    #    vehicle_paths = solver.construct_v1()
    #    vehicle_paths = solver.make_feasible(vehicle_paths)
    #    solver.test_optimizer(vehicle_paths, time_limit=0.2)
    #return


    accumulated_total_distances = 0
    accumulated_time = 0
    cumulative_time = [0] * 100
    best_distance = 10**10
    best_solution = []
    ITER_COUNT = 50
    print(f'Running {ITER_COUNT} tests...')
    for test_num in range(ITER_COUNT):
        print(test_num)
        start_ts = time.time()
        vehicle_paths = solver.construct_v1()
        vehicle_paths = solver.make_feasible(vehicle_paths)
        time_gains = solver.test_optimizer(vehicle_paths, time_limit=0.5, print_res=False)
        time_took = time.time() - start_ts
        total_dist = solver.find_solution_value(vehicle_paths)
        if total_dist < best_distance:
            best_solution = vehicle_paths
            best_distance = total_dist
        accumulated_total_distances += total_dist
        accumulated_time += time_took
        for i in range(len(time_gains)):
            cumulative_time[i] += time_gains[i]
    print("Test ended.")
    print("-"*15)
    average_opt_gains = [100*i/ITER_COUNT for i in cumulative_time]
    print("Reduction in every 0.01th second: ")
    for (i, gain) in enumerate(average_opt_gains):
        print(f'Reduction at {i+1}: {"%.3f" % gain}%')
    average_solution_distance = accumulated_total_distances / ITER_COUNT
    average_time = accumulated_time / ITER_COUNT
    solver.is_feasible(best_solution)
    solver.print_solution(best_solution)
    print(f'Ran {ITER_COUNT} tests. {average_solution_distance} Average, {best_distance} Best. Average time: {average_time}')
    


if __name__ == "__main__":
    main()
    