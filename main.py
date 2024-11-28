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
    for _ in range(10):
        print("-"*15)
        vehicle_paths = solver.construct_v1()
        vehicle_paths = solver.make_feasible(vehicle_paths)
        solver.test_optimizer(vehicle_paths, time_limit=0.2)
    return


    accumulated_total_distances = 0
    accumulated_time = 0
    best_distance = 10**10
    best_solution = []
    ITER_COUNT = 10
    print(f'Running {ITER_COUNT} tests...')
    for _ in range(ITER_COUNT):
        start_ts = time.time()
        vehicle_paths = solver.construct_v1()
        vehicle_paths = solver.make_feasible(vehicle_paths)
        #curr_z = solver.find_solution_value(vehicle_paths)
        
        time_took = time.time() - start_ts
        
        total_dist = solver.find_solution_value(vehicle_paths)
        if total_dist < best_distance:
            best_solution = vehicle_paths
            best_distance = total_dist
        accumulated_total_distances += total_dist
        accumulated_time += time_took
    
    #average_opt_gains = [i/ITER_COUNT for i in optimizer_gains]
    #for (i, gain) in enumerate(average_opt_gains):
    #    print(f'Gain at {i+1}: {gain}')
    average_solution_distance = accumulated_total_distances / ITER_COUNT
    average_time = accumulated_time / ITER_COUNT
    solver.is_feasible(best_solution)
    solver.print_solution(best_solution)
    print(f'Ran {ITER_COUNT} tests. {average_solution_distance} Average, {best_distance} Best. Average time: {average_time}')
    


if __name__ == "__main__":
    main()
    