from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from ModelCreator import *
import GoogleSolver
import json
import GraphAlgos
import VrpSolver
import time
from SolutionExtensions import *

def main():
    model = VrpModel.from_json(json.load(open("model.json")))
    #GoogleSolver.GoogleSolver.solve(model)
    accumulated_total_distances = 0
    accumulated_time = 0
    best_distance = 10**10
    best_solution = []
    ITER_COUNT = 100
    print(f'Running {ITER_COUNT} tests...')
    solver = VrpSolver.VrpSolver(model)
    for _ in range(ITER_COUNT):
        start_ts = time.time()
        vehicle_paths = solver.construct()
        vehicle_paths = solver.optimize(vehicle_paths)
        curr_z = solver.find_solution_value(vehicle_paths)
        for _ in range(100):
            solver.opt_3(vehicle_paths[0])
            solver.opt_3(vehicle_paths[1])
            solver.opt_3(vehicle_paths[2])
            solver.opt_3(vehicle_paths[3])

        time_took = time.time() - start_ts
        
        total_dist = solver.find_solution_value(vehicle_paths)
        if total_dist < best_distance:
            best_solution = vehicle_paths
            best_distance = total_dist
        accumulated_total_distances += total_dist
        accumulated_time += time_took
    average_solution_distance = accumulated_total_distances / ITER_COUNT
    average_time = accumulated_time / ITER_COUNT
    solver.is_feasible(best_solution)
    solver.print_solution(best_solution)
    print(f'Ran {ITER_COUNT} tests. {average_solution_distance} Average, {best_distance} Best. Average time: {average_time}')
    


if __name__ == "__main__":
    main()
    