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
    #GoogleSolver.GoogleSolver.solve(model)
    accumulated_total_distances = 0
    accumulated_time = 0
    best_distance = 10**10
    best_solution = []
    optimizer_gains = [0] * 100
    ITER_COUNT = 10
    print(f'Running {ITER_COUNT} tests...')
    solver = VrpSolver.VrpSolver(model)
    for _ in range(ITER_COUNT):
        start_ts = time.time()
        vehicle_paths = solver.construct_v1()
        vehicle_paths = solver.optimize(vehicle_paths)
        #curr_z = solver.find_solution_value(vehicle_paths)
        while time.time() - start_ts < 1:
            for i in range(25):
                vehicle_paths[0] = solver.opt_2(vehicle_paths[0])
                vehicle_paths[1] = solver.opt_2(vehicle_paths[1])
                vehicle_paths[2] = solver.opt_2(vehicle_paths[2])
                vehicle_paths[3] = solver.opt_2(vehicle_paths[3])
                #new_z = solver.find_solution_value(vehicle_paths)
                #optimizer_gains[i] += new_z - curr_z
                #curr_z = new_z

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
    