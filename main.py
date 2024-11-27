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
    ITER_COUNT = 1
    for _ in range(ITER_COUNT):
        start_ts = time.time()
        vehicle_paths = VrpSolver.construct(model)
        time_took = time.time() - start_ts
        total_dist = 0
        for path in vehicle_paths:
            for i in range(len(path)-1):
                total_dist += model.dist_mat[path[i]][path[i+1]]
            total_dist += model.dist_mat[path[-1]][0]
        if total_dist < best_distance:
            best_solution = vehicle_paths
            best_distance = total_dist
        accumulated_total_distances += total_dist
        accumulated_time += time_took
    average_solution_distance = accumulated_total_distances / ITER_COUNT
    average_time = accumulated_time / ITER_COUNT
    
    total_dist = 0
    for path in best_solution:
        capacity = 0
        for i in range(len(path)-1):
            print(f'{path[i]}->', end='')
            capacity += model.demands[i]
            total_dist += model.dist_mat[path[i]][path[i+1]]
        print(f'0 Total used capacity: {capacity}\n') 
        total_dist += model.dist_mat[path[-1]][0]
    print(f'Total distance: {best_distance}')
    
        
        
    print(f'Ran {ITER_COUNT} tests. {average_solution_distance} Average, {best_distance} Best. Average time: {average_time}')
    


if __name__ == "__main__":
    main()
    