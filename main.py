from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from ModelCreator import *
import GoogleSolver
import json
import GraphAlgos
import VrpSolver
import time
import parameters

def main():
    if False:
        model_config = ModelCreatorConfig(width=5000, height=5000, num_nodes=100, num_vehicles=4, vehicle_capacity=120, max_demand=8)
        new_model = ModelCreator.create_rectangular(model_config)
        with open("new_model.json", 'w+') as fil:
            fil.write(new_model.to_json())
        print("Created new_model.json")
        return
        
    model = VrpModel.from_json(json.load(open("model.json")))
    solver = VrpSolver.VrpSolver(model)
    optimal_value = GoogleSolver.GoogleSolver.solve(model, is_print = False)
    start_time = time.time()
    result = solver.solve(time_limit=30)
    time_took = time.time() - start_time
    solver.is_feasible(result)
    solver.print_solution(result)
    print(f'{solver.find_solution_value(result)} our best solution, {optimal_value} ortools\'. Total time: {"%.3f" % time_took} seconds')
    worse_percentage = ((solver.find_solution_value(result)-optimal_value)/optimal_value) * 100
    print(f'{"%.3f" % worse_percentage}% worse than ortools')
    


if __name__ == "__main__":
    main()