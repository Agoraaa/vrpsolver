from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from ModelCreator import *
import GoogleSolver
import json
from GraphAlgos import GraphAlgos
from VrpSolver import VrpSolver


def main():
    model = VrpModel.from_json(json.load(open("model.json")))
    GoogleSolver.GoogleSolver.solve(model)
    print('-' * 20)
    vehicle_paths = VrpSolver.construct(model)
    total_dist = 0
    for path in vehicle_paths:
        for i in range(len(path)-1):
            print(f'{path[i]}->', end='')
            total_dist += model.dist_mat[path[i]][path[i+1]]
        print('0\n')
        total_dist += model.dist_mat[path[-1]][0]
    print(f'Total distance: {total_dist}')
    


if __name__ == "__main__":
    main()
    