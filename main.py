from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from ModelCreator import *
import GoogleSolver
import json



def main():
    model = VrpModel.from_json(json.load(open("model.json")))
    GoogleSolver.GoogleSolver.solve(model)
    


if __name__ == "__main__":
    main()