import random as rng
from enum import Enum
from VrpModel import VrpModel
class DistanceFunction(Enum):
    Manhattan = 1,
    Euclidean = 2,


class ModelCreatorConfig:
    def __init__(self, width=5000, height=5000, num_nodes=100, num_vehicles=4, distance_function: DistanceFunction = None, vehicle_capacity=120, max_demand=8):
        if distance_function is None:
            self.distance_function = DistanceFunction.Euclidean
        else:
            self.distance_function = distance_function
        self.width = width
        self.height = height
        self.num_nodes = num_nodes
        self.num_vehicles = num_vehicles
        self.vehicle_capacity = vehicle_capacity
        self.max_demand = max_demand
        
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def dist_to(self, p2, dist_func: DistanceFunction):
        if dist_func == DistanceFunction.Manhattan:
            return abs(self.x - p2.x) + abs(self.y - p2.y)
        elif dist_func == DistanceFunction.Euclidean:
            return (
                (self.x-p2.x)**2 + \
                (self.y-p2.y)**2   \
                )**(1/2)
class ModelCreator:
    def create_rectangular(config: ModelCreatorConfig = None):
        if config is None:
            config = ModelCreatorConfig()
        node_coordinates = []
        node_coordinates.append(Point(config.width/2, config.height/2))
        demands = []
        # dummy demand for depot
        demands.append(0)
        for i in range(config.num_nodes-1):
            new_x = rng.uniform(0, config.width)
            new_y = rng.uniform(0, config.height)
            node_coordinates.append(Point(new_x, new_y))
            # maybe make demand gaussian
            demands.append(rng.randint(1, config.max_demand))
        dist_mat = []
        for i in range(config.num_nodes):
            dist_mat.append([0]* config.num_nodes)

        for i in range(config.num_nodes-1):
            for j in range(i+1, config.num_nodes):
                dist = round(node_coordinates[i].dist_to(node_coordinates[j], config.distance_function))
                dist_mat[i][j] = dist
                dist_mat[j][i] = dist
        return VrpModel(dist_mat, config.num_vehicles, config.vehicle_capacity, demands)

        

        