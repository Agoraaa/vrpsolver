import json
import numpy as np
class VrpModel():
    def __init__(self, distance_matrix, num_vehicles, vehicle_capacity, demands, names=None):
        self.dist_mat = distance_matrix
        self.vehicles = num_vehicles
        self.capacity = vehicle_capacity
        self.demands = demands
        if names is None:
            names = [f'City{i}' for i in range(0, len(distance_matrix))]
        self.names = names
    def to_json(self):
        res_dict = {}
        res_dict["nodeNames"] = self.names
        res_dict["vehicleCapacities"] = self.capacity
        res_dict["numberOfVehicles"] = self.vehicles
        res_dict["depotName"] = self.names[0]
        res_dict["demands"] = {}
        for i in range(len(self.demands)):
            res_dict['demands'][self.names[i]] = int(self.demands[i])
        res_dict["distances"] = {}
        for i in range(0, len(self.dist_mat)-1):
            res_dict["distances"][self.names[i]] = {}
            for j in range(i+1, len(self.dist_mat)):
                res_dict["distances"][self.names[i]][self.names[j]] = int(self.dist_mat[i][j])
        return json.dumps(res_dict, indent=5)

    def from_json(json_dict):
        node_cnt = len(json_dict["demands"])
        name_to_ind = {}
        name_to_ind[json_dict["depotName"]] = 0
        curr_ind = 1
        for name in json_dict["nodeNames"]:
            if name == json_dict["depotName"]:
                continue
            name_to_ind[name] = curr_ind
            curr_ind+=1
        demands = [0] * node_cnt
        for city in json_dict["demands"]:
            demands[name_to_ind[city]] = json_dict["demands"][city]
        dist_mat = []
        for i in range(node_cnt):
            dist_mat.append([0] * node_cnt)
        for city1 in json_dict["distances"]:
            for city2 in json_dict["distances"][city1]:
                i = name_to_ind[city1]
                j = name_to_ind[city2]
                dist_mat[i][j]=json_dict["distances"][city1][city2]
                dist_mat[j][i]=json_dict["distances"][city1][city2]
        dist_mat = np.array(dist_mat)
        np.random.normal()
        return VrpModel(dist_mat, json_dict["numberOfVehicles"], json_dict["vehicleCapacities"], demands, json_dict["nodeNames"])


            