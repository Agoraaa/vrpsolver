import random as rng
from VrpModel import *
import GraphAlgos
import networkx as nx
import numpy as np
import math
import time

def _all_possible_matchings(arr):
    res = []
    if len(arr) == 2:
        return [[(arr[0], arr[1])]]
    for i in range(1, len(arr)):
        # pick 0 and i
        matching = (arr[0], arr[i])
        arr.pop(i)
        arr.pop(0)
        sub_res = _all_possible_matchings(arr)
        for sub_matchings in sub_res:
            sub_matchings.append(matching)
        res.extend(sub_res)
        arr.insert(0, matching[0])
        arr.insert(i, matching[1])
    return res

class VrpSolver():
    def __init__(self, model: VrpModel):
        self.model = model
        
    def _mst_to_christofides(self, mst_edges):
        curr_ind = 0
        original_to_index = {}
        index_to_original = []
        for (edge_start, edge_end) in mst_edges:
            if edge_start not in original_to_index:
                original_to_index[edge_start] = curr_ind
                index_to_original.append(edge_start)
                curr_ind += 1
            if edge_end not in original_to_index:
                original_to_index[edge_end] = curr_ind
                index_to_original.append(edge_end)
                curr_ind += 1
        sub_dist_mat = [[0]*len(index_to_original)] * len(index_to_original)
        for i in range(len(index_to_original)):
            for j in range(i+1, len(index_to_original)):
                sub_dist_mat[i][j] =  self.model.dist_mat[index_to_original[i], index_to_original[j]]
                sub_dist_mat[j][i] = self.model.dist_mat[index_to_original[i], index_to_original[j]]
        vertex_degrees = [0] * len(index_to_original)
        for (v1, v2) in mst_edges:
            vertex_degrees[original_to_index[v1]] += 1
            vertex_degrees[original_to_index[v2]] += 1
        odd_degree_edges = []
        for i in range(len(vertex_degrees)):
            if vertex_degrees[i] % 2 == 1:
                odd_degree_edges.append(i)
        matching_graph = nx.Graph()
        for i in range(len(odd_degree_edges)):
            for j in range(i+1, len(odd_degree_edges)):
                matching_graph.add_edge(odd_degree_edges[i], odd_degree_edges[j], weight=sub_dist_mat[odd_degree_edges[i]][odd_degree_edges[j]])
        min_matching_edges = list(nx.min_weight_matching(matching_graph))
        merged_graph = nx.MultiGraph()
        for (edge_start, edge_end) in mst_edges:
            v1 = original_to_index[edge_start]
            v2 = original_to_index[edge_end]
            merged_graph.add_edge(v1,v2)
        for (edge_start, edge_end) in min_matching_edges:
            merged_graph.add_edge(edge_start, edge_end)
        eulerian_circuit = [u for u, v in nx.eulerian_circuit(merged_graph)]
        # shortcut the eulerian circuit
        is_visited = [False] * len(index_to_original)
        tsp_path = []
        for node in eulerian_circuit:
            if is_visited[node]:
                continue
            tsp_path.append(node)
            is_visited[node] = True
        # return to the starting position
        if tsp_path[0] != tsp_path[-1]:
            tsp_path.append(tsp_path[0])
        return [index_to_original[i] for i in tsp_path]

    def construct_random(self):
        cities = [i for i in range(1, len(self.model.demands))]
        rng.shuffle(cities)
        paths = [[0] for i in range(self.model.vehicles)]
        curr_car = 0
        while cities:
            paths[curr_car].append(cities.pop())
            curr_car += 1
            curr_car %= self.model.vehicles
        for p in paths:
            p.append(0)
        return paths
    def construct_v1(self):
        dist_mat = self.model.dist_mat
        # select 2*vehicle edges from depot. currently we select it randomly
        edges = []
        nodes = [i for i in range(1, len(dist_mat))]
        while len(edges) < self.model.vehicles:
            node_to_add = rng.choice(nodes)
            edges.append((0, node_to_add))
            nodes.remove(node_to_add)
        INITIAL_EDGES_TO_ADD = 0
        MAX_DIST_MULTIPLIER = 10.0
        MIN_DIST_MULTIPLIER = 0.0
        average_distance = dist_mat.sum() / (len(dist_mat)**2 - len(dist_mat))
        initial_mst = edges.copy()
        for (root, first_vertex) in edges:
            for _ in range(INITIAL_EDGES_TO_ADD):
                curr_node = first_vertex
                max_dist_multiplier = MAX_DIST_MULTIPLIER
                min_dist_multiplier = MIN_DIST_MULTIPLIER
                while 1:
                    node_to_add = rng.choice(nodes)
                    if(dist_mat[curr_node][node_to_add] > average_distance * max_dist_multiplier) or (dist_mat[curr_node][node_to_add] < average_distance*min_dist_multiplier):
                        max_dist_multiplier += 0.001
                        min_dist_multiplier -= 0.001
                        continue
                    initial_mst.append((curr_node, node_to_add))
                    curr_node = node_to_add
                    nodes.remove(node_to_add)
                    break
        mst = GraphAlgos.minimum_spanning_tree(dist_mat, edges)
        connected_components = {}
        # remove node 0 and calculate connected components, this will give #vehicle MSTs
        counts = [1 for i in range(len(dist_mat))]
        parents = [i for i in range(len(dist_mat))]
        for (edge_start, edge_end) in mst:
            if (edge_start == 0) or (edge_end == 0):
                continue
            GraphAlgos.dsu_merge_sets(edge_start, edge_end, parents, counts)

        for (edge_start, edge_end) in mst:
            if edge_end == 0:
                edge_start, edge_end = edge_end, edge_start
            parent = GraphAlgos.dsu_find_parent(edge_end, parents)
            if parent not in connected_components:
                connected_components[parent] = []
            connected_components[parent].append((edge_start, edge_end))
        
        car_msts = list(connected_components.values())


        solution = [self._mst_to_christofides(mst) for mst in car_msts]
        return solution

    def construct_v2(self):
        dist_mat: np.ndarray = self.model.dist_mat
        mst = GraphAlgos.minimum_spanning_tree(dist_mat[1:, 1:], refuse_chance=0.005)
        for i in range(len(mst)):
            mst[i] = (mst[i][0]+1, mst[i][1]+1)
        loop = self._mst_to_christofides(mst)
        average_capacity = sum(self.model.demands)/self.model.vehicles
        paths = [[0] for i in range(self.model.vehicles)]
        curr_ind = 0
        for i in range(self.model.vehicles):
            used_capacity = 0
            while used_capacity < average_capacity:
                paths[i].append(loop[curr_ind])
                used_capacity += self.model.demands[loop[curr_ind]]
                curr_ind += 1
                if curr_ind >= len(loop)-1:
                    for p in paths:
                        p.append(0)
                    return paths
                

    def _balance_capacity(self, solution):
        heavy_car = solution[0]
        light_car = solution[-1]
        demands = self.model.demands
        max_ind = rng.randint(1, len(heavy_car)-2)
        #max_demand, max_ind = demands[heavy_car[1]], 1
        #for i in range(2, len(heavy_car)-1):
        #    if demands[heavy_car[i]] > max_demand:
        #        max_ind = i
        #        max_demand = demands[heavy_car[i]]
        light_car.insert(len(light_car)-2, heavy_car[max_ind])
        heavy_car.pop(max_ind)

    def opt_2(self, path):
        dist_mat = self.model.dist_mat
        if len(path) < 4:
            return False
        for _ in range(1000):
            v1, v2 = 0, 0
            v1 = rng.randint(2, len(path)-3)
            while 1:
                v2 = rng.randint(2, len(path)-3)
                if abs(v1-v2) > 1:
                    break
            if v2 < v1:
                temp = v2
                v2 = v1
                v1 = temp
            gain = 1* (dist_mat[path[v1]][path[v1+1]] + dist_mat[path[v2]][path[v2+1]]) \
                    - (dist_mat[path[v1]][path[v2]] + dist_mat[path[v1+1]][path[v2+1]])
            if gain > 0:
                #print("Swapping")
                new_path = []
                new_path.extend(path[:(v1+1)])
                new_path.extend(reversed(path[(v1+1):(v2+1)]))
                new_path.extend(path[(v2+1):])
                return new_path
        return path
        

    def opt_3(self, path):
        raise NotImplementedError()

    def make_feasible(self, solution):
        self.sort_solution(solution)
        while not self.is_feasible(solution):
            self._balance_capacity(solution)
            self.sort_solution_bubble(solution)
            
        return solution

    def used_car_capacity(self, car):
        res = 0
        for demand_point in car:
            res += self.model.demands[demand_point]
        return res

    def sort_solution(self, sol):
        sol.sort(key=lambda x: self.used_car_capacity(x), reverse=True)

    def sort_solution_bubble(self, sol):
        for i in range(len(sol)-1):
            if self.used_car_capacity(sol[i]) > self.used_car_capacity(sol[i+1]):
                break
            sol[i], sol[i+1] = sol[i+1], sol[i]
        for i in range(len(sol)-1, 0, -1):
            if self.used_car_capacity(sol[i]) < self.used_car_capacity(sol[i-1]):
                break    
            sol[i], sol[i-1] = sol[i-1], sol[i]

    def find_solution_value(self, vehicle_paths):
        res = 0
        for path in vehicle_paths:
            for i in range(len(path)-1):
                res += self.model.dist_mat[path[i]][path[i+1]] 
        res += self.model.dist_mat[path[-1]][0]
        return res

    def optimize(self, sol):
        sol[0] = self.opt_2(sol[0])
        sol[1] = self.opt_2(sol[1])
        sol[2] = self.opt_2(sol[2])
        sol[3] = self.opt_2(sol[3])

    def is_feasible(self, sol):
        timesVisited = [0] * len(self.model.demands)
        timesVisited[0] = 1
        used_capacities = [0] * self.model.vehicles
        for car_ind in range(len(sol)):
            car = sol[car_ind]
            for node in car[1:-1]:
                used_capacities[car_ind] += self.model.demands[node]
                timesVisited[node] += 1
        for i in timesVisited[1:]:
            if i != 1:
                print("Wadafak")
                raise Exception()
                return False
        return max(used_capacities) <= self.model.capacity
    
    def test_optimizer(self, sol, iteration_limit = None, time_limit = None): # paradox of analysis, for very fast optimizers might not be very accurate
        start_z = self.find_solution_value(sol)
        if time_limit is not None:
            res = [0] * round((time_limit * 100)) # gain in 0.01th second
            for i in range(len(res)):
                prev_z = self.find_solution_value(sol)
                prev_ts = time.time()
                while time.time() - prev_ts < 0.01:
                    self.optimize(sol)
                new_z = self.find_solution_value(sol)
                res[i] = (new_z - prev_z)/start_z
            print("Gain in every 0.01th second: ")
            for (i, gain) in enumerate(res):
                print(f'{i+1}: {100*gain}%')
            print(f'Started from -> optimized to: {start_z}->{self.find_solution_value(sol)}')
            print(f'Total gain: {100*self.find_solution_value(sol)/start_z}%')
            return res
                
            
        else:
            raise NotImplementedError()

    def print_solution(self, sol):
        for path in sol:
            capacity = 0
            for i in range(len(path)-1):
                print(f'{path[i]}->', end='')
                capacity += self.model.demands[path[i]]
            print(f'0 Total used capacity: {capacity}\n') 
        print(f'Total distance: {self.find_solution_value(sol)}')
        


        