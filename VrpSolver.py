import random as rng
from VrpModel import *
import GraphAlgos
import networkx as nx
import numpy as np
import math

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

    def construct_v2(self):  # this gives worse results than v1, i have no idea why
        dist_mat: np.ndarray = self.model.dist_mat
        mst = GraphAlgos.minimum_spanning_tree(dist_mat[1:, 1:])
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

    def opt_3(self, path):
        def move_city_dist(new_place, city):
            return self.model.dist_mat[path[new_place-1]][city] + self.model.dist_mat[city][path[new_place+1]]
        def move_cities_dist(new_places, old_places):
            res = 0
            for i in range(len(old_places)):
                res += move_city_dist(new_places[i], path[old_places[i]])
            return res
        MAX_TRIES = 1000
        for _ in range(MAX_TRIES):
            i, j, k = 0, 0, 0
            i = rng.randint(1, len(path)-2)
            while 1:
                j = rng.randint(1, len(path)-2)
                if abs(j - i) > 0:
                    break
            while 1:
                k = rng.randint(1, len(path)-2)
                if (abs(k - i)>0) and (abs(k - j)>0):
                    break
            old_dist = move_cities_dist([i, j, k], [i, j, k])
            possible_swaps = [[k, i, j], [j, k, i], [i, k, j], [j, i, k], [k, j, i]]
            for possible_swap in possible_swaps:
                new_total_dist = move_cities_dist([i, j, k], possible_swap) 
                if new_total_dist < old_dist:
                    #print(f'Swapping {i}, {j}, {k} to {possible_swap[0]}, {possible_swap[1]}, {possible_swap[2]}')
                    # to not lose values while swapping
                    pathi = path[possible_swap[0]]
                    pathj = path[possible_swap[1]]
                    pathk = path[possible_swap[2]]
                    path[i] = pathi
                    path[j] = pathj
                    path[k] = pathk
                    return True
        return False

    def optimize(self, solution):
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
    
    def print_solution(self, sol):
        for path in sol:
            capacity = 0
            for i in range(len(path)-1):
                print(f'{path[i]}->', end='')
                capacity += self.model.demands[path[i]]
            print(f'0 Total used capacity: {capacity}\n') 
        print(f'Total distance: {self.find_solution_value(sol)}')
        


        