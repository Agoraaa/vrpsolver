import random as rng
from VrpModel import *
import GraphAlgos
import networkx as nx
import numpy as np
import math
import time
import parameters

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

def _pick_geometrically(arr, accept_chance):
    for item in arr:
        if rng.random() < accept_chance:
            return item
    print("Alarm")
    return arr[-1]
    

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
        # select edges for depots, currently we choose randomly because nothing seems to improve the solution
        edges = []
        while len(edges) < self.model.vehicles:
            while 1:
                edge_to_append = rng.randint(1, len(dist_mat[0])-1)
                if (edge_to_append != 0) and (edge_to_append not in edges):
                    break
            edges.append(edge_to_append)
        edges = [(0, v) for v in edges]
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
        start_ts = time.time()
        while time.time() - start_ts < 0.04:
            self.opt_2(loop)
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
    def solve(self, time_limit = 10):
        white_noise_coeff = parameters.RANDOM_NOISE_START
        max_sol_count = parameters.MULTIPLE_SOLUTIONS_CNT
        start_ts = time.time()
        average_edge_dist = sum([sum(self.model.dist_mat[i]) for i in range(0, len(self.model.dist_mat))])/(len(self.model.dist_mat)**2)
        initial_sol = self.construct_v2()
        self.make_feasible(initial_sol)
        solutions = [initial_sol] * max_sol_count # should diversify pretty fast
        construction_ts = time.time() - start_ts
        percentile_results = []
        iter_count = 0
        while time.time() - start_ts < time_limit:
            time_remaining_percentage = (time_limit - (time.time() - start_ts))/time_limit
            noise_std = time_remaining_percentage * white_noise_coeff * average_edge_dist
            if time_remaining_percentage < 0.1:
                # stop diversifying if time is low
                noise_std = 0
            sub_time = time.time()
            while time.time() - sub_time < 0.05:
                iter_count += parameters.SWAP_FREQ + 1
                for solution in solutions:
                    self.optimize(solution, noise_std)
            average_z = sum(self.find_solution_value(sol) for sol in solutions)/len(solutions)
            percentile_results.append(average_z)
        solutions.sort(key=lambda x: self.find_solution_value(x))
        print(f"Iterated f{iter_count} times.")
        return solutions[0]
        
    def _balance_capacity(self, solution):
        heavy_car = solution[0]
        light_car = solution[-1]
        demands = self.model.demands
        max_ind = rng.randint(1, len(heavy_car)-2)
        light_car.insert(len(light_car)-2, heavy_car[max_ind])
        heavy_car.pop(max_ind)

    def opt_2(self, path, noise_std = 0):
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
                    - (dist_mat[path[v1]][path[v2]] + dist_mat[path[v1+1]][path[v2+1]]) \
                    + rng.normalvariate(0, noise_std)
            if gain > 0:
                new_path = []
                new_path.extend(path[:(v1+1)])
                new_path.extend(reversed(path[(v1+1):(v2+1)]))
                new_path.extend(path[(v2+1):])
                return new_path
        return path
        
    def move_city(self, sol, car_ind = 0, noise_std = 0):
        dist_gain_ifremoved = []
        main_path = sol[car_ind]
        for ind, city in enumerate(main_path[1:-1], start=1):
            dist_gain_ifremoved.append((ind, self.model.dist_mat[main_path[ind-1]][city] \
                                        + self.model.dist_mat[city][main_path[ind+1]]\
                                        - self.model.dist_mat[main_path[ind-1]][main_path[ind+1]]))
        # nlogn but whatever
        dist_gain_ifremoved.sort(key=lambda x: x[1], reverse=True)
        city_to_move, dist_gain = rng.choice(dist_gain_ifremoved[:8])
        city_demand = self.model.demands[main_path[city_to_move]]
        other_cars = [i for i in range(len(sol))]
        other_cars.remove(car_ind)
        for i in range(len(other_cars)-1, -1, -1):
            if self.used_car_capacity(sol[other_cars[i]]) + city_demand > self.model.capacity:
                other_cars.pop(i)
        if not other_cars:
            return
        move_city_to = rng.choice(other_cars)
        new_car = sol[move_city_to]
        losses = []
        for (ind, city) in enumerate(new_car[1:-1], start=1):
            losses.append((ind, 
                    self.model.dist_mat[new_car[ind-1]][main_path[city_to_move]]\
                  + self.model.dist_mat[main_path[city_to_move]][new_car[ind]]\
                  - self.model.dist_mat[new_car[ind-1]][new_car[ind]]\
                  + rng.normalvariate(0, noise_std)
                )
            )
        best_move, distance_loss = min(losses, key=lambda x: x[1])
        if dist_gain - distance_loss >= 0:
            new_car.insert(best_move, main_path[city_to_move])
            main_path.pop(city_to_move)
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

    def optimize(self, sol, noise_std = 0):
        for _ in range(parameters.SWAP_FREQ):
            self.opt_2(rng.choice(sol), noise_std)
        self.move_city(sol, rng.randint(0, len(sol)-1), noise_std)

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
        


        