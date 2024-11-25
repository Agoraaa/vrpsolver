import random as rng
from VrpModel import VrpModel
from GraphAlgos import GraphAlgos
import networkx as nx
import numpy as np
import math

def all_possible_matchings(arr):
    res = []
    if len(arr) == 2:
        return [[(arr[0], arr[1])]]
    for i in range(1, len(arr)):
        # pick 0 and i
        matching = (arr[0], arr[i])
        arr.pop(i)
        arr.pop(0)
        sub_res = all_possible_matchings(arr)
        for sub_matchings in sub_res:
            sub_matchings.append(matching)
        res.extend(sub_res)
        arr.insert(0, matching[0])
        arr.insert(i, matching[1])
    return res


class VrpSolver:
    def __mst_to_christofides(dist_mat, mst_edges):
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
                sub_dist_mat[i][j] = dist_mat[index_to_original[i], index_to_original[j]]
                sub_dist_mat[j][i] = dist_mat[index_to_original[i], index_to_original[j]]
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
        eulerian_circuit = [u for u, v in nx.eulerian_circuit(merged_graph, source=original_to_index[0])]
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

    def construct(model: VrpModel):
        dist_mat = model.dist_mat
        # select 2*vehicle edges from depot. currently we select it randomly
        edges = []
        nodes = [i for i in range(1, len(model.dist_mat))]
        while len(edges) < 2*model.vehicles:
            node_to_add = rng.choice(nodes)
            edges.append((0, node_to_add))
            nodes.remove(node_to_add)
        INITIAL_EDGES_TO_ADD = 0
        average_distance = dist_mat.sum() / (len(dist_mat)**2 - len(dist_mat))
        initial_mst = edges.copy()
        for (root, first_vertex) in edges:
            for _ in range(INITIAL_EDGES_TO_ADD):
                max_dist_multiplier = 1.60
                min_dist_multiplier = 1.0
                while 1:
                    #print(f'Accept up to: {accept_dist_ratio}')
                    node_to_add = rng.choice(nodes)
                    if(dist_mat[first_vertex][node_to_add] > average_distance * max_dist_multiplier) or (dist_mat[first_vertex][node_to_add] < average_distance*min_dist_multiplier):
                        max_dist_multiplier += 0.001
                        #min_dist_multiplier -= 0.001
                        continue
                    initial_mst.append((first_vertex, node_to_add))
                    nodes.remove(node_to_add)
                    break
        mst = GraphAlgos.minimum_spanning_tree(model.dist_mat, edges)
        connected_components = {}
        # remove node 0 and calculate connected components, this will give 2*vehicle MSTs
        counts = [1 for i in range(len(dist_mat))]
        parents = [i for i in range(len(dist_mat))]
        for (edge_start, edge_end) in mst:
            if (edge_start == 0) or (edge_end == 0):
                continue
            GraphAlgos.dsu_merge_sets(edge_start, edge_end, parents, counts)
        for (edge_start, edge_end) in mst:
            if edge_end == 0:
                continue
            parent = GraphAlgos.dsu_find_parent(edge_end, parents)
            if parent not in connected_components:
                connected_components[parent] = []
            connected_components[parent].append((edge_start, edge_end))
        sub_msts = []
        solutions = []
        car_path_mergings = all_possible_matchings(edges)
        CHECK_MATCHINGS_RATIO = 0.05
        max_ind = min(len(car_path_mergings), math.ceil(len(car_path_mergings) * CHECK_MATCHINGS_RATIO))
        max_ind = max(max_ind, 1)
        rng.shuffle(car_path_mergings)
        for matchings in car_path_mergings[:max_ind]:
            vehicle_paths = []
            for vehicle_path in matchings: # [(0, v1), (0, v2)]
                path1, path2 = vehicle_path
                mst_to_add = []
                mst_to_add.append(path1)
                mst_to_add.append(path2)
                for edge in connected_components[GraphAlgos.dsu_find_parent(path1[1], parents)]:
                    mst_to_add.append(edge)
                for edge in connected_components[GraphAlgos.dsu_find_parent(path2[1], parents)]:
                    mst_to_add.append(edge)
                vehicle_paths.append(VrpSolver.__mst_to_christofides(dist_mat, mst_to_add))
            solutions.append(vehicle_paths)
        minimum_index = 0
        minimum_solution = model.find_solution_value(solutions[0])
        for i in range(1, len(solutions)):
            sol_value = model.find_solution_value(solutions[i])
            if sol_value < minimum_solution:
                minimum_index = i
                minimum_solution = sol_value
        return solutions[minimum_index]
        

        

