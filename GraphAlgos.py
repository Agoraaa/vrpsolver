import numpy as np
import networkx as nx


def dsu_find_parent(node, parents):
    if parents[node] == node:
        return node
    return dsu_find_parent(parents[node], parents)

def dsu_merge_sets(node1, node2, parents, counts):
    node1 = dsu_find_parent(node1, parents)
    node2 = dsu_find_parent(node2, parents)
    if node1 == node2:
        return
    if counts[node1] < counts[node2]:
        # swap
        tmp = node1
        node1 = node2
        node2 = tmp
    parents[node2] = node1
    counts[node1] += counts[node2]

def minimum_spanning_tree(dist_mat, initial_edges = None):
    res = []
    # dsu stuff
    counts = [1] * len(dist_mat)
    parents = [i for i in range(len(dist_mat))]
    if initial_edges != None:
        for (edge_start, edge_end) in initial_edges:
            res.append((edge_start, edge_end))
            dsu_merge_sets(edge_start, edge_end, parents, counts)
    edge_list = []
    for i in range(len(dist_mat)):
        for j in range(i+1, len(dist_mat)):
            edge_list.append((i, j, dist_mat[i][j]))
    edge_list.sort(key=(lambda a: a[2]))
    for (edge_start, edge_end, cost) in edge_list:
        if dsu_find_parent(edge_start, parents) == dsu_find_parent(edge_end, parents):
            continue
        # dont add any other edge to the root node
        if initial_edges is not None:
            if edge_start == 0 or edge_end == 0:
                continue
        dsu_merge_sets(edge_start, edge_end, parents, counts)
        res.append((edge_start, edge_end))
        if counts[dsu_find_parent(edge_start, parents)] == len(dist_mat):
            # early finish cuz i think ~3n iterations should be enough to build the tree, as opposed to n^2
            break
    return res
def perfect_matching(dist_mat):
    graph = nx.Graph()
    for i in range(len(dist_mat)):
        for j in range(i+1, len(dist_mat)):
            graph.add_edge(i, j, weight=dist_mat[i][j])
    return nx.min_weight_matching(graph)
    
        