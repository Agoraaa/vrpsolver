from VrpModel import *

def used_car_capacity(car, model):
    res = 0
    for demand_point in car:
        res += model.demands[demand_point]
    return res

def sort_solution(sol, model):
    sol.sort(key=lambda x: used_car_capacity(x, model), reverse=True)

def sort_solution_bubble(sol, model):
    for i in range(len(sol)-1):
        if used_car_capacity(sol[i], model) > used_car_capacity(sol[i+1], model):
            break
        sol[i], sol[i+1] = sol[i+1], sol[i]
    for i in range(len(sol)-1, 0, -1):
        if used_car_capacity(sol[i], model) < used_car_capacity(sol[i-1], model):
            break    
        sol[i], sol[i-1] = sol[i-1], sol[i]

def find_solution_value(model, vehicle_paths):
    res = 0
    for path in vehicle_paths:
        for i in range(len(path)-1):
            res += model.dist_mat[path[i]][path[i+1]] 
    res += model.dist_mat[path[-1]][0]
    return res

def is_feasible(sol, model: VrpModel):
    timesVisited = [0] * len(model.demands)
    timesVisited[0] = 1
    used_capacities = [0] * model.vehicles
    for car_ind in range(len(sol)):
        car = sol[car_ind]
        for node in car[1:-1]:
            used_capacities[car_ind] += model.demands[node]
            timesVisited[node] += 1
    for i in timesVisited[1:]:
        if i != 1:
            print("Wadafak")
            return False
    return max(used_capacities) <= model.capacity
    

