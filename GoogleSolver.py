from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from VrpModel import VrpModel
class GoogleSolver:
    def create_data_model(vrp: VrpModel):
        """Stores the data for the problem."""
        data = {}
        data["distance_matrix"] = vrp.dist_mat
        data["num_vehicles"] = vrp.vehicles
        data["demands"] = vrp.demands
        data["vehicle_capacities"] = [vrp.capacity] * vrp.vehicles
        data["depot"] = 0
        return data
    def print_solution(data, manager, routing, solution):
        """Prints solution on console."""
        print(f"Objective: {solution.ObjectiveValue()}")
        max_route_distance = 0
        total_route_distance = 0
        for vehicle_id in range(data["num_vehicles"]):
            index = routing.Start(vehicle_id)
            plan_output = f"Route for vehicle {vehicle_id}:\n"
            route_distance = 0
            while not routing.IsEnd(index):
                plan_output += f" {manager.IndexToNode(index)} -> "
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id
                )
            plan_output += f"{manager.IndexToNode(index)}\n"
            plan_output += f"Distance of the route: {route_distance}m\n"
            print(plan_output)
            total_route_distance += route_distance
            max_route_distance = max(route_distance, max_route_distance)
        #print(f"Maximum of the route distances: {max_route_distance}m")
        print(f'Total route distance: {total_route_distance}')

    def solve(vrp_model: VrpModel, is_print = False):

        # Instantiate the data problem.
        data = GoogleSolver.create_data_model(vrp_model)

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
        )

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        # Create and register a transit callback.
        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data["distance_matrix"][from_node][to_node]
        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        
        # Add Capacity constraint.
        def demand_callback(from_index):
            """Returns the demand of the node."""
            # Convert from routing variable Index to demands NodeIndex.
            from_node = manager.IndexToNode(from_index)
            return data["demands"][from_node]

        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            vrp_model.capacity,  # null capacity slack
            data["vehicle_capacities"],  # vehicle maximum capacities
            True,  # start cumul to zero
            "Capacity",
        )

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        search_parameters.time_limit.FromSeconds(1)

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)
        # Print solution on console.
        if solution:
            if is_print:
                GoogleSolver.print_solution(data, manager, routing, solution)
            return solution.ObjectiveValue()
        else:
            print("No solution found !")



            