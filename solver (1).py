from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve_gvrp(data):
    starts = data["starts"]
    ends = data["ends"]
    num_depots = data["num_depots"]
    time_windows = data["time_windows"]

    # 1. Inisialisasi Manager untuk MULTI-DEPOT
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]),
        data["num_vehicles"],
        starts,
        ends
    )

    routing = pywrapcp.RoutingModel(manager)

    # =====================================================================
    # MULTI-DEPOT GREEN HEURISTIC (DEMAND-GRAVITY)
    # =====================================================================
    cost_matrix = []
    max_cap = max(data["vehicle_capacities"]) if data["vehicle_capacities"] else 1
    
    for i in range(len(data["distance_matrix"])):
        row = []
        for j in range(len(data["distance_matrix"])):
            dist = data["distance_matrix"][i][j]
            
            # Jika berangkat dari SALAH SATU DEPOT menuju LOKASI PENGIRIMAN
            if i < num_depots and j >= num_depots:
                demand_ratio = data["demands"][j] / max_cap
                gravity_discount = 1.0 - (0.45 * demand_ratio)
                row.append(int(dist * gravity_discount))
            else:
                row.append(dist)
                
        cost_matrix.append(row)

    def green_cost_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return cost_matrix[from_node][to_node]

    cost_callback_index = routing.RegisterTransitCallback(green_cost_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(cost_callback_index)
    # =====================================================================

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return int(data["demands"][from_node])

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,
        data["vehicle_capacities"],
        True,
        "Capacity"
    )

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        travel_time = int(data["time_matrix"][from_node][to_node])
        service_time = int(data["service_times"][from_node])
        return travel_time + service_time

    time_callback_index = routing.RegisterTransitCallback(time_callback)
    
    # Maksimal horizon waktu adalah hari (1440 menit)
    routing.AddDimension(
        time_callback_index,
        120,                
        1440,   
        False,
        "Time"
    )
    time_dimension = routing.GetDimensionOrDie("Time")

    # Batasan Jendela Waktu (Absolut) untuk semua lokasi pengiriman
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Batasan Jendela Waktu (Absolut) untuk kendaraan
    for vehicle_id in range(data["num_vehicles"]):
        start_index = routing.Start(vehicle_id)
        end_index = routing.End(vehicle_id)
        
        depot_idx = starts[vehicle_id]
        tw = time_windows[depot_idx]

        time_dimension.CumulVar(start_index).SetRange(tw[0], tw[1])
        time_dimension.CumulVar(end_index).SetRange(tw[0], tw[1])

        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(start_index))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(end_index))

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 10

    solution = routing.SolveWithParameters(search_parameters)

    if solution is None:
        return None

    route_results = []
    stop_results = []
    total_distance = 0
    latest_actual_arrival = 0
    active_vehicles = 0
    display_vehicle_id = 0

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)

        route_distance = 0
        route_load = 0
        
        # Lakukan pre-looping untuk mendapatkan total beban awal di depot
        temp_index = index
        while not routing.IsEnd(temp_index):
            node_idx = manager.IndexToNode(temp_index)
            route_load += int(data["demands"][node_idx])
            temp_index = solution.Value(routing.NextVar(temp_index))

        current_vehicle_load = route_load  
        max_capacity = data["vehicle_capacities"][vehicle_id]
        route_co2_emission = 0.0 

        route_nodes = []
        route_schedule = []
        route_node_indices = []
        temporary_stop_results = []

        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            arrival_time = solution.Min(time_dimension.CumulVar(index))
            
            demand_at_node = int(data["demands"][node_index])
            current_vehicle_load -= demand_at_node 

            route_nodes.append(data["address_list"][node_index])
            route_node_indices.append(node_index)

            absolute_deadline = time_windows[node_index][1]

            route_schedule.append({
                "Location": data["address_list"][node_index],
                "Time": f"{arrival_time // 60:02d}:{arrival_time % 60:02d}",
                "Arrival_Minutes": arrival_time,
                "Deadline_Minutes": absolute_deadline,
                "Demand": demand_at_node,
                "Current Load": current_vehicle_load, 
                "Latitude": data["raw_coords"][node_index][0],
                "Longitude": data["raw_coords"][node_index][1],
                "Stop Type": "Depot" if node_index < num_depots else "Delivery"
            })

            if node_index >= num_depots:
                latest_actual_arrival = max(latest_actual_arrival, arrival_time)
                lateness = max(0, arrival_time - absolute_deadline)

                temporary_stop_results.append({
                    "Location": data["address_list"][node_index],
                    "Arrival Time": f"{arrival_time // 60:02d}:{arrival_time % 60:02d}",
                    "Deadline": f"{absolute_deadline // 60:02d}:{absolute_deadline % 60:02d}",
                    "Demand": demand_at_node,
                    "On-Time Status": "On time" if lateness == 0 else "Late",
                    "Lateness (mins)": lateness
                })

            previous_index = index
            index = solution.Value(routing.NextVar(index))

            prev_node = manager.IndexToNode(previous_index)
            curr_node = manager.IndexToNode(index)
            
            segment_distance_m = data["distance_matrix"][prev_node][curr_node]
            segment_distance_km = segment_distance_m / 1000.0
            route_distance += segment_distance_m
            
            load_ratio = current_vehicle_load / max_capacity if max_capacity > 0 else 0
            emission_rate = data["emission_empty"] + (load_ratio * (data["emission_full"] - data["emission_empty"]))
            segment_emission = segment_distance_km * emission_rate
            
            route_co2_emission += segment_emission

        end_node = manager.IndexToNode(index)
        end_time = solution.Min(time_dimension.CumulVar(index))

        route_nodes.append(data["address_list"][end_node])
        route_node_indices.append(end_node)

        route_schedule.append({
            "Location": data["address_list"][end_node],
            "Time": f"{end_time // 60:02d}:{end_time % 60:02d}",
            "Arrival_Minutes": end_time,
            "Deadline_Minutes": time_windows[end_node][1],
            "Demand": int(data["demands"][end_node]),
            "Current Load": 0,
            "Latitude": data["raw_coords"][end_node][0],
            "Longitude": data["raw_coords"][end_node][1],
            "Stop Type": "Return to Depot"
        })

        if route_load > 0:
            active_vehicles += 1
            display_vehicle_id += 1
            total_distance += route_distance

            for stop in temporary_stop_results:
                stop["Vehicle"] = display_vehicle_id
                stop_results.append(stop)
                
            distance_km = route_distance / 1000
            
            # Kalkulasi Bahan Bakar dalam Satuan Liter & Biaya Riil
            route_fuel_liters = route_co2_emission / data["fuel_co2_per_liter"]
            fuel_cost = route_fuel_liters * data["fuel_cost_per_liter"]
            driver_cost = data["driver_cost_per_vehicle"]
            total_cost = fuel_cost + driver_cost
            
            depot_name = data["address_list"][starts[vehicle_id]]
            
            route_results.append({
                "Vehicle": display_vehicle_id,
                "Depot Assigned": depot_name,
                "Fuel Consumed (L)": round(route_fuel_liters, 2),
                "Fuel Cost": round(fuel_cost, 2),
                "Driver Cost": round(driver_cost, 2),
                "Total Cost": round(total_cost, 2),
                "Route": " -> ".join(route_nodes),
                "Distance (km)": round(distance_km, 2),
                "Total Payload (kg)": route_load,
                "Utilization (%)": round((route_load / max_capacity) * 100, 2),
                "CO2 Emissions (kg)": round(route_co2_emission, 3), 
                "Return Time": f"{end_time // 60:02d}:{end_time % 60:02d}",
                "Schedule": route_schedule,
                "Node Indices": route_node_indices,
                "Coordinates": [data["raw_coords"][i] for i in route_node_indices]
            })

    return {
        "route_results": route_results,
        "stop_results": stop_results,
        "optimized_distance_km": total_distance / 1000,
        "latest_actual_arrival": latest_actual_arrival,
        "active_vehicles": active_vehicles
    }