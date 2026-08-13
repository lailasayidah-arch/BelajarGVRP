import io
import requests
import math
import pandas as pd
import streamlit as st
import folium
from folium.plugins import AntPath
from streamlit_folium import st_folium

# Pastikan file solver kamu bernama solver.py
from solver import solve_gvrp

# -------------------------------
# Session state initialization
# -------------------------------
if "optimization_result" not in st.session_state:
    st.session_state.optimization_result = None
if "optimization_data" not in st.session_state:
    st.session_state.optimization_data = None

if "num_locations" not in st.session_state:
    st.session_state.num_locations = 2 

# --- PAGE CONFIG ---
st.set_page_config(page_title="Distribution Route Optimization System", layout="wide", page_icon="🌱")

# --- HEADER ---
st.title("🌱 Multi-Depot Green Route Optimization (MDGVRP)")

# -------------------------------
# OSRM helpers
# -------------------------------
def get_osrm_matrices(data):
    coord_string = ";".join([f"{lon},{lat}" for lat, lon in data["raw_coords"]])
    url = f"http://router.project-osrm.org/table/v1/driving/{coord_string}?annotations=duration,distance"
    response = requests.get(url, timeout=20)
    response.raise_for_status()
    result = response.json()
    
    if result.get("code") != "Ok":
        raise ValueError(f"OSRM returned an error: {result}")
    
    data["distance_matrix"] = [[int(v) if v is not None else 999999999 for v in row] for row in result["distances"]]
    data["time_matrix"] = [[math.ceil(v/60) if v is not None else 999999 for v in row] for row in result["durations"]]
    return data

@st.cache_data(show_spinner=False)
def get_osrm_route_geometry(start_coord, end_coord):
    start_lat, start_lon = start_coord
    end_lat, end_lon = end_coord
    url = f"http://router.project-osrm.org/route/v1/driving/{start_lon},{start_lat};{end_lon},{end_lat}?overview=full&geometries=geojson&steps=false"
    response = requests.get(url, timeout=20)
    response.raise_for_status()
    result = response.json()
    if result.get("code") != "Ok":
        return [start_coord, end_coord], 0
    geometry = result["routes"][0]["geometry"]["coordinates"]
    return [(lat, lon) for lon, lat in geometry], None

def get_full_osrm_route(route):
    all_points = []
    for i in range(len(route["Coordinates"]) - 1):
        segment_points, _ = get_osrm_route_geometry(route["Coordinates"][i], route["Coordinates"][i+1])
        if i > 0 and len(segment_points) > 0:
            segment_points = segment_points[1:]
        all_points.extend(segment_points)
    return all_points, None

def create_enhanced_route_map(routes, default_lat=0.0, default_lon=0.0):
    route_colors = ["#FF0000", "#0000FF", "#008000", "#FFA500", "#800080", "#FF00FF", "#00FFFF"]
    
    valid_routes = [r for r in routes if r.get("Coordinates") and len(r["Coordinates"]) > 1]
    if not valid_routes:
        fallback_map = folium.Map(location=[default_lat, default_lon], zoom_start=11)
        return fallback_map

    combined_map = folium.Map(location=valid_routes[0]["Coordinates"][0], zoom_start=11)
    for route in valid_routes:
        vehicle_color = route_colors[(route["Vehicle"]-1) % len(route_colors)]
        road_points, _ = get_full_osrm_route(route)
        if road_points:
            AntPath(locations=road_points, color=vehicle_color, weight=6, opacity=0.9, delay=800).add_to(combined_map)
        
        for idx, stop in enumerate(route["Schedule"]):
            tw_start = stop.get('Opening_Minutes', 0)
            tw_end = stop.get('Deadline_Minutes', 1439)
            tw_text = f"{tw_start//60:02d}:{tw_start%60:02d} - {tw_end//60:02d}:{tw_end%60:02d}"
            
            is_depot = stop.get('Stop Type') in ["Depot", "Return to Depot"]

            folium.Marker(
                location=[stop["Latitude"], stop["Longitude"]],
                tooltip=f"Vehicle {route['Vehicle']} - Stop {idx+1}",
                popup=(
                    f"<b>{stop['Location']}</b><br>"
                    f"Stop Type: <b>{stop.get('Stop Type')}</b><br>"
                    f"Operational Window: {tw_text}<br>"
                    f"Arrival Time: {stop.get('Time','N/A')}<br>"
                    f"Payload Dropped: {stop.get('Demand',0)} kg<br>"
                    f"Current Load: {stop.get('Current Load',0)} kg<br>"
                    f"Vehicle Utilization: {route.get('Utilization (%)',0)}%<br>"
                    f"Distance: {route.get('Distance (km)',0)} km"
                ),
                icon=folium.Icon(color="red" if is_depot else "blue", icon="home" if is_depot else "info-sign")
            ).add_to(combined_map)
    return combined_map

def compute_vehicle_metrics(result, data):
    for route in result["route_results"]:
        schedule = route["Schedule"]
        if schedule:
            total_lateness = 0
            for stop in schedule:
                arrival = stop.get("Arrival_Minutes", 0)
                deadline = stop.get("Deadline_Minutes", 1439)
                lateness = max(0, arrival - deadline)
                stop["Lateness_Minutes"] = lateness
                total_lateness += lateness
            route["Lateness (min)"] = total_lateness
        else:
            route["Lateness (min)"] = 0
    return result

def parse_time_to_minutes(time_str):
    try:
        if pd.isna(time_str) or not str(time_str).strip():
            return 0
        
        time_str = str(time_str).strip()
        if "." in time_str and ":" not in time_str:
            time_str = time_str.split(".")[0]
            
        parts = time_str.split(":")
        hours = int(parts[0])
        minutes = int(parts[1]) if len(parts) > 1 else 0
        return (hours * 60) + minutes
    except:
        return 0

# -------------------------------
# Sidebar inputs (Global parameters)
# -------------------------------
st.sidebar.header("🚚 Fleet Parameters")
driver_cost_per_vehicle = st.sidebar.number_input("Driver Cost per Vehicle (Rp)", 0, 500000, 100000)
num_vehicles = st.sidebar.number_input("Total Number of Vehicles", 1, 30, 4)
vehicle_capacity = st.sidebar.number_input("Vehicle Max Capacity (kg)", 1, 50000, 1000, step=100)

st.sidebar.header("🌱 GVRP Fuel & Emission Factors")
vehicle_type = st.sidebar.selectbox(
    "Vehicle Type & Fuel", 
    ["Light Truck (Diesel)", "Pickup/Van (Gasoline)"]
)

fuel_cost_per_liter = st.sidebar.number_input(
    "Fuel Price per Liter (Rp)", 
    0, 50000, 6800, step=100
)

if vehicle_type == "Light Truck (Diesel)":
    fuel_co2_per_liter = 2.68
    fe_empty = 10.0  
    fe_full = 7.0    
else:
    fuel_co2_per_liter = 2.31
    fe_empty = 12.0  
    fe_full = 9.0    

emission_empty = fuel_co2_per_liter / fe_empty if fe_empty > 0 else 0
emission_full = fuel_co2_per_liter / fe_full if fe_full > 0 else 0

st.sidebar.info(
    f"💡 **Automated Emission Baseline:**\n"
    f"- Empty: **{emission_empty:.3f} kg CO2/km**\n"
    f"- Full Payload: **{emission_full:.3f} kg CO2/km**"
)

st.sidebar.header("📦 Product Specifications")
weight_per_unit = st.sidebar.number_input(
    "Average Weight per Unit/Package (kg)", 
    0.1, 1000.0, 5.0, step=0.1
)

# -------------------------------
# Dynamic Sidebar Location Controls
# -------------------------------
st.sidebar.header("📦 Location Configurations")
input_method = st.sidebar.radio("Data Entry Method", ["Manual Entry", "Excel Upload"])

user_locations = []
depot_indices = []

if input_method == "Manual Entry":
    if st.sidebar.button("➕ Add Location"):
        st.session_state.num_locations += 1

    for i in range(st.session_state.num_locations):
        loc_id = i + 1
        with st.sidebar.expander(f"📍 Location {loc_id}", expanded=(i >= 2)):
            loc_type = st.selectbox("Location Type", ["delivery point", "depot"], index=0, key=f"type_{loc_id}")
            
            col_lat, col_lon = st.columns(2)
            lat = col_lat.number_input("Latitude", value=0.0, format="%.6f", key=f"lat_{loc_id}")
            lon = col_lon.number_input("Longitude", value=0.0, format="%.6f", key=f"lon_{loc_id}")
            
            if loc_type == "delivery point":
                demand_input = st.number_input("Demand (Units)", 0, 10000, 0, key=f"d_in_{loc_id}")
            else:
                demand_input = 0
                depot_indices.append(i)
                
            col_start, col_end = st.columns(2)
            open_time_str = col_start.text_input("Open (HH:MM)", "00:00", key=f"o_tm_{loc_id}")
            close_time_str = col_end.text_input("Close (HH:MM)", "23:59", key=f"c_tm_{loc_id}")
            
            start_minutes = parse_time_to_minutes(open_time_str)
            end_minutes = parse_time_to_minutes(close_time_str)
            
            user_locations.append({
                "name": f"Location {loc_id}" if loc_type == "delivery point" else f"Depot {loc_id}",
                "coords": (lat, lon),
                "demand": demand_input,
                "time_window": (start_minutes, end_minutes)
            })

else:
    st.sidebar.markdown("**Step 1: Download the Template**")
    template_data = {
        "Location Name": ["Depot Jakarta", "Depot Bogor", "Delivery A"],
        "Location Type": ["depot", "depot", "delivery point"],
        "Latitude": [-6.200000, -6.600000, -6.335336],
        "Longitude": [106.816666, 106.800000, 106.680340],
        "Demand": [0, 0, 50],
        "Open Time": ["00:00", "00:00", "08:00"],
        "Close Time": ["23:59", "23:59", "20:00"]
    }
    template_df = pd.DataFrame(template_data)
    template_csv = template_df.to_csv(index=False).encode("utf-8")
        
    st.sidebar.download_button("📥 Download Template Table", data=template_csv, file_name="mdgvrp_template.csv", mime="text/csv")
    uploaded_file = st.sidebar.file_uploader("Upload Completed File", type=["xlsx", "xls", "csv"])
    
    if uploaded_file is not None:
        try:
            df = pd.read_csv(uploaded_file) if uploaded_file.name.endswith(".csv") else pd.read_excel(uploaded_file)
            for idx, row in df.iterrows():
                if pd.isna(row["Location Name"]) or pd.isna(row["Location Type"]): continue
                
                l_type = str(row["Location Type"]).strip().lower()
                start_min = parse_time_to_minutes(row["Open Time"])
                end_min = parse_time_to_minutes(row["Close Time"])
                
                if l_type == "depot":
                    depot_indices.append(len(user_locations))
                    dem = 0
                else:
                    dem = int(row["Demand"]) if not pd.isna(row["Demand"]) else 0

                user_locations.append({
                    "name": str(row["Location Name"]),
                    "coords": (float(row["Latitude"]), float(row["Longitude"])),
                    "demand": dem,
                    "time_window": (start_min, end_min)
                })
            
            # Dinamis Sukses Message 
            num_depots = len(depot_indices)
            num_deliveries = len(user_locations) - num_depots
            
            depot_str = f"{num_depots} depot{'s' if num_depots > 1 else ''}"
            loc_str = f"{num_deliveries} location{'s' if num_deliveries > 1 else ''}"
            
            st.sidebar.success(f"✅ {depot_str} and {loc_str} confirmed!")

        except Exception as e:
            st.sidebar.error(f"Error parsing file: {e}")

# -------------------------------
# Run solver
# -------------------------------
if st.button("🚀 Run MDGVRP Optimization"):
    if not user_locations:
        st.error("No location configurations found.")
        st.stop()
    if not depot_indices:
        st.error("Please configure at least one location as 'depot'.")
        st.stop()

    depot_locations = [user_locations[i] for i in depot_indices]
    delivery_locations = [loc for i, loc in enumerate(user_locations) if i not in depot_indices]
    
    sorted_locations = depot_locations + delivery_locations
    num_depots = len(depot_locations)

    starts = [i % num_depots for i in range(num_vehicles)]
    ends = [i % num_depots for i in range(num_vehicles)]

    final_demands = [
        math.ceil(loc["demand"] * weight_per_unit) if idx >= num_depots else 0 
        for idx, loc in enumerate(sorted_locations)
    ]

    data = {
        "address_list": [loc["name"] for loc in sorted_locations],
        "raw_coords": [loc["coords"] for loc in sorted_locations],
        "demands": final_demands,
        "vehicle_capacities": [vehicle_capacity] * num_vehicles,
        "num_vehicles": num_vehicles,
        "num_depots": num_depots,
        "starts": starts, 
        "ends": ends,     
        "time_windows": [loc["time_window"] for loc in sorted_locations], 
        "service_times": [0 if idx < num_depots else 5 for idx in range(len(sorted_locations))], 
        "fuel_cost_per_liter": fuel_cost_per_liter, 
        "fuel_co2_per_liter": fuel_co2_per_liter,
        "driver_cost_per_vehicle": driver_cost_per_vehicle,
        "emission_empty": emission_empty,
        "emission_full": emission_full
    }

    if any(lat == 0.0 or lon == 0.0 for lat, lon in data["raw_coords"]):
        st.error("Ensure all mapped coordinates are valid.")
        st.stop()

    with st.spinner("Solving Multi-Depot Optimization..."):
        try:
            data = get_osrm_matrices(data)
            result = solve_gvrp(data)
        except Exception as e:
            st.error(f"Routing engine error: {e}")
            st.stop()
        
    if result is None:
        st.error("No feasible solution found. Try adding vehicles, increasing capacity, or widening time windows.")
        st.stop()

    for route in result["route_results"]:
        for stop in route["Schedule"]:
            loc_name = stop["Location"]
            idx = data["address_list"].index(loc_name)
            stop["Opening_Minutes"] = data["time_windows"][idx][0]

    st.session_state.optimization_result = compute_vehicle_metrics(result, data)
    st.session_state.optimization_data = data

# -------------------------------
# Display results
# -------------------------------
if st.session_state.optimization_result:
    result = st.session_state.optimization_result
    data = st.session_state.optimization_data
    routes = result["route_results"]
    num_depots = data["num_depots"]

    total_unoptimized_meters = 0
    matrix = data["distance_matrix"]
    for i in range(num_depots, len(data["address_list"])):
        min_dist = min(matrix[d][i] + matrix[i][d] for d in range(num_depots))
        total_unoptimized_meters += min_dist
    
    baseline_distance = round(total_unoptimized_meters / 1000, 2)
    optimized_distance = sum(r.get("Distance (km)", 0) for r in routes)
    
    avg_emission = (data["emission_empty"] + data["emission_full"]) / 2
    baseline_emissions = baseline_distance * avg_emission
    
    optimized_emissions = sum(r.get("CO2 Emissions (kg)", 0) for r in routes)
    emission_reduction = baseline_emissions - optimized_emissions
    
    col1, col2, col3, col4, col5, col6 = st.columns(6)
    total_operational_cost = sum(r.get("Total Cost", 0) for r in routes)
    total_packages = sum(r.get('Total Payload (kg)', 0) for r in routes)
    total_capacity = sum(data['vehicle_capacities'])
    
    col1.metric("Baseline Dist.", f"{baseline_distance:.2f} km")
    col2.metric("Optimized Dist.", f"{optimized_distance:.2f} km")
    col3.metric("Fleet Utilization", f"{(total_packages / total_capacity * 100) if total_capacity > 0 else 0:.1f}%")
    col4.metric("Operational Cost", f"Rp {total_operational_cost:,.0f}")
    col5.metric("Baseline CO2", f"{baseline_emissions:.2f} kg")
    col6.metric("Optimized CO2", f"{optimized_emissions:.2f} kg", f"-{emission_reduction:.2f} kg", delta_color="inverse")
    
    st.subheader("🗺️ Combined Route Map")
    fallback_coord = data["raw_coords"][0]
    st_folium(create_enhanced_route_map(routes, fallback_coord[0], fallback_coord[1]), width=1000, height=500)

    st.subheader("🚛 Vehicle Summary Table")
    
    vehicle_summary_df = pd.DataFrame(routes)[[
        "Vehicle",
        "Depot Assigned",
        "Distance (km)",
        "Total Payload (kg)",
        "Utilization (%)",
        "CO2 Emissions (kg)",
        "Fuel Consumed (L)", 
        "Total Cost"
    ]]
    st.dataframe(vehicle_summary_df, use_container_width=True)

    for route in routes:
        st.markdown(f"### Vehicle {route['Vehicle']} (Based at {route['Depot Assigned']})")
        if route.get("Total Payload (kg)", 0) == 0 or not route.get("Schedule"):
            st.info("Vehicle not needed for this configuration.")
            continue

        with st.expander(f"Vehicle {route['Vehicle']} Map", expanded=False):
            st_folium(create_enhanced_route_map([route], fallback_coord[0], fallback_coord[1]), width=1000, height=450)

        schedule_records = []
        for s in route["Schedule"]:
            open_min = s.get("Opening_Minutes", 0)
            close_min = s.get("Deadline_Minutes", 1439)
            
            schedule_records.append({
                "Location": s["Location"],
                "Stop Type": s.get("Stop Type"),
                "Operational Window": f"{open_min//60:02d}:{open_min%60:02d} - {close_min//60:02d}:{close_min%60:02d}",
                "Arrival Time": s["Time"],
                "Payload Dropped (kg)": s["Demand"],
                "Current Load (kg)": s.get("Current Load", 0),
                "Latitude": s["Latitude"],
                "Longitude": s["Longitude"]
            })
            
        stop_df = pd.DataFrame(schedule_records)[["Location", "Stop Type", "Operational Window", "Arrival Time", "Payload Dropped (kg)", "Current Load (kg)", "Latitude", "Longitude"]]
        
        st.markdown(f"#### Stop-Level Delivery Table")
        st.dataframe(stop_df, use_container_width=True)

# -------------------------------
# WATERMARK FOOTER
# -------------------------------
st.markdown("---")
st.markdown(
    """
    <div style='text-align: center; color: #888888; font-size: 0.85rem; line-height: 1.6; padding: 10px 0;'>
        Property of Elementary Industrial Laboratory of industrial engineering<br>
        <span style='font-size: 0.8rem; color: #aaaaaa;'>
            Made by: Daniel Delbert Ardielry, Zufar Fathan Hasdiono, Maulida Boru Butarbutar, Natanael Bayu Anggara
        </span>
    </div>
    """, 
    unsafe_allow_html=True
)