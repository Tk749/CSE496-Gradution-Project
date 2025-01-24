import tkinter as tk
from tkinter import ttk
from tkinter import messagebox

import pyomo.environ as pyo
import networkx as nx
from itertools import islice

from tkintermapview import TkinterMapView

import osmnx as ox
from shapely.geometry import Polygon

def k_shortest_paths(graph, source, target, k):
    return list(islice(nx.shortest_simple_paths(graph, source, target), k))

def create_graph(edges, capacity):
    directed_graph = nx.DiGraph()
    for (i, j) in edges:
        directed_graph.add_edge(i, j, capacity=capacity[(i, j)])
    return directed_graph

def create_evacuation_model(edges, passengers, capacity, base_travel_time, K=10):
    model = pyo.ConcreteModel()
    directed_graph = create_graph(edges, capacity)

    all_nodes = set(u for (u, v) in edges) | set(v for (u, v) in edges)
    start_nodes = sorted([n for n in all_nodes if str(n).startswith("Start")])
    end_nodes = sorted([n for n in all_nodes if str(n).startswith("End")])

    model.edges = pyo.Set(initialize=edges)
    model.start_nodes = pyo.Set(initialize=start_nodes)
    model.end_nodes = pyo.Set(initialize=end_nodes)

    paths_dict = {}
    for s in start_nodes:
        for t in end_nodes:
            try:
                paths_dict[(s, t)] = k_shortest_paths(directed_graph, s, t, K)
            except:
                pass

    model.paths = pyo.Set(
        initialize=[(s, t, k_idx)
                    for (s, t) in paths_dict
                    for k_idx in range(len(paths_dict[(s, t)]))]
    )

    model.edge_flow = pyo.Var(model.edges, domain=pyo.NonNegativeIntegers)
    model.path_flow = pyo.Var(model.paths, domain=pyo.NonNegativeIntegers)
    model.travel_time = pyo.Var(model.edges, domain=pyo.NonNegativeReals)

    def objective_rule(m):
        return sum(m.edge_flow[e] * m.travel_time[e] for e in m.edges)
    model.objective = pyo.Objective(rule=objective_rule, sense=pyo.minimize)

    def demand_rule(m, s):
        return sum(
            m.path_flow[s, t, k_idx]
            for t in end_nodes
            for k_idx in range(len(paths_dict.get((s, t), [])))
            if (s, t, k_idx) in m.paths
        ) == passengers[s]
    model.demand = pyo.Constraint(model.start_nodes, rule=demand_rule)

    def capacity_rule(m, i, j):
        return m.edge_flow[i, j] <= capacity[(i, j)]
    model.capacity = pyo.Constraint(model.edges, rule=capacity_rule)

    def flow_conservation_rule(m, i, j):
        ef = m.edge_flow[i, j]
        pf_sum = sum(
            m.path_flow[s, t, k_idx]
            for (s, t, k_idx) in m.paths
            if (i, j) in zip(paths_dict[(s, t)][k_idx][:-1], paths_dict[(s, t)][k_idx][1:])
        )
        return ef == pf_sum
    model.flow_conservation = pyo.Constraint(model.edges, rule=flow_conservation_rule)

    def travel_time_rule(m, i, j):
        return m.travel_time[i, j] == base_travel_time[(i, j)] * (
            1 + (m.edge_flow[i, j]/capacity[(i, j)])
        )
    model.travel_time_constraint = pyo.Constraint(model.edges, rule=travel_time_rule)

    return model, paths_dict, directed_graph

def solve_model(model, base_travel_time, capacity):
    solver = pyo.SolverFactory('ipopt')
    solver.options['max_iter'] = 10000
    solver.options['tol'] = 1e-6
    results = solver.solve(model, tee=False)

    for e in model.edges:
        val = pyo.value(model.edge_flow[e])
        model.edge_flow[e].set_value(round(val))

    for p in model.paths:
        val = pyo.value(model.path_flow[p])
        model.path_flow[p].set_value(round(val))

    for e in model.edges:
        flow_val = pyo.value(model.edge_flow[e])
        base_t = base_travel_time[e]
        cap = capacity[e]
        new_t = base_t * (1 + flow_val/cap)
        model.travel_time[e].set_value(new_t)

    return results

class OSMnxEvacuationApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("OSMnx + Pyomo + All Roads - Coordinate Entry, Deletion, Capacity")
        self.geometry("1300x800")

        style = ttk.Style(self)
        style.theme_use('clam')

        self.selected_zone_polygon = []
        self.zone_polygon_obj = None

        self.start_points_str = []
        self.end_points_str = []

        self.edges = []
        self.capacity = {}
        self.base_travel_time = {}
        self.model = None
        self.graph_nx = None
        self.passengers = {}

        self.drawn_polylines = []
        self.all_roads_polylines = []
        self.markers = []

        self._build_ui()

    def _build_ui(self):
        map_frame = ttk.Frame(self)
        map_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.map_widget = TkinterMapView(map_frame, width=900, height=800, corner_radius=0)
        self.map_widget.pack(fill=tk.BOTH, expand=True)
        self.map_widget.set_position(39.0, 35.0)
        self.map_widget.set_zoom(6)

        side_notebook = ttk.Notebook(self)
        side_notebook.pack(side=tk.RIGHT, fill=tk.BOTH)

        self.side_tab1 = ttk.Frame(side_notebook, padding=10)
        self.side_tab2 = ttk.Frame(side_notebook, padding=10)

        side_notebook.add(self.side_tab1, text="Control Panel")
        side_notebook.add(self.side_tab2, text="Model Results")

        # --- Tab 1 ---
        zone_label = ttk.Label(self.side_tab1, text="Zone")
        zone_label.pack(pady=5)

        zone_corner1_entry = ttk.Entry(self.side_tab1, width=25)
        zone_corner1_entry.insert(0, "40.8470680 29.2964156")
        zone_corner1_entry.pack(pady=2)

        zone_corner2_entry = ttk.Entry(self.side_tab1, width=25)
        zone_corner2_entry.insert(0, "40.8550000 29.3050000")
        zone_corner2_entry.pack(pady=2)

        def set_zone():
            try:
                lat1, lon1 = zone_corner1_entry.get().split()
                lat1 = float(lat1)
                lon1 = float(lon1)
                lat2, lon2 = zone_corner2_entry.get().split()
                lat2 = float(lat2)
                lon2 = float(lon2)
            except:
                messagebox.showerror("Error", "Please enter zone corners in 'lat lon' format.")
                return
            min_lat, max_lat = sorted([lat1, lat2])
            min_lon, max_lon = sorted([lon1, lon2])
            zone_coords = [
                (min_lat, min_lon),
                (min_lat, max_lon),
                (max_lat, max_lon),
                (max_lat, min_lon)
            ]
            self.selected_zone_polygon = zone_coords
            if self.zone_polygon_obj:
                self.zone_polygon_obj.delete()
            self.zone_polygon_obj = self.map_widget.set_polygon(
                zone_coords, outline_color="red", fill_color="red"
            )
            self.info_label.config(text=f"Zone rectangle: {zone_coords}")

        ttk.Button(self.side_tab1, text="Select Zone", command=set_zone).pack(pady=5)

        ttk.Label(self.side_tab1, text="Add Start Point").pack(pady=5)
        start_entry = ttk.Entry(self.side_tab1, width=25)
        start_entry.insert(0, "40.8470680 29.2964156")
        start_entry.pack(pady=2)

        def add_start():
            val = start_entry.get().strip()
            if not val:
                return
            try:
                lat_s, lon_s = val.split()
                float(lat_s), float(lon_s)
            except:
                messagebox.showerror("Error", "Start point format must be 'lat lon'.")
                return
            self.start_points_str.append(val)
            self._refresh_start_list()
            la, lo = float(lat_s), float(lon_s)
            mk = self.map_widget.set_marker(la, lo, text="Start", marker_color_outside="green")
            self.markers.append(mk)

        ttk.Button(self.side_tab1, text="Add Start", command=add_start).pack(pady=5)

        self.start_listbox = tk.Listbox(self.side_tab1, height=5)
        self.start_listbox.pack(pady=2)

        def delete_start():
            sel = self.start_listbox.curselection()
            if not sel:
                return
            idx = sel[0]
            del self.start_points_str[idx]
            self._rebuild_all_markers()
            self._refresh_start_list()

        ttk.Button(self.side_tab1, text="Delete Selected Start", command=delete_start).pack(pady=5)

        ttk.Label(self.side_tab1, text="Add End Point").pack(pady=5)
        end_entry = ttk.Entry(self.side_tab1, width=25)
        end_entry.insert(0, "40.8510000 29.2990000")
        end_entry.pack(pady=2)

        def add_end():
            val = end_entry.get().strip()
            if not val:
                return
            try:
                lat_e, lon_e = val.split()
                float(lat_e), float(lon_e)
            except:
                messagebox.showerror("Error", "End point format must be 'lat lon'.")
                return
            self.end_points_str.append(val)
            self._refresh_end_list()
            la, lo = float(lat_e), float(lon_e)
            mk = self.map_widget.set_marker(la, lo, text="End", marker_color_outside="blue")
            self.markers.append(mk)

        ttk.Button(self.side_tab1, text="Add End", command=add_end).pack(pady=5)

        self.end_listbox = tk.Listbox(self.side_tab1, height=5)
        self.end_listbox.pack(pady=2)

        def delete_end():
            sel = self.end_listbox.curselection()
            if not sel:
                return
            idx = sel[0]
            del self.end_points_str[idx]
            self._rebuild_all_markers()
            self._refresh_end_list()

        ttk.Button(self.side_tab1, text="Delete Selected End", command=delete_end).pack(pady=5)

        ttk.Button(self.side_tab1, text="Create OSMnx Graph", command=self.build_osmnx_graph).pack(pady=5)
        self.info_label = ttk.Label(self.side_tab1, text="...")
        self.info_label.pack(pady=10)

        # --- Tab 2 ---
        self.result_label = ttk.Label(self.side_tab2, text="Model results will appear here.", wraplength=250)
        self.result_label.pack(pady=10)

        ttk.Button(self.side_tab2, text="Draw All Roads", command=self.draw_all_zone_roads).pack(pady=5)
        ttk.Button(self.side_tab2, text="Hide All Roads", command=self.hide_all_zone_roads).pack(pady=5)
        ttk.Button(self.side_tab2, text="Show Start Edge Capacities", command=self.show_start_edge_capacities).pack(pady=5)
        ttk.Button(self.side_tab2, text="Create/Solve Pyomo Model", command=self.create_and_solve_model).pack(pady=5)
        ttk.Button(self.side_tab2, text="Draw Solution on Map", command=self.draw_solution_on_map).pack(pady=5)

    def _refresh_start_list(self):
        self.start_listbox.delete(0, tk.END)
        for s in self.start_points_str:
            self.start_listbox.insert(tk.END, s)

    def _refresh_end_list(self):
        self.end_listbox.delete(0, tk.END)
        for s in self.end_points_str:
            self.end_listbox.insert(tk.END, s)

    def _rebuild_all_markers(self):
        for mk in self.markers:
            mk.delete()
        self.markers.clear()

        for s_str in self.start_points_str:
            lat_s, lon_s = s_str.split()
            la, lo = float(lat_s), float(lon_s)
            mk = self.map_widget.set_marker(la, lo, text="Start", marker_color_outside="green")
            self.markers.append(mk)

        for e_str in self.end_points_str:
            lat_e, lon_e = e_str.split()
            la, lo = float(lat_e), float(lon_e)
            mk = self.map_widget.set_marker(la, lo, text="End", marker_color_outside="blue")
            self.markers.append(mk)

    def build_osmnx_graph(self):
        if not self.selected_zone_polygon:
            messagebox.showerror("Error", "Please define the zone first.")
            return
        if not self.start_points_str or not self.end_points_str:
            messagebox.showerror("Error", "At least one Start and one End point are required.")
            return

        poly_lonlat = [(lon, lat) for (lat, lon) in self.selected_zone_polygon]
        polygon = Polygon(poly_lonlat)
        G = ox.graph_from_polygon(polygon, network_type='all')
        self.graph_nx = G

        start_coords = []
        for s_str in self.start_points_str:
            lat_s, lon_s = s_str.split()
            start_coords.append((float(lat_s), float(lon_s)))

        end_coords = []
        for e_str in self.end_points_str:
            lat_e, lon_e = e_str.split()
            end_coords.append((float(lat_e), float(lon_e)))

        all_points = start_coords + end_coords
        lats = [p[0] for p in all_points]
        lons = [p[1] for p in all_points]
        nearest_nodes = ox.distance.nearest_nodes(G, X=lons, Y=lats)

        num_start = len(start_coords)
        start_nodes_osm = nearest_nodes[:num_start]
        end_nodes_osm = nearest_nodes[num_start:]

        osm_to_label = {}
        for i, osm_id in enumerate(start_nodes_osm, start=1):
            osm_to_label[osm_id] = f"Start{i}"
        for i, osm_id in enumerate(end_nodes_osm, start=1):
            osm_to_label[osm_id] = f"End{i}"

        edges_temp = []
        capacity_temp = {}
        base_time_temp = {}

        for u, v, key, data in G.edges(keys=True, data=True):
            dist_m = data.get('length', 10.0)
            speed_kph = data.get('speed_kph', 30)
            travel_time_min = (dist_m / 1000.0) / speed_kph * 60.0

            cap = 5000 - int(dist_m / 5)
            if cap < 300:
                cap = 300

            lu = osm_to_label[u] if u in osm_to_label else str(u)
            lv = osm_to_label[v] if v in osm_to_label else str(v)

            edges_temp.append((lu, lv))
            capacity_temp[(lu, lv)] = cap
            base_time_temp[(lu, lv)] = travel_time_min

        self.edges = edges_temp
        self.capacity = capacity_temp
        self.base_travel_time = base_time_temp

        self.passengers = {}
        for i in range(num_start):
            lbl = f"Start{i+1}"
            self.passengers[lbl] = 3000

        self.info_label.config(
            text=f"Graph created.\nNodes: {len(G.nodes)} | Edges: {len(edges_temp)}\nStart: {num_start}, End: {len(end_coords)}"
        )

    def draw_all_zone_roads(self):
        if not self.graph_nx:
            messagebox.showerror("Error", "Please create the OSMnx graph first.")
            return
        self.hide_all_zone_roads()

        for u, v, key in self.graph_nx.edges(keys=True):
            lat_u = self.graph_nx.nodes[u]['y']
            lon_u = self.graph_nx.nodes[u]['x']
            lat_v = self.graph_nx.nodes[v]['y']
            lon_v = self.graph_nx.nodes[v]['x']
            pl = self.map_widget.set_path([(lat_u, lon_u), (lat_v, lon_v)], color="gray", width=1)
            self.all_roads_polylines.append(pl)

        self.info_label.config(text="All roads (gray) drawn.")

    def hide_all_zone_roads(self):
        for pl in self.all_roads_polylines:
            pl.delete()
        self.all_roads_polylines = []
        self.info_label.config(text="All roads hidden.")

    def show_start_edge_capacities(self):
        if not self.capacity:
            messagebox.showinfo("Info", "Graph not created or no capacities.")
            return

        lines = []
        for (i, j), cap_val in self.capacity.items():
            if i.startswith("Start"):
                lines.append(f"{i} -> {j}: capacity={cap_val}")

        text_out = "\n".join(lines) if lines else "No edges found starting with 'Start'."
        messagebox.showinfo("Start Edge Capacities", text_out)

    def create_and_solve_model(self):
        if not self.edges:
            messagebox.showerror("Error", "Create the OSMnx graph first.")
            return

        self.model, _, _ = create_evacuation_model(
            edges=self.edges,
            passengers=self.passengers,
            capacity=self.capacity,
            base_travel_time=self.base_travel_time,
            K=10
        )
        results = solve_model(self.model, self.base_travel_time, self.capacity)
        obj_val = pyo.value(self.model.objective)

        self.result_label.config(
            text=f"Model solved.\nStatus: {results.solver.status}\nObjective: {obj_val}"
        )
        self.info_label.config(
            text=f"Model solved. Status: {results.solver.status}"
        )

    def draw_solution_on_map(self):
        if self.model is None:
            messagebox.showerror("Error", "Please create and solve the model first.")
            return
    
        # Önceden çizilmiş polylineleri sil.
        for pl in self.drawn_polylines:
            pl.delete()
        self.drawn_polylines = []
    
        # Etiketlerin (Start/End) yerine gerçek OSM ID'lerine dönmek için yardımcı fonksiyon:
        def unlabel_node(lbl):
            if lbl.startswith("Start") or lbl.startswith("End"):
                return None
            else:
                try:
                    return int(lbl)
                except:
                    return None
    
        # Modelde akış değerleri > 0 olan kenarları çiz.
        for (i, j) in self.model.edges:
            flow_val = pyo.value(self.model.edge_flow[i, j])
            if flow_val > 0:
                osm_i = unlabel_node(i)
                osm_j = unlabel_node(j)
                if (osm_i is None) or (osm_j is None):
                    continue
                if (osm_i in self.graph_nx.nodes) and (osm_j in self.graph_nx.nodes):
                    lat_i = self.graph_nx.nodes[osm_i]['y']
                    lon_i = self.graph_nx.nodes[osm_i]['x']
                    lat_j = self.graph_nx.nodes[osm_j]['y']
                    lon_j = self.graph_nx.nodes[osm_j]['x']
    
                    # Kalınlık, akışa bağlı olarak ayarlanabilir.
                    width = min(8, 1 + flow_val / 500)
    
                    # Renk seçimi: az-orta-fazla akış
                    if flow_val < 1000:
                        color = "green"
                    elif flow_val < 2000:
                        color = "orange"
                    else:
                        color = "red"
    
                    pl = self.map_widget.set_path([(lat_i, lon_i), (lat_j, lon_j)],
                                                  color=color, width=width)
                    self.drawn_polylines.append(pl)
    
        self.info_label.config(text="Flows are drawn on the map.")


if __name__ == "__main__":
    app = OSMnxEvacuationApp()
    app.mainloop()
