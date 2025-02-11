# Tsunami Traffic Evacuation Optimization

## Overview

This project focuses on optimizing traffic evacuation strategies in tsunami scenarios. The goal is to minimize evacuation time and improve overall efficiency using advanced optimization models. The study incorporates both mixed-integer nonlinear programming (MINLP) and integer linear programming (ILP) approaches to determine the most effective vehicle routing during emergency evacuations.

## Methodology

The study employs a two-stage evacuation model:

1. **First Stage:** Minimize the maximum evacuation time for individuals reaching temporary shelters.
2. **Second Stage:** Optimize the total evacuation time from temporary shelters to safe zones.

### Optimization Models Used:

- **Minimizing Maximum Evacuation Time (MM):** Ensures the longest individual evacuation time is minimized.
- **Link-Based System Optimization (LSO):** Focuses on minimizing total travel time and traffic flow.

### Tools & Libraries:

- **Optimization:** Pyomo, IBM CPLEX Optimization Studio (DOcplex)
- **Graph Processing:** NetworkX
- **GUI Development:** Tkinter
- **Map Processing:** OSMnx
- **Data Handling & Visualization:** pandas, matplotlib

## Experimental Study

The optimization models were tested using real-world data from **Honolulu, Hawaii**, with different tsunami scenarios:

- **Scenario 1:** Local tsunami with limited evacuation time.
- **Scenario 2:** Distant tsunami allowing a full-scale evacuation.

Key performance metrics include:

- Total travel time
- Maximum evacuation time
- Flow distribution across paths
- Capacity utilization of road networks
- Efficiency comparison of optimization techniques

## GUI Implementation

A Tkinter-based graphical user interface (GUI) allows users to:

- Select a geographic area for evacuation.
- Define start and end points.
- Generate road networks using OSMnx.
- Solve the optimization model and visualize results on a map.
- View traffic flow distribution and capacity utilization.

## Results & Findings

- The **LSO model** consistently produced the best results in minimizing total travel time.

- Real-world constraints, such as congestion and road capacities, were incorporated into the models.

## Challenges & Limitations

- Assumes complete compliance with evacuation orders.
- Real-world uncertainties, such as panic and communication delays, are not fully modeled.
- The computational complexity of solving large-scale networks remains a challenge.

---

For more details, check out the full [report](./report.pdf) in the repository!
