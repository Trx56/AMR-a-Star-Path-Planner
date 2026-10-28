# 3D AMR Labyrinth Path Planning Simulation 

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![NumPy](https://img.shields.io/badge/NumPy-Scientific%20Computing-orange.svg)](https://numpy.org/)
[![Matplotlib](https://img.shields.io/badge/Matplotlib-3D%20Visualization-green.svg)](https://matplotlib.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**AMR-A-Star-Path-Planner** is a 3D simulation engine designed for an Autonomous Mobile Robot (AMR) navigating a complex, multi-corridor warehouse labyrinth. The project models spatial mapping, optimal pathfinding, and dynamic trajectory execution from scratch using Python.

---

##  Key Features

* **A* Search Algorithm Integration:** Implements an 8-directional graph traversal algorithm leveraging Euclidean distance heuristics to compute the shortest, collision-free route across a discrete grid.
* **3D Spatial Extrusion:** Transforms flat NumPy grid matrices into solid 3D warehouse walls and obstacles using Matplotlib's `bar3d` toolkit.
* **Dynamic Breadcrumb Tracking:** Renders a real-time, step-by-step orange trail that follows the robot as it successfully maneuvers through restrictive maze hallways.
* **Corner-Collision Constraints:** Features custom path-checking logic that prevents the pathfinder from illegally cutting diagonally through tight internal maze corners.
* **Automated Camera Orbiting:** Dynamically rotates the 3D viewing angle frame-by-frame to showcase spatial depth and obstacles during transit.

---

## Tech Stack & Libraries
* **Language:** Python
* **Grid & Matrix Math:** NumPy
* **Animation & Rendering:** Matplotlib (`mpl_toolkits.mplot3d`, `FuncAnimation`)

---

##  Code Architecture (`amr_planner.py`)
```text
 AMR-A-Star-Path-Planner
 ┣  amr_planner.py      # Core A* search algorithm, labyrinth layout, and 3D animator
 ┗  README.md           # Project documentation          Core Implementation Overview:

1. Priority Queue Search (`astar_planner`): Evaluates neighboring nodes using total cost scoring ($f = g + h$), ensuring optimal path discovery while circumventing static obstacles.
2. Dynamic Frame Rendering (`animate`): Updates robot coordinates across $(X, Y, Z)$ axes iteratively, generating smooth HTML5 video outputs suitable for Google Colab or local execution.

