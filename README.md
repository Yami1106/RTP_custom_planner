# RTP_custom_planner

As part of my Master's coursework in Robotics, I implemented a **custom RTP (Random Tree Planner)** motion planner using the [OMPL](https://ompl.kavrakilab.org/) library.  
This project demonstrates the design, implementation, and benchmarking of a simple motion planner in comparison with standard planners like EST and RRT.

---

## 📌 Overview

The **RTP planner** is a basic sampling-based motion planner inspired by RRT, with a randomized parent-selection strategy:
- A random node is chosen from the tree (not always the nearest one).
- The tree is extended toward a random sample in the space.
- Valid states are added until the goal region is reached or the time budget expires.

This makes RTP less greedy than RRT and often more exploratory, at the cost of efficiency.

---

## 🛠️ Features

- Supports **point robot** in 2D (R² space).  
- Supports **square robot** in SE(2) (translation + rotation).  
- Custom **collision checkers** for both point and square robots.  
- Two different **environments** with rectangular obstacles:
  - Corridor-like layered environment (Environment 1).
  - Maze-like environment with U-shaped bays and central block (Environment 2).  
- Writes out **solution paths** and **obstacle files** for visualization.  
- Benchmarking results comparing RTP vs EST vs RRT:
  - Metrics include **computation time**, **path length**, and **number of sampled states**.


---

## 🚀 How to Build & Run

### Requirements
- [OMPL](https://ompl.kavrakilab.org/) (>=1.6)
- C++17 (tested with g++/clang++)
- CMake

### Build
```bash
mkdir build && cd build
cmake ..
make


