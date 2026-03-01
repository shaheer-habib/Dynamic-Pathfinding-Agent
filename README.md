# 🤖 Dynamic Pathfinding Agent

> A real-time interactive pathfinding visualizer built in Python + Pygame.
> Implements **A\* Search** and **Greedy Best-First Search (GBFS)** with dynamic obstacle spawning and automatic re-planning.

---

## 📋 Table of Contents

- [About the Project](#about-the-project)
- [Features](#features)
- [Installation](#installation)
- [How to Run](#how-to-run)
- [How to Use](#how-to-use)
- [Keyboard Shortcuts](#keyboard-shortcuts)
- [Color Legend](#color-legend)
- [Algorithms](#algorithms)
- [Heuristics](#heuristics)
- [Project Structure](#project-structure)
- [Requirements](#requirements)

---

## 📖 About the Project

This project was built for the **Artificial Intelligence** course at the  
**National University of Computer & Emerging Sciences — Chiniot-Faisalabad Campus**.

The goal is to develop a **Dynamic Pathfinding Agent** that can navigate a grid-based environment
where obstacles appear randomly while the agent is moving — requiring real-time path detection
and re-planning.

---

## ✨ Features

| Feature | Description |
|---------|-------------|
| 🔍 A\* Search | Finds the optimal shortest path using `f(n) = g(n) + h(n)` |
| ⚡ Greedy BFS | Fast heuristic-only search using `f(n) = h(n)` |
| 📐 Manhattan Heuristic | Best for 4-directional grid movement |
| 📏 Euclidean Heuristic | Straight-line distance estimate |
| 🎲 Random Maze Generator | Generate mazes with custom obstacle density (5%–75%) |
| ✏️ Interactive Map Editor | Click/drag to place or remove walls manually |
| 🚀 Dynamic Mode | Obstacles spawn in real time; agent re-plans automatically |
| 📊 Live Metrics | Nodes visited, path cost, execution time (ms) |
| 🔄 Animated Visualization | Watch frontier, visited nodes, and final path animate step by step |
| ↔️ Resizable Grid | Grow or shrink the grid at any time |

---

## 🛠️ Installation

### Step 1 — Install Python 3.11

> ⚠️ **Use Python 3.11** — Python 3.14 is too new and Pygame does not support it yet.

Download Python 3.11 from:  
👉 https://www.python.org/downloads/release/python-3119/

Click **"Windows installer (64-bit)"** and install.

> 🔴 **IMPORTANT:** During installation, check the box **"Add Python to PATH"** before clicking Install Now.

---

### Step 2 — Install Pygame

Open a terminal (or VS Code terminal with `Ctrl + ~`) and run:

```bash
py -3.11 -m pip install pygame
```

---

## ▶️ How to Run

```bash
py -3.11 pathfinding_agent.py
```

Or if `py` doesn't work on your system:

```bash
python pathfinding_agent.py
```

---

## 🎮 How to Use

### Basic Pathfinding

1. The app opens with a random maze already generated
2. **S** (Teal) = Start node — top-left corner by default
3. **G** (Orange) = Goal node — bottom-right corner by default
4. Select your **Algorithm** (A\* or Greedy BFS) from the right panel
5. Select your **Heuristic** (Manhattan or Euclidean)
6. Click **▶ Run Search** or press `Enter`
7. Watch the animation — Yellow = frontier, Blue = visited, Green = final path
8. Check the **Metrics box** on the panel for results

---

### Drawing Walls Manually

1. Click **Draw Wall** on the panel (it will highlight green = active)
2. Click any white cell on the grid to place a wall
3. Click a dark cell to remove it
4. You can click and drag to draw multiple walls quickly

---

### Setting Start / Goal

1. Click **Set Start** on the panel
2. Click any empty cell on the grid — the **S** marker moves there
3. Do the same with **Set Goal** for the **G** marker

---

### Generating a Random Maze

1. Use **Density +** / **Density –** to set how many walls you want (e.g. 30%)
2. Click **Generate Maze** — a new random grid is created
3. Click **Run Search** to solve it

---

### Dynamic Mode (Real-Time Re-Planning)

1. Click **Dynamic Mode: OFF** to turn it **ON** (button turns purple)
2. Click **▶ Run Search**
3. After the search finishes, a **purple dot** (the agent) starts moving along the path
4. New walls randomly spawn on the grid while the agent moves
5. If a new wall blocks the remaining path → the agent **immediately re-plans** from its current position
6. Watch the path update in real time!

---

### Resizing the Grid

- **Grid Larger** — adds 2 rows and 2 columns (max 30×38)
- **Grid Smaller** — removes 2 rows and 2 columns (min 6×8)
- The cell size auto-adjusts to always fill the display area

---

## ⌨️ Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `Enter` | Run Search |
| `C` | Clear path (keep walls) |
| `R` | Reset grid (remove all walls) |
| `G` | Generate new random maze |

---

## 🎨 Color Legend

| Color | What It Means |
|-------|---------------|
| ⬜ Light Gray | Empty / passable cell |
| ⬛ Dark Gray | Wall (impassable) |
| 🟦 Teal | Start node (S) |
| 🟧 Orange | Goal node (G) |
| 🟨 Yellow | Frontier — nodes currently in the queue |
| 🟦 Blue | Visited — nodes already expanded |
| 🟩 Green | Final path found |
| 🟣 Purple dot | Agent position (Dynamic Mode only) |

---

## 🔬 Algorithms

### A\* Search
```
f(n) = g(n) + h(n)
```
- `g(n)` = actual cost from Start to node n (number of steps)
- `h(n)` = heuristic estimate from n to Goal
- **Optimal** — always finds the shortest path
- **Complete** — always finds a path if one exists
- Slightly slower than GBFS but much more reliable

### Greedy Best-First Search (GBFS)
```
f(n) = h(n)
```
- Only uses the heuristic — ignores actual path cost
- **NOT optimal** — may find a longer path
- **Faster** than A\* in open grids
- Can get stuck or produce bad results in complex mazes

---

## 📐 Heuristics

### Manhattan Distance
```
h = |x1 - x2| + |y1 - y2|
```
Best for this project because the agent moves in 4 directions only (up/down/left/right).
Never overestimates — **admissible** — guarantees optimal path with A\*.

### Euclidean Distance
```
h = sqrt((x1-x2)² + (y1-y2)²)
```
Straight-line distance. Slightly underestimates in grid movement.
Can produce good results but less accurate for 4-directional movement.

---

## 📁 Project Structure

```
pathfinding-agent/
│
├── pathfinding_agent.py    ← Main file (entire project in one file)
├── README.md               ← This file
```

Everything is in a single Python file — no extra folders or config files needed.

---

## 📦 Requirements

```
Python  >= 3.11
pygame  >= 2.0
```

Install all dependencies:
```bash
py -3.11 -m pip install pygame
```

---

## 🏫 Course Information

| Field | Detail |
|-------|--------|
| Course | Artificial Intelligence |
| Institution | NUCES — Chiniot-Faisalabad Campus |
| Project Type | Informed Search / Dynamic Replanning |

---

> Built with Python 🐍 + Pygame 🎮
