## Vacuum Robot Planner

A Go implementation of a vacuum robot path planner using uniform-cost search to find optimal cleaning routes in grid environments.

### Features

- **Uniform-Cost Search (UCS)**: Finds optimal solutions using a priority queue and closed list
- **State Space Search**: Tracks robot position and remaining dirt locations
- **Obstacle Avoidance**: Handles blocked cells (#) and boundary constraints
- **Performance Metrics**: Reports nodes generated and expanded

### Usage

- go build -o algo algo.go
- ./algo uniform-cost < input2.txt
