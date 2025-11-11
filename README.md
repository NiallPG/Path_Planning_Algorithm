Vacuum Robot Planner
A Go implementation of an AI path planner for a vacuum robot in a grid environment. Uses uniform-cost search to find optimal cleaning sequences.
Features

Uniform-Cost Search: Finds minimum-cost action sequences to clean all dirt
Grid-based Navigation: Handles walls and multiple dirt locations
Optimal Planning: Guarantees shortest path solutions
Performance Tracking: Reports nodes generated and expanded during search

To run:
go build -o algo algo.go
./algo uniform-cost < input2.txt
