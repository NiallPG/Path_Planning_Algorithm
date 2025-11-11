package main // creates standard executable

import (
	"bufio"
	"fmt"
	"os"
	"container/heap"
	"strconv"
	"strings"
) // import formatted io

type position struct {
	x int
	y int
}

type state struct { // represent the "what" for the configuration of the world
	current_position position
	// dirt_positions is an array of position items
	dirt_positions []position
}

type Node struct { // represents the "how" for the search path and cost
	G      int    // cost to reach this state from start
	State  state  // the world looks like
	Parent *Node  // where did we just come from? (path reconstruction)
	Action string // what was performed here (N,E,S,W,V for path reconstruction)
}

type PriorityQueue []*Node

func (pq PriorityQueue) Len() int { return len(pq) }
func (pq PriorityQueue) Less(i, j int) bool {
	return pq[i].G < pq[j].G
}
func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
}
func (pq *PriorityQueue) Push(x interface{}) {
	node := x.(*Node)
	*pq = append(*pq, node)
}
func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	node := old[n-1]
	*pq = old[0 : n-1]
	return node
}

func isValid(x, y int, grid [][]rune, rows, cols int) bool { // bounds helper func
    // Check bounds
    if x < 0 || x >= cols || y < 0 || y >= rows {
        return false
    }
    // Check not a wall
    return grid[y][x] != '#'
}

func UCS(grid [][]rune, robotStart position, dirtPositions []position, rows, cols int) {
	// setup here is {x y} where x is x over from the left and y is y down from the top

	// we need to
	inital_state := state{
		current_position: robotStart,
		dirt_positions:   dirtPositions,
	}

	var_node := Node{
		G:      0,
		State:  inital_state,
		Parent: nil,
		Action: "",
	}

	// we want to modify the original priority queue, not copies, that's why we need &
	pq := &PriorityQueue{}
	heap.Init(pq) // init the heap
	heap.Push(pq, &var_node)

	// keys are strings (state), vals are bools (t/f if visited)
	closedList := make(map[string]bool)

	nodes_generated := 1
	nodes_expanded := 0 // 0 then incremented when popped
	var currentNode *Node
	for pq.Len() > 0 { // while pq is not empty
		// pop removes elem at root of heap
		// getting lowest cost node to expand
		currentNode = heap.Pop(pq).(*Node) // (*Node) is type assertion
		// we need to check 
		// create a unique key for each state
		stateKey := fmt.Sprintf("%d,%d|%v", 
		currentNode.State.current_position.x, 
        currentNode.State.current_position.y, 
        currentNode.State.dirt_positions)

		if closedList[stateKey] == true { // if statekey is in closed list, continue
			continue
		}
		// otherwise we have expanded to a new node
		nodes_expanded++
		closedList[stateKey] = true

		// goal state is if dirt array is empty
		if len(currentNode.State.dirt_positions) == 0 {
			break // then go to path construction
		}
		// north is x + 0, y - 1
		// south is x + 0, y + 1
		// west is x - 1, y + 0
		// east is x + 1, y + 0
		// north
		new_x := currentNode.State.current_position.x
		new_y := currentNode.State.current_position.y - 1
		if isValid(new_x, new_y, grid, rows, cols) {
			nodes_generated += 1
			new_pos := position{x: new_x, y: new_y}
			// need new state, add it to the pq
			new_state := state{current_position: new_pos, dirt_positions: currentNode.State.dirt_positions}
			new_node := Node{
				G: currentNode.G + 1,
				State: new_state,
				Parent: currentNode,
				Action: "N",
			}
			heap.Push(pq, &new_node)
		}
		//south
		new_x = currentNode.State.current_position.x
		new_y = currentNode.State.current_position.y + 1
		if isValid(new_x, new_y, grid, rows, cols) {
			nodes_generated += 1
			new_pos := position{x: new_x, y: new_y}
			// need new state, add it to the pq
			new_state := state{current_position: new_pos, dirt_positions: currentNode.State.dirt_positions}
			new_node := Node{
				G: currentNode.G + 1,
				State: new_state,
				Parent: currentNode,
				Action: "S",
			}
			heap.Push(pq, &new_node)
		}
		//east 
		new_x = currentNode.State.current_position.x + 1
		new_y = currentNode.State.current_position.y
		if isValid(new_x, new_y, grid, rows, cols) {
			nodes_generated += 1
			new_pos := position{x: new_x, y: new_y}
			// need new state, add it to the pq
			new_state := state{current_position: new_pos, dirt_positions: currentNode.State.dirt_positions}
			new_node := Node{
				G: currentNode.G + 1,
				State: new_state,
				Parent: currentNode,
				Action: "E",
			}
			heap.Push(pq, &new_node)
		}
		// west
		new_x = currentNode.State.current_position.x - 1
		new_y = currentNode.State.current_position.y
		if isValid(new_x, new_y, grid, rows, cols) {
			nodes_generated += 1
			new_pos := position{x: new_x, y: new_y}
			// need new state, add it to the pq
			new_state := state{current_position: new_pos, dirt_positions: currentNode.State.dirt_positions}
			new_node := Node{
				G: currentNode.G + 1,
				State: new_state,
				Parent: currentNode,
				Action: "W",
			}
			heap.Push(pq, &new_node)
		}
		// Vacuum
		hasDirt := false
		dirtIndex := -1
		for i, dirt := range currentNode.State.dirt_positions {
			if dirt.x == currentNode.State.current_position.x && 
			dirt.y == currentNode.State.current_position.y {
				hasDirt = true
				dirtIndex = i
				break
			}
		}

		if hasDirt {
			nodes_generated += 1
			// create new dirt list without current position
			new_dirt := make([]position, 0, len(currentNode.State.dirt_positions)-1)
			for i, dirt := range currentNode.State.dirt_positions {
				if i != dirtIndex {
					new_dirt = append(new_dirt, dirt)
				}
			}
			new_state := state{
				current_position: currentNode.State.current_position, // same position
				dirt_positions: new_dirt,
			}
			new_node := Node{
				G: currentNode.G + 1,
				State: new_state,
				Parent: currentNode,
				Action: "V",
			}
			heap.Push(pq, &new_node)
		}
		
	}
	// we have reached goal state and gotten out of the loop
	// this means currentNode holds goal state
	// now we can walk backwards with *Node parent ptrs
	reconSlice := []string{}
	node := currentNode
	for node.Parent != nil {
		reconSlice = append(reconSlice, node.Action)
		node = node.Parent
	}
	// reverse reconSlice and print
	for i := len(reconSlice) - 1; i >= 0; i-- {
		fmt.Println(reconSlice[i])
	}
	fmt.Printf("%d nodes generated\n", nodes_generated)
	fmt.Printf("%d nodes expanded\n", nodes_expanded)


}

func main() {
	// Read command-line argument
	algorithm := os.Args[1] // arg in this pos[0,x,1,2]
	if algorithm != "uniform-cost" && algorithm != "depth-first" {
		fmt.Fprintf(os.Stderr, "Error: algorithm must be 'uniform-cost' or 'depth-first'\n")
		os.Exit(1)
	}

	// Read from stdin
	scanner := bufio.NewScanner(os.Stdin)

	// Read columns
	// go forces err to be returned
	if !scanner.Scan() {
		fmt.Fprintf(os.Stderr, "Error: failed to read columns\n")
		os.Exit(1)
	}
	cols, err := strconv.Atoi(strings.TrimSpace(scanner.Text()))
	if err != nil {
		fmt.Fprintf(os.Stderr, "Error: invalid columns: %v\n", err)
		os.Exit(1)
	}

	// Read rows
	if !scanner.Scan() {
		fmt.Fprintf(os.Stderr, "Error: failed to read rows\n")
		os.Exit(1)
	}
	rows, err := strconv.Atoi(strings.TrimSpace(scanner.Text()))
	if err != nil {
		fmt.Fprintf(os.Stderr, "Error: invalid rows: %v\n", err)
		os.Exit(1)
	}

	// Read grid
	// rune is int32
	grid := make([][]rune, rows)
	var robotStart position
	var dirtPositions []position

	for i := 0; i < rows; i++ {
		if !scanner.Scan() {
			fmt.Fprintf(os.Stderr, "Error: failed to read row %d\n", i+1)
			os.Exit(1)
		}
		line := strings.TrimSpace(scanner.Text())
		if len(line) != cols {
			fmt.Fprintf(os.Stderr, "Error: row %d has length %d, expected %d\n", i+1, len(line), cols)
			os.Exit(1)
		}
		grid[i] = make([]rune, cols)
		for j, char := range line {
			grid[i][j] = char
			pos := position{x: j, y: i}
			switch char {
			case '@':
				robotStart = pos
			case '*':
				dirtPositions = append(dirtPositions, pos)
			case '_', '#':
				// Valid characters, no action needed
			default:
				fmt.Fprintf(os.Stderr, "Error: invalid character '%c' at position (%d, %d)\n", char, i+1, j+1)
				os.Exit(1)
			}
		}
	}

	if err := scanner.Err(); err != nil {
		fmt.Fprintf(os.Stderr, "Error reading input: %v\n", err)
		os.Exit(1)
	}

	// Verify robot starting position was found
	if grid[robotStart.y][robotStart.x] != '@' {
		fmt.Fprintf(os.Stderr, "Error: robot starting position (@) not found in grid\n")
		os.Exit(1)
	}

	// - algorithm: "uniform-cost" or "depth-first"
	// - grid: 2D array of runes representing the grid
	// - robotStart: starting position of the robot
	// - dirtPositions: list of positions that need to be cleaned
	// - rows, cols: grid dimensions

	if algorithm == "uniform-cost" {
		UCS(grid, robotStart, dirtPositions, rows, cols)
	}
}
