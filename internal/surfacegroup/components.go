// Package surfacegroup groups connected surface elements by their recorded topology.
package surfacegroup

// Connected partitions faces by shared edges. It preserves each face's input
// order and orders groups by their first face in the input.
func Connected[F comparable, E comparable](faces []F, byEdge map[E][]F) [][]F {
	parent := make(map[F]F, len(faces))
	for _, face := range faces {
		parent[face] = face
	}
	for _, group := range byEdge {
		for i := 1; i < len(group); i++ {
			a, b := root(parent, group[0]), root(parent, group[i])
			if a != b {
				parent[a] = b
			}
		}
	}
	order := make([]F, 0, len(faces))
	groups := make(map[F][]F)
	for _, face := range faces {
		representative := root(parent, face)
		if _, ok := groups[representative]; !ok {
			order = append(order, representative)
		}
		groups[representative] = append(groups[representative], face)
	}
	out := make([][]F, len(order))
	for i, representative := range order {
		out[i] = groups[representative]
	}
	return out
}

// ChainCount counts connected components of undirected edges by endpoint identity.
func ChainCount[V comparable](edges [][2]V) int {
	parent := make(map[V]V)
	for _, edge := range edges {
		if _, ok := parent[edge[0]]; !ok {
			parent[edge[0]] = edge[0]
		}
		if _, ok := parent[edge[1]]; !ok {
			parent[edge[1]] = edge[1]
		}
	}
	for _, edge := range edges {
		a, b := root(parent, edge[0]), root(parent, edge[1])
		if a != b {
			parent[a] = b
		}
	}
	roots := make(map[V]struct{})
	for vertex := range parent {
		roots[root(parent, vertex)] = struct{}{}
	}
	return len(roots)
}

func root[T comparable](parent map[T]T, element T) T {
	for parent[element] != element {
		parent[element] = parent[parent[element]]
		element = parent[element]
	}
	return element
}
