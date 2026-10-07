package stitchweld

// Use records one directed use of an edge by a face.
type Use[E comparable] struct {
	Edge    E
	Forward bool
}

// Topology preserves face, loop, and coedge order for the orientation and
// closure checks. Adjacent is the edge's recorded face list, not a list
// inferred from Uses.
type Topology[F comparable, E comparable] struct {
	Faces    []F
	Uses     map[F][]Use[E]
	Adjacent map[E][]F
}

func (t Topology[F, E]) directionFor(f F, e E) (bool, bool) {
	for _, use := range t.Uses[f] {
		if use.Edge == e {
			return use.Forward, true
		}
	}
	return false, false
}

// Orientation finds the whole-face flips required for opposite directed uses
// across each two-face edge. The bool is false for a contradictory cycle.
func (t Topology[F, E]) Orientation() (map[F]bool, bool) {
	flip := map[F]bool{}
	for _, root := range t.Faces {
		if _, ok := flip[root]; ok {
			continue
		}
		flip[root] = false
		queue := []F{root}
		for len(queue) > 0 {
			f := queue[0]
			queue = queue[1:]
			for _, use := range t.Uses[f] {
				e := use.Edge
				adjacent := t.Adjacent[e]
				if len(adjacent) != 2 {
					continue
				}
				other := adjacent[0]
				if other == f {
					other = adjacent[1]
				}
				if other == f {
					continue
				}
				dirOther, ok := t.directionFor(other, e)
				if !ok {
					continue
				}
				effectiveF := use.Forward != flip[f]
				wantOther := !effectiveF
				flipOther := dirOther != wantOther
				if existing, seen := flip[other]; seen {
					if existing != flipOther {
						return nil, false
					}
					continue
				}
				flip[other] = flipOther
				queue = append(queue, other)
			}
		}
	}
	return flip, true
}

// ClosureViolation identifies the first edge whose recorded adjacency and
// directed coedge uses disagree.
type ClosureViolation uint8

const (
	ClosureValid ClosureViolation = iota
	ClosureOneFaceUses
	ClosureTwoFaceParity
	ClosureTooManyFaces
)

// CheckClosure checks edges in first-use order, including open one-face edges.
func (t Topology[F, E]) CheckClosure() ClosureViolation {
	uses := map[E]int{}
	forward := map[E]int{}
	backward := map[E]int{}
	var order []E
	seen := map[E]struct{}{}
	for _, f := range t.Faces {
		for _, use := range t.Uses[f] {
			e := use.Edge
			if _, ok := seen[e]; !ok {
				seen[e] = struct{}{}
				order = append(order, e)
			}
			uses[e]++
			if use.Forward {
				forward[e]++
			} else {
				backward[e]++
			}
		}
	}
	for _, e := range order {
		switch len(t.Adjacent[e]) {
		case 1:
			if uses[e] != 1 {
				return ClosureOneFaceUses
			}
		case 2:
			if uses[e] != 2 || forward[e] != 1 || backward[e] != 1 {
				return ClosureTwoFaceParity
			}
		default:
			return ClosureTooManyFaces
		}
	}
	return ClosureValid
}
