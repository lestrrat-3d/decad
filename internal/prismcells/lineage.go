package prismcells

import (
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// RawEdgePieces identifies the contiguous split edges made from one raw edge.
type RawEdgePieces struct{ First, Last int }

// ProfileLineage indexes raw outer and hole edges into split cell walks.
type ProfileLineage struct {
	Outer []RawEdgePieces
	Holes [][]RawEdgePieces
}

// SplitRunsWithLineage splits the single raw publication and verifies that each
// output walk partitions its raw edge ranges with identical entities and ends.
func SplitRunsWithLineage(budget *proofbound.WorkBudget,
	raw []*sketch.Profile) ([]*sketch.Profile, []ProfileLineage, bool, error) {
	split, ok, err := splitRuns(budget, raw, true)
	if err != nil || !ok {
		return nil, nil, ok, err
	}
	if len(raw) != len(split) {
		return nil, nil, false, nil
	}
	lineage := make([]ProfileLineage, len(raw))
	for i := range raw {
		if err := budget.Step(); err != nil {
			return nil, nil, false, err
		}
		outer, good, e := traceRawLoop(budget, raw[i].Outer, split[i].Outer)
		if e != nil || !good {
			return nil, nil, good, e
		}
		if len(raw[i].Holes) != len(split[i].Holes) {
			return nil, nil, false, nil
		}
		lineage[i].Outer = outer
		lineage[i].Holes = make([][]RawEdgePieces, len(raw[i].Holes))
		for h := range raw[i].Holes {
			lineage[i].Holes[h], good, e = traceRawLoop(budget,
				raw[i].Holes[h], split[i].Holes[h])
			if e != nil || !good {
				return nil, nil, good, e
			}
		}
	}
	return split, lineage, true, nil
}

func traceRawLoop(budget *proofbound.WorkBudget,
	raw, split []sketch.BoundaryEdge) ([]RawEdgePieces, bool, error) {
	if len(raw) == 0 {
		return nil, false, nil
	}
	out := make([]RawEdgePieces, len(raw))
	next := 0
	for ri, source := range raw {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		if next >= len(split) || len(source.Polyline) < 2 ||
			source.TStart >= source.TEnd {
			return nil, false, nil
		}
		first := next
		expected := source.TStart
		if source.Reversed {
			expected = source.TEnd
		}
		for {
			if next >= len(split) {
				return nil, false, nil
			}
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			piece := split[next]
			if piece.Entity != source.Entity || piece.Reversed != source.Reversed ||
				piece.TStart >= piece.TEnd || len(piece.Polyline) < 2 {
				return nil, false, nil
			}
			if source.Reversed {
				if piece.TEnd != expected || piece.TStart < source.TStart {
					return nil, false, nil
				}
				expected = piece.TStart
			} else {
				if piece.TStart != expected || piece.TEnd > source.TEnd {
					return nil, false, nil
				}
				expected = piece.TEnd
			}
			if next == first {
				if piece.Polyline[0] != source.Polyline[0] {
					return nil, false, nil
				}
			} else if split[next-1].Polyline[len(split[next-1].Polyline)-1] !=
				piece.Polyline[0] {
				return nil, false, nil
			}
			next++
			if !source.Reversed && expected == source.TEnd ||
				source.Reversed && expected == source.TStart {
				if piece.Polyline[len(piece.Polyline)-1] !=
					source.Polyline[len(source.Polyline)-1] {
					return nil, false, nil
				}
				break
			}
		}
		out[ri] = RawEdgePieces{First: first, Last: next - 1}
	}
	if next != len(split) {
		return nil, false, nil
	}
	return out, true, nil
}
