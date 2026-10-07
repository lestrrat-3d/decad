package prismcells

import (
	"sort"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// SplitRuns restates every returned cell's line edges at the arrangement
// vertices other cells report on the same line, so every cell names a shared
// span by one (entity, range) key. sketch reports a cell's boundary as runs:
// where a cell walks one line straight through vertices that only bound
// cells on the line's other side, it reports one edge over the whole run.
// Two outlines sharing part of a wall give exactly that: the cell on one
// side walks the named line whole, the cell on the other the shared span
// alone, and the merge's count (Merge) and the classification's links
// (Classify) match edges by entity and range. The split points are the line
// parameters some other edge of the same line ends at, each with that edge's
// own Polyline end, which is the vertex sketch split the line at.
//
// A piece's TExact is the conjunction of the run's own and that of every
// edge whose end it takes, so an uncertified bound stays uncertified, and
// recordEdge refuses it as it refuses any other. A circular run is not split,
// since a piece of one would need its own densified Polyline: a circle or arc
// edge holding another edge's end strictly inside its range makes the scene
// unresolved (resolved=false, err nil). A cell whose lines need no split is
// returned as reported; the slice is a copy only where a cell changed.
func SplitRuns(budget *proofbound.WorkBudget, profiles []*sketch.Profile) ([]*sketch.Profile, bool, error) {
	type end struct {
		point [2]float64
		exact bool
	}
	ends := map[sketch.Entity]map[float64]end{}
	note := func(ent sketch.Entity, t float64, p [2]float64, exact bool) {
		m := ends[ent]
		if m == nil {
			m = map[float64]end{}
			ends[ent] = m
		}
		if prev, ok := m[t]; ok {
			exact = exact && prev.exact
		}
		m[t] = end{point: p, exact: exact}
	}
	for _, p := range profiles {
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.Step(); err != nil {
					return nil, false, err
				}
				if len(e.Polyline) < 2 {
					continue
				}
				first, last := e.Polyline[0], e.Polyline[len(e.Polyline)-1]
				if e.Reversed {
					first, last = last, first
				}
				exact := e.TExact || !e.Partial
				note(e.Entity, e.TStart, first, exact)
				note(e.Entity, e.TEnd, last, exact)
			}
		}
	}

	resolved := true
	split := func(loop []sketch.BoundaryEdge) ([]sketch.BoundaryEdge, bool) {
		var out []sketch.BoundaryEdge
		changed := false
		for _, e := range loop {
			if len(e.Polyline) < 2 {
				out = append(out, e)
				continue
			}
			var cuts []float64
			for t := range ends[e.Entity] {
				if t > e.TStart && t < e.TEnd {
					cuts = append(cuts, t)
				}
			}
			if len(cuts) == 0 {
				out = append(out, e)
				continue
			}
			if _, isLine := e.Entity.(*sketch.Line); !isLine {
				resolved = false
				out = append(out, e)
				continue
			}
			changed = true
			sort.Float64s(cuts)
			first, last := e.Polyline[0], e.Polyline[len(e.Polyline)-1]
			if e.Reversed {
				first, last = last, first
			}
			params := append(append([]float64{e.TStart}, cuts...), e.TEnd)
			points := make([][2]float64, len(params))
			exact := make([]bool, len(params))
			points[0], exact[0] = first, e.TExact || !e.Partial
			points[len(params)-1], exact[len(params)-1] = last, e.TExact || !e.Partial
			for i, t := range cuts {
				c := ends[e.Entity][t]
				points[i+1], exact[i+1] = c.point, c.exact
			}
			pieces := make([]sketch.BoundaryEdge, len(params)-1)
			for i := range pieces {
				piece := e
				piece.TStart, piece.TEnd = params[i], params[i+1]
				piece.Partial = true
				piece.TExact = exact[i] && exact[i+1]
				piece.Polyline = [][2]float64{points[i], points[i+1]}
				if e.Reversed {
					piece.Polyline = [][2]float64{points[i+1], points[i]}
				}
				pieces[i] = piece
			}
			if e.Reversed {
				for i, j := 0, len(pieces)-1; i < j; i, j = i+1, j-1 {
					pieces[i], pieces[j] = pieces[j], pieces[i]
				}
			}
			out = append(out, pieces...)
		}
		return out, changed
	}

	out := make([]*sketch.Profile, len(profiles))
	for i, p := range profiles {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		outer, changed := split(p.Outer)
		holes := make([][]sketch.BoundaryEdge, len(p.Holes))
		for h, loop := range p.Holes {
			var c bool
			holes[h], c = split(loop)
			changed = changed || c
		}
		if !changed {
			out[i] = p
			continue
		}
		cp := *p
		cp.Outer = outer
		if p.Holes != nil {
			cp.Holes = holes
		}
		out[i] = &cp
	}
	if !resolved {
		return nil, false, nil
	}
	return out, true, nil
}
