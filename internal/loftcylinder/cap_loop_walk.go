package loftcylinder

import (
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// ChamferCapWalk checks that public coedges partition the audited ring once
// in the cap's walk direction. The cap audit proves the mesh boundary is
// exactly ring; public edges may coalesce adjacent segments.
func ChamferCapWalk(loop *topo.Loop, verts []r3.Vec, ring []int, bottom bool) bool {
	if loop == nil || !loop.Outer || len(loop.CoEdges) < 3 || len(loop.CoEdges) > len(ring) {
		return false
	}
	index := make(map[r3.Vec]int, len(ring))
	for i, vertex := range ring {
		if vertex < 0 || vertex >= len(verts) {
			return false
		}
		if _, duplicate := index[verts[vertex]]; duplicate {
			return false
		}
		index[verts[vertex]] = i
	}
	covered := 0
	for i, use := range loop.CoEdges {
		if use.Edge == nil {
			return false
		}
		start, end := use.Edge.Start, use.Edge.End
		if !use.Forward {
			start, end = end, start
		}
		if start == nil || end == nil {
			return false
		}
		next := loop.CoEdges[(i+1)%len(loop.CoEdges)]
		nextStart := next.Edge.Start
		if !next.Forward {
			nextStart = next.Edge.End
		}
		if end.Position != nextStart.Position {
			return false
		}
		a, okA := index[start.Position]
		b, okB := index[end.Position]
		if !okA || !okB {
			return false
		}
		steps := (b - a + len(ring)) % len(ring)
		if bottom {
			steps = (a - b + len(ring)) % len(ring)
		}
		if steps == 0 {
			return false
		}
		covered += steps
		if covered > len(ring) {
			return false
		}
	}
	return covered == len(ring)
}
