package loftcylinder

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// ChamferSelectsOuterLoops checks that edges are exactly the two complete
// outer cap loops of body. Each of start and end must be the origin of one
// face, and the first loop of each such face must walk its recorded outer
// ring (ChamferCapWalk). It never interprets an arbitrary faceted edge as an
// offset carrier.
func ChamferSelectsOuterLoops(body *topo.Body, start, end provenance.Ref,
	edges []*topo.Edge, verts []r3.Vec, outer [2][]int) error {
	var caps [2]*topo.Face
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
			if origin == start {
				if caps[0] != nil {
					return fmt.Errorf("%w: loft/cylinder start cap is split", decaderr.ErrUnsupported)
				}
				caps[0] = face
			}
			if origin == end {
				if caps[1] != nil {
					return fmt.Errorf("%w: loft/cylinder end cap is split", decaderr.ErrUnsupported)
				}
				caps[1] = face
			}
		}
	}
	if caps[0] == nil || caps[1] == nil {
		return fmt.Errorf("%w: loft/cylinder result has no current cap faces", decaderr.ErrUnsupported)
	}
	selected := make(map[*topo.Edge]struct{}, len(edges))
	for _, edge := range edges {
		selected[edge] = struct{}{}
	}
	if len(selected) != len(edges) {
		return fmt.Errorf("%w: loft/cylinder chamfer repeats an edge", decaderr.ErrUnsupported)
	}
	for cap, face := range caps {
		if len(face.Loops) == 0 || !ChamferCapWalk(face.Loops[0], verts, outer[cap], cap == 0) {
			return fmt.Errorf("%w: loft/cylinder cap %d has no outer loop", decaderr.ErrUnsupported, cap)
		}
		for _, ce := range face.Loops[0].CoEdges {
			if _, yes := selected[ce.Edge]; !yes {
				return fmt.Errorf("%w: loft/cylinder chamfer needs both complete outer cap loops", decaderr.ErrUnsupported)
			}
			delete(selected, ce.Edge)
		}
	}
	if len(selected) != 0 {
		return fmt.Errorf("%w: loft/cylinder chamfer selection includes a bore or side edge", decaderr.ErrUnsupported)
	}
	return nil
}

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
