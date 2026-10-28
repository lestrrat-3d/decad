package compositesweep

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// SewJoin sews one analytic path join: each coedge of from's loops is paired
// with the coedge at the same loop and position in to.
func SewJoin(
	ctx context.Context,
	from, to *topo.Face,
	previous *topo.Body,
	sewn map[*topo.Edge]struct{},
) error {
	for li := range from.Loops {
		for ci := range from.Loops[li].CoEdges {
			if err := ctx.Err(); err != nil {
				return err
			}
			fromUse := from.Loops[li].CoEdges[ci]
			toUse := to.Loops[li].CoEdges[ci]
			// The next span's start rim is evaluated directly in the certified
			// transported frame. Keep that edge and replace the preceding span's
			// independently rotated end rim, so the published join reads the
			// exact shared section rather than a second libm image of it.
			shared, discarded := toUse.Edge, fromUse.Edge
			fromStart, fromEnd := topobuild.CoEdgeStart(fromUse), topobuild.CoEdgeEnd(fromUse)
			toStart, toEnd := topobuild.CoEdgeStart(toUse), topobuild.CoEdgeEnd(toUse)
			RewriteVertex(previous, fromStart, toStart)
			RewriteVertex(previous, fromEnd, toEnd)
			RewriteEdge(previous, discarded, shared)
			shared.Faces = append(shared.Faces, discarded.Faces...)
			sewn[shared] = struct{}{}
		}
	}
	return nil
}

// SewTwistLoftJoin sews one join between twisted Loft cells. A Loft's start
// and end cap loops walk in opposite directions. Match the recorded station
// coordinates at a join instead of pairing coedge positions. Both adjoining
// Lofts use the same held frame, so equality is exact here.
func SewTwistLoftJoin(ctx context.Context, from, to *topo.Face, previous *topo.Body, sewn map[*topo.Edge]struct{}) error {
	if len(from.Loops) != 1 || len(to.Loops) != 1 {
		return fmt.Errorf(`%w: a twisted Loft join needs one section loop`, decaderr.ErrUnsupported)
	}
	for _, old := range from.Loops[0].CoEdges {
		if err := ctx.Err(); err != nil {
			return err
		}
		var shared topo.CoEdge
		matches := 0
		for _, candidate := range to.Loops[0].CoEdges {
			if topobuild.CoEdgeStart(old).Position == topobuild.CoEdgeStart(candidate).Position &&
				topobuild.CoEdgeEnd(old).Position == topobuild.CoEdgeEnd(candidate).Position ||
				topobuild.CoEdgeStart(old).Position == topobuild.CoEdgeEnd(candidate).Position &&
					topobuild.CoEdgeEnd(old).Position == topobuild.CoEdgeStart(candidate).Position {
				shared = candidate
				matches++
			}
		}
		if matches != 1 {
			return fmt.Errorf(`%w: a twisted Loft join has %d matching rim edges`, decaderr.ErrUnsupported, matches)
		}
		var start, end *topo.Vertex
		if topobuild.CoEdgeStart(old).Position == topobuild.CoEdgeStart(shared).Position {
			start, end = topobuild.CoEdgeStart(shared), topobuild.CoEdgeEnd(shared)
		} else {
			start, end = topobuild.CoEdgeEnd(shared), topobuild.CoEdgeStart(shared)
		}
		RewriteVertex(previous, topobuild.CoEdgeStart(old), start)
		RewriteVertex(previous, topobuild.CoEdgeEnd(old), end)
		discarded := old.Edge
		RewriteEdge(previous, discarded, shared.Edge)
		shared.Edge.Faces = append(shared.Edge.Faces, discarded.Faces...)
		sewn[shared.Edge] = struct{}{}
	}
	return nil
}

// RewriteVertex replaces old with replacement at every edge end of body.
func RewriteVertex(body *topo.Body, old, replacement *topo.Vertex) {
	if old == replacement {
		return
	}
	for _, lump := range body.Lumps {
		for _, shell := range lump.Shells {
			for _, face := range shell.Faces {
				for _, loop := range face.Loops {
					for _, use := range loop.CoEdges {
						edge := use.Edge
						if edge.Start == old {
							edge.Start = replacement
						}
						if edge.End == old {
							edge.End = replacement
						}
					}
				}
			}
		}
	}
}

// RewriteEdge replaces old with replacement in every coedge of body's faces.
func RewriteEdge(body *topo.Body, old, replacement *topo.Edge) {
	for _, face := range topobuild.Faces(body) {
		for _, loop := range face.Loops {
			for i := range loop.CoEdges {
				if loop.CoEdges[i].Edge == old {
					loop.CoEdges[i].Edge = replacement
				}
			}
		}
	}
}

// LiveFaces returns faces without nil entries, repeats, or faces that do not
// belong to body, keeping the first occurrence's order.
func LiveFaces(faces []*topo.Face, body *topo.Body) []*topo.Face {
	out := make([]*topo.Face, 0, 2)
	for _, face := range faces {
		if face == nil || face.Body != body || slices.Contains(out, face) {
			continue
		}
		out = append(out, face)
	}
	return out
}
