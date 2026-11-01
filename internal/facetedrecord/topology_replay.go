package facetedrecord

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func buildTopologyReplay(ctx context.Context, b *proofbound.WorkBudget, alloc topo.Alloc, verts []r3.Vec, tris [][3]int, facetFace []*topo.Face, vertexBound []float64, axial bool) error {
	if b.ReplayID == nil {
		return errBrakeReplayMissingReplayTopologyMeter
	}
	ids, err := topo.NewFaceBoundTable(ctx, len(facetFace), b.ReplayID)
	if err != nil {
		return err
	}
	if err := b.ReserveBytesMul(uint64(len(facetFace)), uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
		return err
	}
	if err := b.ReserveBytesMul(uint64(len(facetFace)), uint64(unsafe.Sizeof(int(0)))); err != nil {
		return err
	}
	if err := b.ReserveBytesMul(uint64(len(facetFace)), uint64(unsafe.Sizeof(false))); err != nil {
		return err
	}
	faces := make([]*topo.Face, 0, len(facetFace))
	idsOf := make([]int, len(facetFace))
	planar := make([]bool, 0, len(facetFace))
	for i, f := range facetFace {
		id, ok, err := ids.Lookup(ctx, f, b.ReplayID)
		if err != nil {
			return err
		}
		if !ok {
			id = float64(len(faces))
			if err := ids.Set(ctx, f, id, b.ReplayID); err != nil {
				return err
			}
			faces = append(faces, f)
			planar = append(planar, f.HeldPlanar)
		}
		idsOf[i] = int(id)
	}
	plan, err := facetedtopology.BuildWithReplay(ctx, b, verts, tris, idsOf, planar, vertexBound)
	if err != nil {
		return err
	}
	if err := b.ReserveBytesMul(uint64(len(verts)), uint64(unsafe.Sizeof((*topo.Vertex)(nil)))); err != nil {
		return err
	}
	if err := b.ReserveBytesMul(uint64(len(plan.Edges)), uint64(unsafe.Sizeof((*topo.Edge)(nil)))); err != nil {
		return err
	}
	vertices := make([]*topo.Vertex, len(verts))
	edges := make([]*topo.Edge, len(plan.Edges))
	vertexOf := func(vi int) (*topo.Vertex, error) {
		if vertices[vi] != nil {
			return vertices[vi], nil
		}
		if err := reserveReplayNode(b, alloc, "vertex"); err != nil {
			return nil, err
		}
		vertices[vi] = topobuild.NewVertex(alloc, topo.Vertex{Position: verts[vi], Bound: units.Millimeters(vertexBound[vi])})
		return vertices[vi], nil
	}
	for i, chain := range plan.Edges {
		a, c := faces[chain.Faces[0]], faces[chain.Faces[1]]
		_, aPlane := a.Surface.(surfacegeom.Plane)
		_, cPlane := c.Surface.(surfacegeom.Plane)
		// Denoted and curved tags require an exact proof that this replay path
		// cannot yet meter. Keep the faceted curve and its original ChainBound;
		// this changes edge metadata only, not the held mesh or its topology.
		proveCurve := a.RetainedTag && c.RetainedTag && a.Denoted == nil && c.Denoted == nil && aPlane && cPlane
		if proveCurve {
			// The plane-plane path scans the chain at most twice and uses
			// fixed-size floating-point calculations for one candidate line.
			n := uint64(len(chain.Vertices))
			if n > (proofbound.MaxMultiwayWorkPerCall-128)/128 {
				return errBrakeReplayReplayRetainedCurveChainExceedsWorkLimit
			}
			if _, err := b.StepN(128 + 128*n); err != nil {
				return err
			}
			if err := b.Reserve(2*n + 1); err != nil {
				return err
			}
		}
		if err := b.Step(); err != nil {
			return err
		}
		if err := b.ReserveBytesMul(uint64(len(chain.Vertices)), uint64(unsafe.Sizeof(r3.Vec{}))); err != nil {
			return err
		}
		if err := b.ReserveBytesMul(uint64(len(chain.Vertices)), uint64(unsafe.Sizeof(float64(0)))); err != nil {
			return err
		}
		var curve surfacegeom.Curve = surfacegeom.FacetedCurve{Bound: units.Millimeters(chain.Bound)}
		retained := false
		if proveCurve {
			if analytic, ok := retainedRimCurve(chain, faces, verts, vertexBound); ok {
				curve, retained = analytic, true
			}
		}
		start, err := vertexOf(chain.Vertices[0])
		if err != nil {
			return err
		}
		end, err := vertexOf(chain.Vertices[len(chain.Vertices)-1])
		if err != nil {
			return err
		}
		if err := reserveReplayNode(b, alloc, "edge"); err != nil {
			return err
		}
		edge := topobuild.NewEdge(alloc, topo.Edge{Convex: chain.Convex, Length: chain.Length, LengthBound: chain.LengthBound, LengthUnbounded: chain.LengthUnbounded, Curve: curve, RetainedCurve: retained, ChainBound: chain.Bound, Start: start, End: end, Faces: []*topo.Face{faces[chain.Faces[0]], faces[chain.Faces[1]]}})
		if axial {
			if _, err := b.StepN(uint64(len(chain.Vertices))); err != nil {
				return err
			}
			edge.SelectorDirection, edge.SelectorDirectionProven = facetedtopology.AxialChainDirection(verts, chain.Vertices)
		}
		edges[i] = edge
	}
	for fi, loops := range plan.ReplayFaceLoops {
		if len(loops) == 0 {
			continue
		}
		if fi >= len(faces) {
			return errBrakeReplayReplayLoopFaceAbsent
		}
		f := faces[fi]
		if err := b.ReserveBytesMul(uint64(len(loops)), uint64(unsafe.Sizeof((*topo.Loop)(nil)))); err != nil {
			return err
		}
		newLoops := make([]*topo.Loop, 0, len(loops))
		for _, p := range loops {
			if err := reserveReplayNode(b, alloc, "loop"); err != nil {
				return err
			}
			if err := b.ReserveBytesMul(uint64(len(p.Coedges)), uint64(unsafe.Sizeof(topo.CoEdge{}))); err != nil {
				return err
			}
			loop := topobuild.NewLoop(alloc, topo.Loop{Outer: p.Outer})
			loop.CoEdges = make([]topo.CoEdge, 0, len(p.Coedges))
			for _, ce := range p.Coedges {
				idx := -1
				for i, edge := range plan.Edges {
					if err := b.Step(); err != nil {
						return err
					}
					if edge == ce.Edge {
						idx = i
						break
					}
				}
				if idx < 0 {
					return errBrakeReplayReplayLoopEdgeAbsent
				}
				loop.CoEdges = append(loop.CoEdges, topo.CoEdge{Edge: edges[idx], Forward: ce.Forward})
			}
			newLoops = append(newLoops, loop)
		}
		if err := b.ReserveBytesMul(uint64(len(f.Loops)+len(newLoops)), uint64(unsafe.Sizeof((*topo.Loop)(nil)))); err != nil {
			return err
		}
		combined := make([]*topo.Loop, 0, len(f.Loops)+len(newLoops))
		combined = append(combined, f.Loops...)
		combined = append(combined, newLoops...)
		f.Loops = combined
	}
	return nil
}
