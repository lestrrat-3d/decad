package topobuild

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file holds the node helpers every flat-triangle loft-family builder
// (loft, chain loft, circle-bore loft, coil, twisted sweep, mitred sweep)
// shares: the vertex and edge a table index names, a face's Plane read from
// its own triangle, the whole-shell walk reversal, and the finiteness gate on
// the body's four measurements. See docs/loft-design.md §5.1 and §7.

// LoftVertex builds a vertex at a recorded (or lifted-from-recorded)
// coordinate: every loft vertex position comes from Plane.Origin + p.U*Plane.U
// + p.V*Plane.V, the identical single float64 evaluation Extrude already
// performs for a cap vertex (§5), so a vertex of a build whose delta is zero
// carries the same zero-bound standing; any other vertex carries the payload's
// own delta (§12 PR 2a). Zero delta is NOT the same claim as unplaced: an
// unplaced build still carries a positive delta wherever a station was
// COMPUTED rather than pinned (loftPayload's own delta doc comment).
func LoftVertex(alloc topo.Alloc, p r3.Vec, delta float64) *topo.Vertex {
	return NewVertex(alloc, topo.Vertex{Position: p, Bound: units.Millimeters(delta)})
}

// LoftEdge builds one straight loft edge between two vertex-table indices,
// with the given walked-boundary convexity.
func LoftEdge(alloc topo.Alloc, vertices []*topo.Vertex, positions []r3.Vec, a, b int, convex bool, delta float64) *topo.Edge {
	held, bound := loftmesh.EdgeLength(positions[a], positions[b], delta)
	return NewEdge(alloc, topo.Edge{Convex: convex, Length: held, LengthBound: bound, Curve: surfacegeom.Line3{}, Start: vertices[a], End: vertices[b]})
}

// PlaneFromTriangle builds a face's Plane surface directly from one of its
// own (already outward-oriented) triangles: origin at its first vertex, U and
// V its two edge vectors. r3.NewFrame orthonormalizes them (Gram-Schmidt in
// effect), and the resulting normal U×V is (B-A)x(C-A) up to positive
// scaling — the outward normal of an outward-wound triangle, so the face's
// `reversed` flag stays false (§5's wall-face row: every wall face is a Plane
// wound outward). The Frame is the exact answer for the three vertices handed
// to it whatever their own standing — §5's surface-parameter carve-out — while
// the face's own area and its vertices' positions carry the payload's delta.
func PlaneFromTriangle(verts []r3.Vec, tri [3]int) (surfacegeom.Plane, error) {
	a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
	f, err := r3.NewFrame(a, b.Sub(a), c.Sub(a))
	if err != nil {
		return surfacegeom.Plane{}, fmt.Errorf(`%w: a loft triangle has no plane: %s`, decaderr.ErrDegenerate, err)
	}
	return surfacegeom.Plane{Frame: f}, nil
}

// LoftLoopCoedges carries §5's whole-shell reversal into one face's directed
// boundary. Every walk a loft-family topology builder emits is written from
// the LOCAL vertex order of §5's construction table, which is the order the
// triangles had BEFORE the whole-shell step; a face's Plane, by contrast, is
// rebuilt from its own already-flipped triple (PlaneFromTriangle), so on a
// reversed shell the two disagree and the published boundary — Loop.CoEdges,
// CoEdge.Start/End/IsForward — walks the material on the RIGHT of the face's
// own outward normal, the opposite of decad's material-on-the-left convention.
//
// Reversing a walk is reversing its coedge order and negating each use's
// sense; nothing but the direction changes. The edge identities and their
// count are untouched, so every edge still bounds exactly the same two faces
// and Loop.Edges' undirected view is merely re-ordered.
func LoftLoopCoedges(co []topo.CoEdge, reversed bool) []topo.CoEdge {
	if !reversed {
		return co
	}
	out := make([]topo.CoEdge, len(co))
	for i, ce := range co {
		out[len(co)-1-i] = topo.CoEdge{Edge: ce.Edge, Forward: !ce.Forward}
	}
	return out
}

// ValidateLoftBodyMeasurements is a loft-family build's own finiteness gate
// (design O2); loftmesh.ValidateBodyReadings owns the rule.
func ValidateLoftBodyMeasurements(body *topo.Body) error {
	return loftmesh.ValidateBodyReadings(loftmesh.BodyReadings{
		Volume: body.Volume.Value.Base(), VolumeBound: body.Volume.Bound.Base(),
		Area:     body.Area.Value.Base(),
		Centroid: body.Centroid.Value, CentroidBound: body.Centroid.Bound.Base(),
		BoundsMin: body.Bounds.Min, BoundsMax: body.Bounds.Max,
		BoundsBound: body.Bounds.Bound.Base(),
	})
}
