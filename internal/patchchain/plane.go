package patchchain

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Vertex holds the position and absolute displacement bound of a chain vertex.
type Vertex struct {
	Position r3.Vec
	Bound    float64
}

// Edge holds the exactness and carrier-plane facts of a selected edge.
type Edge struct {
	Exact        bool
	Center, Axis r3.Vec
	Curved       bool
}

// ProvePlane proves every vertex and curved carrier lies in one exact plane.
// Vertex order is the first-seen order of the root topology adapter.
func ProvePlane(verts []Vertex, edges []Edge) error {
	for _, v := range verts {
		if !proofbound.FiniteVec(v.Position) || v.Bound != 0 {
			return fmt.Errorf(`%w: a Body.Patch chain vertex carries a nonzero bound, so its position is not proven exactly (docs/surface-design.md Table R row R6)`, decaderr.ErrUnsupported)
		}
	}
	for _, e := range edges {
		if !e.Exact {
			return fmt.Errorf(`%w: a Body.Patch chain edge's own curve carries a nonzero bound, so its geometry is not proven exactly (docs/surface-design.md Table R row R6)`, decaderr.ErrUnsupported)
		}
		if e.Curved && (!proofbound.FiniteVec(e.Center) || !proofbound.FiniteVec(e.Axis)) {
			return fmt.Errorf(`%w: a Body.Patch chain edge's carrier plane is not representable (docs/surface-design.md Table R row R6)`, decaderr.ErrUnsupported)
		}
	}

	normal, origin, ok := planeFromVertices(verts)
	if !ok {
		normal, origin, ok = planeFromCarrier(edges)
	}
	if !ok {
		return fmt.Errorf(`%w: a Body.Patch chain has too few independent points to determine a plane (docs/surface-design.md Table R row R6)`, decaderr.ErrUnsupported)
	}

	normalDy := proofarith.DyVec(normal)
	originDy := proofarith.DyVec(origin)
	for _, v := range verts {
		rel := proofarith.DvSub(proofarith.DyVec(v.Position), originDy)
		if !proofarith.DvDot(normalDy, rel).IsZero() {
			return fmt.Errorf(`%w: a Body.Patch chain is not planar (docs/surface-design.md Table R row R6)`, decaderr.ErrUnsupported)
		}
	}
	for _, e := range edges {
		if !e.Curved {
			continue
		}
		if !proofarith.DvIsZero(proofarith.DvCross(proofarith.DyVec(e.Axis), normalDy)) {
			return fmt.Errorf(`%w: a Body.Patch chain edge's carrier plane does not match the chain's plane (docs/surface-design.md Table R row R6)`, decaderr.ErrUnsupported)
		}
		rel := proofarith.DvSub(proofarith.DyVec(e.Center), originDy)
		if !proofarith.DvDot(normalDy, rel).IsZero() {
			return fmt.Errorf(`%w: a Body.Patch chain edge's carrier plane does not match the chain's plane (docs/surface-design.md Table R row R6)`, decaderr.ErrUnsupported)
		}
	}
	return nil
}

func planeFromVertices(verts []Vertex) (normal, origin r3.Vec, ok bool) {
	if len(verts) < 3 {
		return r3.Vec{}, r3.Vec{}, false
	}
	v0 := verts[0].Position
	d0 := proofarith.DyVec(v0)
	for i := 1; i < len(verts); i++ {
		vi := verts[i].Position
		di := proofarith.DvSub(proofarith.DyVec(vi), d0)
		if proofarith.DvIsZero(di) {
			continue
		}
		for j := i + 1; j < len(verts); j++ {
			vj := verts[j].Position
			dj := proofarith.DvSub(proofarith.DyVec(vj), d0)
			cross := proofarith.DvCross(di, dj)
			if !proofarith.DvIsZero(cross) {
				return vi.Sub(v0).Cross(vj.Sub(v0)), v0, true
			}
		}
	}
	return r3.Vec{}, r3.Vec{}, false
}

func planeFromCarrier(edges []Edge) (normal, origin r3.Vec, ok bool) {
	for _, e := range edges {
		if e.Curved {
			return e.Axis, e.Center, true
		}
	}
	return r3.Vec{}, r3.Vec{}, false
}
