package patchchain

import (
	"errors"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// ProveChainPlane is gate 3: proves one chain's edges lie in a single
// plane. The first, exact arm (proveChainPlaneExact) is tried first and
// is the whole of what earlier increments proved; a second, LEVEL arm
// (ProvePlaneLevel) is tried only when the first refuses on a
// nonzero bound, and never reads a coordinate, a residual or a bound
// magnitude (docs/surface-design.md §5.2's own two-arm amendment). Its
// return states what the level arm admitted: the denotation.LevelToken and axialBound
// are what buildFace reads for the new face's own plane and AxialDelta,
// mirroring what a prism cap already carries; a chain the exact arm admits
// instead publishes the zero token and a zero axialBound, unchanged.
func ProveChainPlane(edges []*topo.Edge) (denotation.LevelToken, float64, bool, error) {
	exactErr := proveChainPlaneExact(edges)
	if exactErr == nil {
		return denotation.LevelToken{}, 0, false, nil
	}
	if !errors.Is(exactErr, decaderr.ErrUnsupported) {
		return denotation.LevelToken{}, 0, false, exactErr
	}
	if lvl, bound, ok := ProvePlaneLevel(edges); ok {
		return lvl, bound, true, nil
	}
	return denotation.LevelToken{}, 0, false, exactErr
}

// proveChainPlaneExact is gate 3's first, exact arm: proves one chain's
// edges lie in a single plane, decided over dyadic.go's exact rational lift
// as a zero determinant, never a residual against a fitted plane
// (docs/surface-design.md §5.2).
//
// Every chain vertex must carry a zero bound, and every chain edge's own
// geometry must be exact (edgeGeometryExact) — a bounded vertex or edge
// is only known to lie within its own bound, and fitting a plane to it would
// be exactly the fitted geometry §1.3 refuses. Both are [ErrUnsupported]
// (Table R row R6), which ProveChainPlane's own second arm then gets a
// chance to admit instead.
//
// Three or more vertices that are not all collinear give the plane a
// determinant to take: planeFromVertices searches for two vertex
// differences from a common point whose exact cross product is nonzero, and
// that cross product IS the plane's (unoriented) normal. Under three
// independent vertices — a lone closed circular edge has exactly one, a
// two-edge chain of two arcs has exactly two — there is no such determinant,
// and §5.2 is silent on what decides the plane there; this evaluator decides
// it from a curved edge's own carrier plane instead (planeFromCarrier):
// a Circle3 or Arc3's Center and Axis already state a plane, and that is the
// only plane its whole curve — not just its two endpoints — can lie in.
//
// Once a candidate plane is in hand, from either source, every chain vertex
// must lie on it (an exact dot product against the plane normal), and every
// curved edge's OWN carrier plane must equal it (its Axis parallel to the
// proven normal, its Center on the proven plane) — a curved edge is planar
// only when its own carrier plane is the chain's, never merely its two
// endpoints. Any failure is a chain proven NON-planar, also
// [ErrUnsupported] (R6): decad's reject-only rule treats "not proven
// planar" and "proven non-planar" alike, since neither admits the chain.
func proveChainPlaneExact(edges []*topo.Edge) error {
	verts := ChainVertices(edges)
	points := make([]Vertex, len(verts))
	for i, v := range verts {
		points[i] = Vertex{Position: v.Position, Bound: v.Bound.Base()}
	}
	curves := make([]Edge, len(edges))
	for i, e := range edges {
		center, axis, curved := curveCarrier(e)
		curves[i] = Edge{Exact: edgeGeometryExact(e), Center: center, Axis: axis, Curved: curved}
	}
	return ProvePlane(points, curves)
}

// edgeGeometryExact reports whether e's own curve is proven exact,
// filling what §5.2 leaves silent about what "a curve carrying a nonzero
// bound" means at this gate. A Line3's whole geometry lives in its two
// vertices, already checked separately, so its own proven length bound —
// the same one Length()'s Exactness answers from — is a sound reading of it
// and is zero whenever they are exact.
//
// Circle3 and Arc3 carry Center, Axis and Radius with no bound field of
// their own (docs/surface-design.md §6.2's Table J amendment) — those ARE
// the held values, with nothing further to widen — so lengthBound is NOT
// this reading for them: a circular arc's length is never an exact
// rational (internal/momentregion), so lengthBound is nonzero for essentially every
// circular edge regardless of how exactly its Center, Axis and Radius are
// known, and reading it here would refuse the commonest thing this gate
// exists to admit (a lone closed circular rim). lengthUnbounded is the one
// genuine signal available instead: this evaluator's own admission that
// even a bound on the edge's length could not be proven, which only a
// construction this gate has no other reason to trust would raise.
func edgeGeometryExact(e *topo.Edge) bool {
	switch e.Curve.(type) {
	case surfacegeom.Circle3, surfacegeom.Arc3:
		return !e.LengthUnbounded
	default:
		return e.LengthBound == 0 && !e.LengthUnbounded
	}
}

// curveCarrier returns a curved edge's own carrier plane — Center and
// Axis — and reports true for Circle3 and Arc3. Every other Curve variant
// (Line3, and the free-form kinds no free rim edge of a body carries as a
// whole edge, docs/surface-design.md §6.2's Table J amendment) reports false:
// it has no carrier plane narrower than the chain's own.
func curveCarrier(e *topo.Edge) (center, axis r3.Vec, curved bool) {
	switch c := e.Curve.(type) {
	case surfacegeom.Circle3:
		return c.Center, c.Axis, true
	case surfacegeom.Arc3:
		return c.Center, c.Axis, true
	default:
		return r3.Vec{}, r3.Vec{}, false
	}
}
