package meshbool

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// AxisRays is the deterministic retry list for the parity test: the six
// axis-aligned directions. axis names the swept coordinate, dir its sense,
// and (u, v) the two projection coordinates.
var AxisRays = [6]struct {
	Axis, U, V int
	Dir        int
}{
	{0, 1, 2, 1}, {0, 1, 2, -1},
	{1, 2, 0, 1}, {1, 2, 0, -1},
	{2, 0, 1, 1}, {2, 0, 1, -1},
}

// CoordOf reads one coordinate of a float vertex by axis index.
func CoordOf(v r3.Vec, axis int) float64 {
	switch axis {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

// RatCoordOf materialises one exact coordinate of p as a big.Rat, by axis
// index — the projection into Xp2's own (untouched) rational domain. This is
// the one place a homogeneous coordinate pays a normalisation to become a
// value; a sign-only reader wants XIntCoordOf instead.
func RatCoordOf(p proof.Xpt, axis int) *big.Rat {
	switch axis {
	case 0:
		return new(big.Rat).SetFrac(p.X, p.W)
	case 1:
		return new(big.Rat).SetFrac(p.Y, p.W)
	default:
		return new(big.Rat).SetFrac(p.Z, p.W)
	}
}

// XIntCoordOf reads one coordinate's raw homogeneous numerator by axis index,
// with no normalisation at all: since the denominator (p.w) is always
// positive, this integer's sign already IS the coordinate's sign, which is
// what every sign-only reader in this file actually wants.
func XIntCoordOf(p proof.Xpt, axis int) *big.Int {
	switch axis {
	case 0:
		return p.X
	case 1:
		return p.Y
	default:
		return p.Z
	}
}

// ParityMesh is one mesh's vertex-projection cache for the parity kernel: the
// mesh's own vertex and facet buffers, held by reference, plus one lazily
// filled projection slice per swept axis.
//
// A cache belongs to ONE prepared operand within ONE operation. Every path that
// holds one — KeepSide, ClassifyRegion, and the near-miss depth witness scan —
// runs serially on its caller's goroutine, so an entry is never read while
// another goroutine writes it. There is deliberately no global, no pool and no
// Document field: a cache outliving its operation would outlive the buffers it
// projects.
//
// An entry is IMMUTABLE once initialized. Cross2xSign and Cross2x only read an
// Xp2's rationals — neither ever uses one as an arithmetic destination — so a
// projection shared across every query of this mesh classifies exactly as the
// fresh per-facet copy it replaces.
type ParityMesh struct {
	Verts []r3.Vec
	Tris  [][3]int
	// projections[axis][vi] is vertex vi projected onto the plane a ray
	// sweeping axis leaves. Both rays of an axis carry the same (u, v) pair
	// (AxisRays), so one slot per axis serves both senses. A slot stays nil
	// until that axis is first swept, and within a slot a nil u marks an entry
	// still unfilled: a materialized projection's coordinates are always
	// non-nil rationals, an exact zero included, so the two states never alias.
	Projections [3][]Xp2
	// boxes[axis][ti] is facet ti's projected box on that same plane, filled
	// on the facet's first visit for that axis and immutable from there. A
	// slot stays nil until the axis is first swept, and ParityFacetBox.built
	// marks an individual entry filled.
	Boxes [3][]ParityFacetBox
	// unfiltered turns the projected-box rejection off, leaving every query to
	// the exact sign tests alone. It exists so a differential test can run the
	// same query both ways and require the same answer; nothing in production
	// sets it.
	Unfiltered bool
}

// ParityAreaErrCoef and ParityAreaFloor bound projectedFacetBox's own float
// evaluation of the projected area. The true forward error of a 2x2
// determinant over float64 inputs is a few units in the last place of its
// permanent, so the relative coefficient carries about three decades of
// margin — enough to cover the rounding of the permanent and the threshold
// themselves. The floor is one absolute term for the gradual-underflow crumbs
// a relative bound cannot speak for: each is at most 2⁻¹⁰⁷⁵ and there are a
// handful, so 2⁻¹⁰⁰⁰ dominates them together.
const (
	ParityAreaErrCoef = 1e-12
	ParityAreaFloor   = 0x1p-1000
)

// ParityFacetBox is one facet's projection onto the plane a ray sweeping some
// axis leaves, reduced to a coordinate box plus the one fact that makes the box
// usable as a rejection filter.
//
// The bounds are EXACT, not an outward enclosure. A parity mesh's vertices are
// float64 coordinates and a projection just selects two of them, so each bound
// IS one of the triangle's own coordinates with no rounding to widen. The query
// side is what rounds: Xp2 caches its rational coordinate's nearest float
// (NewXP2), and round-to-nearest is monotone — x ≤ y implies rn(x) ≤ rn(y) — so
// rn(q) strictly past a bound proves q strictly past it exactly. The converse
// never holds, which is why equality decides nothing and falls through to the
// exact predicate.
//
// nondegenerate records that the projected triangle's area is provably nonzero,
// and without it the box says nothing about the answer.
// MeshParityPreparedContext turns a strict separation into a `continue` only
// because the three edge signs of a nondegenerate triangle cannot all be
// non-positive — they sum to twice its signed area — so a point outside it
// always shows the loop both a negative and a positive sign. A projection that
// collapses to a segment sums to zero instead, and every point on that
// segment's LINE, however far outside this box, makes all three signs vanish
// and the ray AMBIGUOUS. Skipping such a facet would drop an ambiguity the
// reference path reports, so a facet whose projected area cannot be proven
// nonzero is never filtered.
type ParityFacetBox struct {
	MinU, MaxU, MinV, MaxV float64
	Nondegenerate          bool
	Built                  bool
}

// rejects reports whether a query projected to (fu, fv) provably lies strictly
// outside this facet's projection. The caller owes it a finite (fu, fv) —
// MeshParityPreparedContext checks the query's own floatFinite once per scan
// rather than once per facet.
func (b *ParityFacetBox) Rejects(fu, fv float64) bool {
	return b.Nondegenerate &&
		(fu < b.MinU || fu > b.MaxU || fv < b.MinV || fv > b.MaxV)
}

// buildFacetBox builds facet ti's box on the (u, v) plane a ray sweeping axis
// leaves. A non-finite coordinate leaves nondegenerate false: the box would not
// bound anything, and an unusable box costs only the rejections it does not
// make.
//
// The area is decided adaptively, the same discipline as OrientSign: a float
// determinant whose magnitude clears its own error bound proves the sign, and
// anything inside that bound falls back to the exact 2D cross over the cached
// projections. The exact leg is what a coarse tessellation needs — a thin
// facet's float determinant can sit inside the bound while its true area is
// plainly nonzero — and it is paid once per facet and axis, against the many
// queries that then read the answer.
func (pm *ParityMesh) BuildFacetBox(axis, u, v, ti int) ParityFacetBox {
	tri := pm.Tris[ti]
	a, b, c := pm.Verts[tri[0]], pm.Verts[tri[1]], pm.Verts[tri[2]]
	au, av := CoordOf(a, u), CoordOf(a, v)
	bu, bv := CoordOf(b, u), CoordOf(b, v)
	cu, cv := CoordOf(c, u), CoordOf(c, v)
	box := ParityFacetBox{
		MinU:  math.Min(au, math.Min(bu, cu)),
		MaxU:  math.Max(au, math.Max(bu, cu)),
		MinV:  math.Min(av, math.Min(bv, cv)),
		MaxV:  math.Max(av, math.Max(bv, cv)),
		Built: true,
	}
	if proofbound.IsNonFinite(box.MinU) || proofbound.IsNonFinite(box.MaxU) ||
		proofbound.IsNonFinite(box.MinV) || proofbound.IsNonFinite(box.MaxV) {
		return box
	}
	left, right := (bu-au)*(cv-av), (bv-av)*(cu-au)
	bound := ParityAreaErrCoef*(math.Abs(left)+math.Abs(right)) + ParityAreaFloor
	if det := left - right; det > bound || det < -bound {
		box.Nondegenerate = true
		return box
	}
	qa := pm.VertexProjection(axis, u, v, tri[0])
	qb := pm.VertexProjection(axis, u, v, tri[1])
	qc := pm.VertexProjection(axis, u, v, tri[2])
	box.Nondegenerate = Cross2xSign(qa, qb, qc) != 0
	return box
}

// facetBoxes returns the per-facet projected-box slice for one swept axis,
// allocating it on that axis's first sweep. Both rays of an axis project onto
// the same plane (AxisRays), so one slice serves both senses.
func (pm *ParityMesh) FacetBoxes(axis int) []ParityFacetBox {
	if pm.Boxes[axis] == nil {
		pm.Boxes[axis] = make([]ParityFacetBox, len(pm.Tris))
	}
	return pm.Boxes[axis]
}

// NewParityMesh prepares verts and tris for repeated parity queries. It stores
// the caller's buffers by reference and materializes no projection at all: an
// axis slice is allocated on that axis's first sweep, and a vertex's projection
// on its own first use.
func NewParityMesh(verts []r3.Vec, tris [][3]int) *ParityMesh {
	return &ParityMesh{Verts: verts, Tris: tris}
}

// vertexProjection returns vertex vi projected onto the (u, v) plane a ray
// sweeping axis leaves, constructing it on first use and caching it unchanged.
// The returned Xp2 is read-only: its rationals are the cache's own, so a caller
// must never make one the destination of an arithmetic operation.
func (pm *ParityMesh) VertexProjection(axis, u, v, vi int) Xp2 {
	slot := pm.Projections[axis]
	if slot == nil {
		slot = make([]Xp2, len(pm.Verts))
		pm.Projections[axis] = slot
	}
	if slot[vi].U == nil {
		vert := pm.Verts[vi]
		slot[vi] = NewXP2(polynomial.MustRatOf(CoordOf(vert, u)), polynomial.MustRatOf(CoordOf(vert, v)))
	}
	return slot[vi]
}

// meshParity reports, exactly, whether p lies inside the closed float-vertex
// mesh restricted to the given facet subset: the crossing parity of an
// axis-aligned ray. A ray the point's projection meets at a facet's projected
// boundary is ambiguous and the next axis is tried; a p exactly ON a facet is
// onBoundary. All six axes ambiguous is a genuine failure — never a guess.
//
// This is the raw-buffer entry point: it prepares a single-use projection cache
// and answers one query through it. A caller holding an operand across many
// queries wants MeshParityPreparedContext with that operand's own cache.
func MeshParityContext(ctx context.Context, p proof.Xpt, verts []r3.Vec, tris [][3]int, subset []int) (bool, bool, error) {
	return MeshParityPreparedContext(ctx, p, NewParityMesh(verts, tris), subset)
}

// MeshParityPreparedContext is MeshParityContext over a prepared mesh whose
// vertex projections persist between queries. The classification is identical:
// only where each projection's rationals come from changes, and the cache hands
// back the same value the per-facet construction built.
func MeshParityPreparedContext(ctx context.Context, p proof.Xpt, prepared *ParityMesh, subset []int) (bool, bool, error) {
	for _, ray := range AxisRays {
		crossings := 0
		ambiguous := false
		onBoundary := false
		// The query's projection depends only on the ray, so it is built once
		// per nonempty scan rather than once per facet, and read-only from
		// there — Cross2xSign and Cross2x never mutate an Xp2, so one shared
		// value classifies exactly as a per-facet copy did. Constructing it
		// inside the loop, after the periodic cancellation check, is what
		// keeps an empty subset paying nothing and a canceled context
		// returning before the first conversion.
		var pa Xp2
		// The projected-box rejection needs the query's own float coordinates,
		// so it can only be armed once pa exists, and its per-facet boxes are
		// allocated in the same place — an empty subset still pays nothing.
		var boxes []ParityFacetBox
		boxFilter := false
		for i, ti := range subset {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return false, false, err
				}
			}
			if i == 0 {
				pa = NewXP2(RatCoordOf(p, ray.U), RatCoordOf(p, ray.V))
				boxFilter = !prepared.Unfiltered && pa.FloatFinite
				if boxFilter {
					boxes = prepared.FacetBoxes(ray.Axis)
				}
			}
			if boxFilter {
				// A facet whose projection provably has area and provably does
				// not reach the query's projected coordinate classifies
				// STRICTLY OUTSIDE below, whichever way its three signs fall
				// (ParityFacetBox). Skipping it therefore removes three exact
				// sign tests and changes no answer, and because it is exactly
				// the facets that would `continue` that go, the subset's order
				// and the facet an ambiguity is first seen at are untouched.
				box := &boxes[ti]
				if !box.Built {
					*box = prepared.BuildFacetBox(ray.Axis, ray.U, ray.V, ti)
				}
				if box.Rejects(pa.Fu, pa.Fv) {
					continue
				}
			}
			tri := prepared.Tris[ti]
			qa := prepared.VertexProjection(ray.Axis, ray.U, ray.V, tri[0])
			qb := prepared.VertexProjection(ray.Axis, ray.U, ray.V, tri[1])
			qc := prepared.VertexProjection(ray.Axis, ray.U, ray.V, tri[2])
			s1 := Cross2xSign(qa, qb, pa)
			s2 := Cross2xSign(qb, qc, pa)
			s3 := Cross2xSign(qc, qa, pa)
			neg := s1 < 0 || s2 < 0 || s3 < 0
			pos := s1 > 0 || s2 > 0 || s3 > 0
			if neg && pos {
				continue // strictly outside the projection
			}
			if s1 == 0 || s2 == 0 || s3 == 0 {
				// On the projected boundary: the ray may graze an edge or a
				// vertex, and the count would be unreliable — try another axis.
				ambiguous = true
				break
			}
			// Strictly inside the projection: the projected area is nonzero,
			// so the plane normal's swept component cannot vanish.
			a, b, c := prepared.Verts[tri[0]], prepared.Verts[tri[1]], prepared.Verts[tri[2]]
			xa, xb, xc := proof.XptOf(a), proof.XptOf(b), proof.XptOf(c)
			n := proof.Xcross(proof.Xsub(xb, xa), proof.Xsub(xc, xa))
			nAxis := XIntCoordOf(n, ray.Axis)
			if nAxis.Sign() == 0 {
				ambiguous = true
				break
			}
			// t = tNum/nAxis decides the crossing; nAxis is already proven
			// nonzero above, so its sign alone tells the division's sign
			// without ever forming the quotient — only t's sign is read, and
			// proof.XdotNum's raw numerator carries that sign with no normalisation
			// anywhere in the chain (docs/evaluator-design.md §9's
			// reject-only discipline extends to never paying for a value
			// nothing but Sign() consumes).
			tNum := proof.XdotNum(proof.Xsub(xa, p), n)
			switch s := tNum.Sign() * nAxis.Sign() * ray.Dir; {
			case s > 0:
				crossings++
			case tNum.Sign() == 0:
				onBoundary = true
			}
			if onBoundary {
				break
			}
		}
		if onBoundary {
			return false, true, nil
		}
		if ambiguous {
			continue
		}
		return crossings%2 == 1, false, nil
	}
	return false, false, fmt.Errorf(`%w: every parity ray was ambiguous`, decaderr.ErrBooleanFailed)
}
