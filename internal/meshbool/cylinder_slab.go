package meshbool

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// SlabCylinder is the exact cylinder a facet's true piece lies on: every point
// of the axis line Origin + s·Axis has distance Radius (mm) from the true
// piece's points, with Axis any nonzero direction. Delta is the proven
// displacement of the true wall from it: every true point lies within Delta of
// the cylinder.
type SlabCylinder struct {
	Origin, Axis r3.Vec
	Radius       float64
	Delta        float64
}

// SetCylinderSlabBounds lowers SlabBound for the facets whose true piece lies
// on an exact cylinder (agent-docs/faceted-source-retention-design.md §11,
// Claim C). cylOf returns facet i's cylinder and true for such a facet. Every
// other facet keeps its SlabBound. SetVertexBounds must have run.
//
// A facet's FacetBound carries the largest bound of its corners, which beside
// an earlier rim is that rim's bound, while the cylinder still lies within its
// chord's sagitta of the facet's plane across the whole facet. RimBound reads
// the slab, so the rim a later Boolean makes on such a facet reads the
// cylinder's own departure and not the earlier rim's bound.
func (bm *BoolMesh) SetCylinderSlabBounds(cylOf func(facet int) (SlabCylinder, bool)) {
	for i := range bm.Tris {
		cyl, ok := cylOf(i)
		if !ok {
			continue
		}
		bm.lowerCylinderSlab(i, cyl)
	}
}

// cylinderSlabWork is the flat work a bounded replay charges for one facet's
// cylinder slab. CylinderSlab forms a fixed number of exact dot and cross
// products of the facet's corners and the cylinder's held floats, then a fixed
// number of float interval operations, so its cost is a constant multiple of
// those floats' size.
const cylinderSlabWork = 64

// SetCylinderSlabBoundsReplay is SetCylinderSlabBounds on a bounded replay: it
// charges one unit per facet and cylinderSlabWork per facet cylOf names.
func (bm *BoolMesh) SetCylinderSlabBoundsReplay(ctx context.Context, cylOf func(facet int) (SlabCylinder, bool),
	budget *proofbound.ReplayBudget) error {
	for i := range bm.Tris {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		cyl, ok := cylOf(i)
		if !ok {
			continue
		}
		if err := budget.SpendWork(ctx, cylinderSlabWork); err != nil {
			return err
		}
		bm.lowerCylinderSlab(i, cyl)
	}
	return nil
}

// lowerCylinderSlab lowers facet i's SlabBound to its cylinder slab when that
// is smaller.
func (bm *BoolMesh) lowerCylinderSlab(i int, cyl SlabCylinder) {
	tri := bm.Tris[i]
	slab, ok := CylinderSlab([3]r3.Vec{bm.Verts[tri[0]], bm.Verts[tri[1]], bm.Verts[tri[2]]}, bm.FacetBound[i], cyl)
	if ok && slab < bm.SlabBound[i] {
		bm.SlabBound[i] = slab
	}
}

// CylinderSlab bounds how far the true piece of the facet with the given
// corners lies from the facet's own plane P, given that the piece lies within
// cyl.Delta of the cylinder cyl and within beta of the facet. It reports false
// when it proves no bound.
//
// Claim C. Let x be a point of the cylinder within b = beta + cyl.Delta of a
// point y of the facet. Write D for the axis direction, O for the axis origin,
// N for the facet normal (c1 − c0) × (c2 − c0), σ(p) = D·(p − O), and split
// x − O = (σ(x)/D·D)·D + z with z ⊥ D, |z| = R. Let ν be the unit vector along
// the part of N normal to D, M = |N × D|/|D| its length, and τ the unit vector
// along D × N, so ν, τ is an orthonormal frame of the plane normal to D. Then
//
//	N·(x − c0) = (N·D/D·D)·σ(x) + M·(ν·z) − N·(c0 − O).
//
// σ is affine, so σ(x) lies within b·|D| of the corners' range. The projection
// normal to D shortens distances, so τ·z lies within b of the corners' range of
// τ·(p − O). Because y lies on P, M·(ν·y_⊥) = N·(c0 − O) − (N·D/D·D)·σ(y); when
// its smallest value over the corners, divided by M, exceeds b, every such z
// has ν·z > 0, so ν·z = √(R² − (τ·z)²), which is bounded by the τ range. The
// distance from x to P is |N·(x − c0)|/|N|. A true point within cyl.Delta of
// x is within cyl.Delta more.
//
// Every quantity is formed from the held floats exactly, rounded outward to a
// float interval, and combined in intervals rounded outward, so the bound is
// never below the true supremum.
func CylinderSlab(corners [3]r3.Vec, beta float64, cyl SlabCylinder) (float64, bool) {
	if !(beta >= 0) || !(cyl.Delta >= 0) || !(cyl.Radius > 0) || math.IsInf(cyl.Radius, 0) {
		return 0, false
	}
	b := proofbound.AbsSumUpper(beta, cyl.Delta)
	if proofbound.IsNonFinite(b) {
		return 0, false
	}
	o := proof.DyVec(cyl.Origin)
	d := proof.DyVec(cyl.Axis)
	c := [3]proof.DyV3{proof.DyVec(corners[0]), proof.DyVec(corners[1]), proof.DyVec(corners[2])}
	n := proof.DvCross(proof.DvSub(c[1], c[0]), proof.DvSub(c[2], c[0]))
	nn := proof.DvDot(n, n)
	dd := proof.DvDot(d, d)
	nxd := proof.DvCross(n, d)
	x2 := proof.DvDot(nxd, nxd)
	if nn.IsZero() || dd.IsZero() || x2.IsZero() {
		return 0, false
	}
	q0 := proof.DvSub(c[0], o)
	a := proof.DvDot(n, q0)
	bs := proof.DvDot(n, d)
	// (a·D·D − bs·σ(c0))/D·D is the part of N normal to D dotted with c0's
	// offset from the axis. Orient N so that it is positive, which puts the
	// axis on N's negative side; the distance is symmetric in that choice.
	if proof.DySubScalar(proof.DyMul(a, dd), proof.DyMul(bs, proof.DvDot(d, q0))).Sign() < 0 {
		a, bs = proof.DyNeg(a), proof.DyNeg(bs)
	}
	lenD := sqrtIval(dyIval(dd))
	lenX := sqrtIval(dyIval(x2))
	m := lenX.div(lenD)
	k := ratioIval(bs, dd) // N·D / D·D
	var sig, tau ival
	for i, ci := range c {
		q := proof.DvSub(ci, o)
		si := dyIval(proof.DvDot(d, q))
		ti := dyIval(proof.DvDot(nxd, q)).div(lenX)
		if i == 0 {
			sig, tau = si, ti
			continue
		}
		sig, tau = sig.hull(si), tau.hull(ti)
	}
	// The smallest M·(ν·y_⊥) over the facet is at a corner, where σ is extreme.
	ay := dyIval(a).sub(k.mul(sig))
	if !(ay.lo > 0) {
		return 0, false
	}
	if low := ay.div(m).lo; !(low > b) {
		return 0, false
	}
	bLen := ival{b, b}.mul(lenD)
	sigX := ival{sig.lo, sig.hi}.widen(bLen.hi)
	tauX := ival{tau.lo, tau.hi}.widen(b)
	r := ival{cyl.Radius, cyl.Radius}
	near, far := tauX.absRange()
	vHi := r.mul(r).sub(near.mul(near))
	if !(vHi.hi > 0) {
		return 0, false
	}
	vLo := r.mul(r).sub(far.mul(far))
	v := ival{lo: 0, hi: sqrtIval(vHi).hi}
	if vLo.lo > 0 {
		v.lo = sqrtIval(vLo).lo
	}
	f := k.mul(sigX).add(m.mul(v)).sub(dyIval(a))
	top := math.Max(math.Abs(f.lo), math.Abs(f.hi))
	slab := proofbound.DivUpper(top, proof.DySqrtDown(nn))
	slab = proofbound.AbsSumUpper(slab, cyl.Delta)
	if proofbound.IsNonFinite(slab) {
		return 0, false
	}
	return slab, true
}

// ival is a closed float interval whose every operation rounds outward, so it
// always encloses the exact result of the same operation on enclosed values.
type ival struct{ lo, hi float64 }

func down(f float64) float64 { return math.Nextafter(f, math.Inf(-1)) }
func up(f float64) float64   { return math.Nextafter(f, math.Inf(1)) }

// dyIval encloses an exact dyadic.
func dyIval(x proof.Dyadic) ival { return ival{proof.DyFloatDown(x), proof.DyFloatUp(x)} }

// ratioIval encloses the exact quotient n/d, d nonzero.
func ratioIval(n, d proof.Dyadic) ival {
	return ival{proof.DyQuoFloatDown(n, d), proof.DyQuoFloatUp(n, d)}
}

// sqrtIval encloses the square root of the non-negative part of x.
func sqrtIval(x ival) ival {
	lo := 0.0
	if x.lo > 0 {
		lo = math.Max(0, down(math.Sqrt(x.lo)))
	}
	return ival{lo, up(math.Sqrt(math.Max(0, x.hi)))}
}

func (x ival) add(y ival) ival { return ival{down(x.lo + y.lo), up(x.hi + y.hi)} }
func (x ival) sub(y ival) ival { return ival{down(x.lo - y.hi), up(x.hi - y.lo)} }

func (x ival) mul(y ival) ival {
	p := [4]float64{x.lo * y.lo, x.lo * y.hi, x.hi * y.lo, x.hi * y.hi}
	lo, hi := p[0], p[0]
	for _, v := range p[1:] {
		lo, hi = math.Min(lo, v), math.Max(hi, v)
	}
	return ival{down(lo), up(hi)}
}

// div divides by y, which must not contain zero; a y that does yields an
// interval that encloses nothing useful (±Inf or NaN), which every caller
// refuses.
func (x ival) div(y ival) ival {
	if y.lo <= 0 && y.hi >= 0 {
		return ival{math.Inf(-1), math.Inf(1)}
	}
	p := [4]float64{x.lo / y.lo, x.lo / y.hi, x.hi / y.lo, x.hi / y.hi}
	lo, hi := p[0], p[0]
	for _, v := range p[1:] {
		lo, hi = math.Min(lo, v), math.Max(hi, v)
	}
	return ival{down(lo), up(hi)}
}

func (x ival) hull(y ival) ival { return ival{math.Min(x.lo, y.lo), math.Max(x.hi, y.hi)} }

// widen grows x by w on each side.
func (x ival) widen(w float64) ival { return ival{down(x.lo - w), up(x.hi + w)} }

// absRange returns intervals enclosing the smallest and the largest |t| over t
// in x: the first's lower end and the second's upper end are the ones a
// caller reads.
func (x ival) absRange() (ival, ival) {
	far := math.Max(math.Abs(x.lo), math.Abs(x.hi))
	if x.lo <= 0 && x.hi >= 0 {
		return ival{0, 0}, ival{far, far}
	}
	near := math.Min(math.Abs(x.lo), math.Abs(x.hi))
	return ival{near, near}, ival{far, far}
}
