package patchchain

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// AreaVertex is one built rim corner and its proven position bound.
type AreaVertex struct {
	Position r3.Vec
	Bound    float64
}

// AreaEdge is one built rim edge in the patch chain's walk order. Its curve
// and length readings are taken from the built edge, not the source record.
type AreaEdge struct {
	Curve           surfacegeom.Curve
	Start, End      AreaVertex
	Forward         bool
	Length          float64
	LengthBound     float64
	LengthUnbounded bool
	CurveBound      float64
	CurveBounded    bool
}

// PolygonAreaBound bounds the difference between area and the denoted
// polygon's area. For held corners p_i and displacements |d_i| ≤ β_i, the
// denoted vector area differs by at most
// ½Σ β_i|p_(i+1)−p_(i−1)| + ½Σ β_iβ_(i+1).
func PolygonAreaBound(area float64, edges []AreaEdge) (float64, bool) {
	n := len(edges)
	if n < 3 {
		return 0, false
	}
	corners := make([]proofarith.DyV3, n)
	beta := make([]float64, n)
	for i, e := range edges {
		if _, ok := e.Curve.(surfacegeom.Line3); !ok {
			return 0, false
		}
		v := e.Start
		if !e.Forward {
			v = e.End
		}
		if !proofbound.FiniteVec(v.Position) || proofbound.IsNonFinite(v.Bound) {
			return 0, false
		}
		corners[i] = proofarith.DyVec(v.Position)
		beta[i] = v.Bound
	}
	var twice proofarith.DyV3
	for i := range n {
		twice = proofarith.DvAdd(twice, proofarith.DvCross(corners[i], corners[(i+1)%n]))
	}
	norm2 := proofarith.DvDot(twice, twice)
	lo := proofarith.DySqrtDown(norm2) / 2
	hi := proofarith.DySqrtUp(norm2) / 2
	held, okHeld := proofarith.DyOf(area)
	dLo, okLo := proofarith.DyOf(lo)
	dHi, okHi := proofarith.DyOf(hi)
	if !okHeld || !okLo || !okHi {
		return 0, false
	}
	gap := max(
		proofarith.DyFloatUp(proofarith.DyAbs(proofarith.DySubScalar(held, dLo))),
		proofarith.DyFloatUp(proofarith.DyAbs(proofarith.DySubScalar(held, dHi))),
	)
	perturb := 0.0
	for i := range n {
		if beta[i] == 0 {
			continue
		}
		span := proofarith.DvSub(corners[(i+1)%n], corners[(i+n-1)%n])
		perturb = proofbound.AbsSumUpper(perturb,
			proofbound.ProductUpper(beta[i], proofarith.DySqrtUp(proofarith.DvDot(span, span))),
			proofbound.ProductUpper(beta[i], beta[(i+1)%n]))
	}
	if perturb > 0 {
		gap = proofbound.AbsSumUpper(gap, proofbound.ProductUpper(0.5, perturb))
	}
	if proofbound.IsNonFinite(gap) {
		return 0, false
	}
	return gap, true
}

// CurvedAreaCharge bounds how far the area of the planar region a rim
// with circular edges denotes sits from the area of the loop segs records
// in frame's plane coordinates, carried to world by frame's map l (stretch
// its orthonormality defect): the area the integrals compute, up to l's own
// stretch, which the caller charges.
//
// Both loops are closed planar curves, so each area is the length of its
// vector area ½∮ x × dx. Given a continuous correspondence x_d(t) ↔ x_l(t)
// with |x_d − x_l| ≤ κ, ∮ x_d × dx_d − ∮ x_l × dx_l = 2∮ d × dx_l + ∮ d × dd
// for d = x_d − x_l, so the areas differ by at most
// Σ κ_e·(1.5·len_l,e + 0.5·len_d,e) over the edges. Per edge:
//
//   - a line corresponds linearly between its two ends, so κ_e is the larger
//     end gap: the held vertex's own bound plus its exact distance from the
//     lifted record point;
//   - a circular edge corresponds through its held circle: the denoted curve
//     projects radially onto it within the edge's curveBound, the lifted
//     record circle within its centre's exact gap plus
//     massmoment.CircleImageGap's terms, and matching the ends adds twice
//     the larger end gap, which includes how far an ArcSeg's recorded end
//     sits off its own circle. A whole circle has no ends to match.
//
// len_d,e is the edge's own published length and bound, len_l,e the lifted
// record curve's, a whole turn's for a circular one. It answers false when
// any edge has no curve bound, no finite length or a gap not below half its
// radius.
func CurvedAreaCharge(frame r3.Frame, l [3][3]*big.Rat, stretch float64, segs []sectionrecord.CurveSegment, edges []AreaEdge) (float64, bool) {
	if len(segs) != len(edges) {
		return 0, false
	}
	o, fu, fv := frame.Origin(), frame.U(), frame.V()
	for _, v := range [...]r3.Vec{o, fu, fv} {
		if !proofbound.FiniteVec(v) {
			return 0, false
		}
	}
	do, du, dv := proofarith.DyVec(o), proofarith.DyVec(fu), proofarith.DyVec(fv)
	lift := func(p sectionrecord.Point2) (proofarith.DyV3, bool) {
		pu, okU := proofarith.DyOf(p.U)
		pv, okV := proofarith.DyOf(p.V)
		if !okU || !okV {
			return proofarith.DyV3{}, false
		}
		var out proofarith.DyV3
		for i := range 3 {
			out[i] = proofarith.DyAdd(do[i], proofarith.DyAdd(proofarith.DyMul(du[i], pu), proofarith.DyMul(dv[i], pv)))
		}
		return out, true
	}
	dist := func(a proofarith.DyV3, b r3.Vec) (float64, bool) {
		if !proofbound.FiniteVec(b) {
			return 0, false
		}
		d := proofarith.DvSub(a, proofarith.DyVec(b))
		return proofarith.DySqrtUp(proofarith.DvDot(d, d)), true
	}
	planeLen := func(a, b sectionrecord.Point2) float64 {
		da, _ := proofarith.DyOf(a.U)
		db, _ := proofarith.DyOf(b.U)
		ea, _ := proofarith.DyOf(a.V)
		eb, _ := proofarith.DyOf(b.V)
		x, y := proofarith.DySubScalar(db, da), proofarith.DySubScalar(eb, ea)
		return proofarith.DySqrtUp(proofarith.DyAdd(proofarith.DyMul(x, x), proofarith.DyMul(y, y)))
	}
	onePlus := proofbound.AbsSumUpper(1, stretch)
	twoPi := proofbound.TwoPiUpper()
	total := 0.0
	for i, ne := range edges {
		if ne.LengthUnbounded || proofbound.IsNonFinite(ne.Length) || proofbound.IsNonFinite(ne.LengthBound) {
			return 0, false
		}
		vs, ve := ne.Start, ne.End
		if !ne.Forward {
			vs, ve = ve, vs
		}
		endGap := func(v AreaVertex, p sectionrecord.Point2) (float64, bool) {
			at, ok := lift(p)
			if !ok || proofbound.IsNonFinite(v.Bound) {
				return 0, false
			}
			g, ok := dist(at, v.Position)
			return proofbound.AbsSumUpper(v.Bound, g), ok
		}
		denotedLen := proofbound.AbsSumUpper(ne.Length, ne.LengthBound)
		var kappa, liftedLen float64
		switch seg := segs[i].(type) {
		case sectionrecord.LineSeg:
			gs, ok1 := endGap(vs, seg.Start)
			ge, ok2 := endGap(ve, seg.End)
			if !ok1 || !ok2 {
				return 0, false
			}
			kappa = max(gs, ge)
			liftedLen = proofbound.ProductUpper(onePlus, planeLen(seg.Start, seg.End))
		case sectionrecord.CircleSeg, sectionrecord.ArcSeg:
			var center sectionrecord.Point2
			var radius, radiusGap, ends float64
			heldCenter, heldAxis, heldRadius, ok := surfacegeom.CircleOf(ne.Curve)
			if !ok || !ne.CurveBounded {
				return 0, false
			}
			switch c := seg.(type) {
			case sectionrecord.CircleSeg:
				center, radius = c.Center, c.Radius.Base()
				radiusGap = proofbound.RatFloatUp(new(big.Rat).Abs(new(big.Rat).Sub(new(big.Rat).SetFloat64(radius), new(big.Rat).SetFloat64(heldRadius))))
			case sectionrecord.ArcSeg:
				center = c.Center
				r2 := planeLen(c.Center, c.Start)
				radius = r2
				radiusGap = proofbound.AbsSumUpper(math.Abs(r2-heldRadius), proofbound.ProductUpper(2, math.Abs(r2)*0x1p-52))
				// The recorded end's own distance off the circle its start
				// fixes is a jump the lifted loop makes there.
				offCircle := proofbound.AbsSumUpper(math.Abs(planeLen(c.Center, c.End)-r2), proofbound.ProductUpper(4, math.Abs(r2)*0x1p-52))
				gs, ok1 := endGap(vs, c.Start)
				ge, ok2 := endGap(ve, c.End)
				if !ok1 || !ok2 {
					return 0, false
				}
				ends = proofbound.ProductUpper(2, proofbound.AbsSumUpper(max(gs, ge), proofbound.ProductUpper(onePlus, offCircle)))
			}
			at, ok := lift(center)
			if !ok {
				return 0, false
			}
			centerGap, ok := dist(at, heldCenter)
			if !ok {
				return 0, false
			}
			liftedGap := proofbound.AbsSumUpper(centerGap, massmoment.CircleImageGap(l, stretch, heldAxis, heldRadius, radiusGap))
			if !(proofbound.ProductUpper(2, liftedGap) < heldRadius) {
				return 0, false
			}
			kappa = proofbound.AbsSumUpper(ne.CurveBound, liftedGap, ends)
			liftedLen = proofbound.ProductUpper(onePlus, proofbound.ProductUpper(twoPi, proofbound.AbsSumUpper(radius, radiusGap)))
		default:
			return 0, false
		}
		edgeCharge := proofbound.ProductUpper(kappa, proofbound.AbsSumUpper(proofbound.ProductUpper(1.5, liftedLen), proofbound.ProductUpper(0.5, denotedLen)))
		total = proofbound.AbsSumUpper(total, edgeCharge)
	}
	if proofbound.IsNonFinite(total) {
		return 0, false
	}
	return total, true
}
