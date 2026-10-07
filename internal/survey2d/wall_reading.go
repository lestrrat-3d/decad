package survey2d

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// WalkElem converts one boundary walk into a survey element. The material
// side is irrelevant to ray parity, so an arc's walk sense is not consulted.
// Free-form walks have no survey element, so false leaves callers undecided.
func WalkElem(w SegmentWalk) (SurveyElem, bool) {
	switch w.Kind {
	case WalkCircular:
		e, ok := ArcElem(w.CU, w.CV, w.Radius, w.Th0, w.Th1, w.Closed)
		if !ok {
			return e, false
		}
		// A walk's radius can be computed rather than recorded. Keep its bound.
		e.RrBound = w.RadiusBound
		return e, true
	case WalkLine:
		return LineElem(w.StartU, w.StartV, w.EndU, w.EndV)
	default:
		return SurveyElem{}, false
	}
}

// MirrorElem reflects an element across the axis and reverses its walk so
// material stays on the left. Reflection preserves the radius bound.
func MirrorElem(e SurveyElem) SurveyElem {
	if e.Kind == SurveyLine {
		m, _ := LineElem(e.Bx, -e.By, e.Ax, -e.Ay)
		return m
	}
	m, _ := ArcElem(e.Qx, -e.Qy, e.Rr, -e.Th1, -e.Th0, e.Closed)
	m.RrBound = e.RrBound
	return m
}

// JunctionPinch reports a material corner whose dihedral fits the allowance.
// Such a wedge pinches an allowance-qualified wall to exact zero.
func JunctionPinch(inU, inV, outU, outV, alpha float64) bool {
	cross := inU*outV - inV*outU
	dot := inU*outU + inV*outV
	turn := math.Atan2(cross, dot)
	delta := math.Pi - turn
	return delta <= alpha+SurvAngTol
}

// WallReading is a prism section's bounded minimum wall diameter. A nil
// Reading with Ok means the section has no allowance-qualified wall.
type WallReading struct {
	Reading *float64
	Bound   float64
	Ok      bool
}

// PrismHeight holds the prism's two axial levels and their displacement bounds.
type PrismHeight struct {
	Z0, Z1           float64
	Z0Delta, Z1Delta float64
}

// PrismWallReading measures a prism from neutral section walks and axial
// levels with proven displacement bounds. The budget must not be nil.
func PrismWallReading(
	budget *proofbound.WorkBudget, loops [][]SideWalk, height PrismHeight, alpha float64,
) (WallReading, error) {
	if err := WallBudgetErr(budget); err != nil {
		return WallReading{}, err
	}
	h := height.Z1 - height.Z0
	var elems []SurveyElem
	var verts [][2]float64
	pinch := false
	for _, loop := range loops {
		single := len(loop) == 1 && loop[0].Closed
		for i, w := range loop {
			if err := WallBudgetStep(budget); err != nil {
				return WallReading{}, err
			}
			el, ok := WalkElem(w.SegmentWalk)
			if !ok {
				return WallReading{}, nil
			}
			elems = append(elems, el)
			if single {
				continue
			}
			verts = append(verts, [2]float64{w.StartU, w.StartV})
			prev := loop[(i+len(loop)-1)%len(loop)]
			if JunctionPinch(prev.TanOutU, prev.TanOutV, w.TanInU, w.TanInV, alpha) {
				pinch = true
			}
		}
	}
	k, err := NewWallKernelBudget(budget, elems, nil, verts, alpha, proofbound.ExactScalar(0), false, h)
	if err != nil {
		return WallReading{}, err
	}
	out, err := k.RunBudget(budget)
	if err != nil {
		return WallReading{}, err
	}
	if !out.Ok {
		return WallReading{}, nil
	}
	// The three arms are candidates of ONE minimum, so they reduce through the
	// §9.2 aggregate rather than through a comparison of held values. Two of
	// them — the section's spanning diameter and the cap-to-cap height — are
	// independent quantities carrying independent bounds, and the smaller held
	// value is not always the smaller truth: whenever the height's own axial
	// displacement is wider than the gap between the two held numbers, a
	// winner-only reduction publishes the section's interval while the true
	// minimum is the height's, sitting below it. The aggregate reaches down to
	// whichever arm's interval reaches lowest.
	agg := MinAggregate()
	if pinch {
		agg.Take(0, 0)
	}
	if out.SubTolFar && !pinch {
		// Same rule as the revolve path: a dropped off-junction
		// sub-tolerance disk could be a real web thinner than the kernel
		// resolves — only an exact zero still decides.
		return WallReading{}, nil
	}
	if out.HasSpan {
		agg.Take(out.Span, out.SpanBound)
	}
	// The height arm's ADMISSION is a separate question from its value: the
	// cap-to-cap ball exists only where the section's own largest inscribed
	// disk reaches h/2. That reading is the kernel's inradius aggregate, so
	// the gate is taken on its proven interval — an interval that cannot be
	// separated from h/2 decides neither that the arm belongs (which would
	// publish a wall the body may not have) nor that it does not (which would
	// drop a wall the body may have), and leaves the survey undecided.
	//
	// Unlike the kernel's own candidate guards, this one really does have
	// decades of room, so the undecided branch is an edge and not the ordinary
	// case: it compares against k.tol, 1e-9 of the section's scale, while the
	// inradius aggregate's half-width is the largest empty disk's own
	// arithmetic error — ulp-scale, since that disk is pinned by recorded
	// coordinates rather than by an angle-limit division. The boundary case
	// the gate exists for is the cube, whose inradius equals h/2 exactly and
	// clears by the whole of k.tol.
	switch proofbound.AdmitAbove(proofbound.MeasuredScalar(out.Inradius, out.InradiusBound), h/2-k.Tol) {
	case proofbound.SurvAdmit:
		agg.Take(h, heightArmBound(height))
	case proofbound.SurvStraddle:
		return WallReading{}, nil
	}
	if agg.Empty() && !agg.Unbounded {
		return WallReading{Ok: true}, nil
	}
	best, bestBound, ok := agg.Resolve()
	if !ok {
		// A candidate this reading relied on could not be bounded: the answer
		// is not one this evaluator can stand behind (see runBudget's own doc
		// comment), so it is undecided rather than published with an unusable
		// bound.
		return WallReading{}, nil
	}
	return WallReading{Reading: &best, Bound: bestBound, Ok: true}, nil
}

// heightArmBound adds the two independent axial displacements rather than
// taking their maximum. It also includes the held subtraction's rounding
// against the exact rational difference.
func heightArmBound(height PrismHeight) float64 {
	z0R, z1R := proofarith.FloatRat(height.Z0), proofarith.FloatRat(height.Z1)
	if z0R == nil || z1R == nil {
		return math.Inf(1)
	}
	h := height.Z1 - height.Z0
	subErr := RatAbsDiff(new(big.Rat).Sub(z1R, z0R), h)
	return proofbound.AbsSumUpper(height.Z0Delta, height.Z1Delta, subErr)
}
