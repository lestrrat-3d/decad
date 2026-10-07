package revolvesurvey

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// AxisClassifier identifies meridian walks that lie on the revolve axis.
type AxisClassifier interface {
	IsAxis(survey2d.SegmentWalk) bool
}

// WallReading is the spanning-ball reading of a revolved body, computed in
// its meridian section: a full revolution's balls are the disks of the
// section mirrored across the axis (a ball's contacts with surfaces of
// revolution lie in the meridian plane through its center), and a partial
// sweep's are the plain section's disks constrained by the cap wedge —
// whose own two contacts span exactly when the sweep is within the
// allowance, the on-axis cap edge (dihedral = the sweep itself) included.
func WallReading(
	budget *proofbound.WorkBudget, loops [][]survey2d.SideWalk, classify AxisClassifier,
	full bool, phi0, phi1, angularDelta, alpha float64,
) (survey2d.WallReading, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return survey2d.WallReading{}, err
	}
	// The sweep, with the subtraction's OWN rounding plus the proven angular
	// displacement (docs/evaluator-design.md §6) as its bound — the two
	// recorded angles are held floats, not the angle the record denotes, so
	// the displacement between the two is part of the error too. A survey
	// bound may never be floored at some coarse magnitude ceiling the way a
	// mere size estimate can be: a bound wider than the value would refuse
	// every wedge candidate below.
	dphiBS := proofbound.BoundedSub(proofbound.ExactScalar(phi1), proofbound.ExactScalar(phi0))
	dphiBS.Bound = proofbound.AbsSumUpper(dphiBS.Bound, angularDelta)
	dphi := dphiBS.Value
	var elems, containOnly []survey2d.SurveyElem
	var verts [][2]float64
	pinch := false
	axisTouch := false
	for _, loop := range loops {
		n := len(loop)
		single := n == 1 && loop[0].Closed
		kinds := make([]bool, n)
		for i, w := range loop {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return survey2d.WallReading{}, err
			}
			kinds[i] = classify.IsAxis(w.SegmentWalk)
		}
		for i, w := range loop {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return survey2d.WallReading{}, err
			}
			if kinds[i] {
				axisTouch = true
				if !full {
					if el, ok := survey2d.WalkElem(w.SegmentWalk); ok {
						containOnly = append(containOnly, el)
					}
				}
				continue
			}
			el, ok := survey2d.WalkElem(w.SegmentWalk)
			if !ok {
				return survey2d.WallReading{}, nil
			}
			elems = append(elems, el)
			if full {
				elems = append(elems, survey2d.MirrorElem(el))
			}
			if single {
				continue
			}
			if w.StartV == 0 || w.EndV == 0 {
				axisTouch = true
			}
			verts = append(verts, [2]float64{w.StartU, w.StartV})
			if full && w.StartV > 0 {
				verts = append(verts, [2]float64{w.StartU, -w.StartV})
			}
			prev := loop[(i+n-1)%n]
			prevAxis := kinds[(i+n-1)%n]
			nextAxis := kinds[(i+1)%n]
			switch {
			case !prevAxis:
				if survey2d.JunctionPinch(prev.TanOutU, prev.TanOutV, w.TanInU, w.TanInV, alpha) {
					pinch = true
				}
			case full:
				// The walk continues as its own mirror image across the axis:
				// the dihedral there is the corner the symmetrized section
				// shows (a cone's apex reads its full apex angle).
				if survey2d.JunctionPinch(-w.TanInU, w.TanInV, w.TanInU, w.TanInV, alpha) {
					pinch = true
				}
			}
			if nextAxis && full {
				if survey2d.JunctionPinch(w.TanOutU, w.TanOutV, -w.TanOutU, w.TanOutV, alpha) {
					pinch = true
				}
			}
		}
	}
	wedgeS := proofbound.ExactScalar(0)
	wedgeSpans := false
	if !full {
		// A mid-sweep ball at meridian radius ρ clears each cap HALF-plane
		// by ρ·sin(dphi/2) only while dphi/2 ≤ 90°: past that (a reflex
		// sweep) the perpendicular foot leaves the half-plane, the nearest
		// cap point is its edge — the axis — and the clearance is ρ itself.
		// So the factor saturates at sin(π/2) and never shrinks again;
		// shrinking it past 90° would erase real walls
		// (TestWallReflexSweep pins the hand-checked case).
		//
		// The factor is a SINE, the one kernel input that is not an exact
		// leaf, so it is proven rather than trusted: survey2d.RadianTrigBounds encloses
		// sin of the held half-angle from the same rational bracket a Cone's
		// normal reads (never math.Sin's own accuracy), and the sweep's own
		// subtraction rounding rides in on top because θ ↦ sin(min(θ, π/2)) is
		// 1-Lipschitz — halving is exact, so half the subtraction's bound is
		// the whole displacement the half-angle can carry.
		sinBS, _ := survey2d.RadianTrigBounds(math.Min(dphi/2, math.Pi/2))
		wedgeS = proofbound.MeasuredScalar(sinBS.Value, proofbound.AbsSumUpper(sinBS.Bound, dphiBS.Bound/2))
		if proofbound.IsNonFinite(wedgeS.Bound) {
			// No proven enclosure of the cap half-angle's sine: every
			// wedge-derived candidate would publish an unusable interval, so
			// the survey is undecided rather than silently exact.
			return survey2d.WallReading{}, nil
		}
		if proofbound.AdmitAbove(wedgeS, 0) != proofbound.SurvAdmit {
			// The kernel reads a positive wedge sine as "this body has caps",
			// so the sine must be PROVEN positive before it is handed over: a
			// sweep whose half-angle cannot be told from zero would otherwise
			// either erase the caps or starve every candidate against them.
			// Undecided, which reads Suspect — never a silent pass.
			return survey2d.WallReading{}, nil
		}
		wedgeSpans = dphi <= alpha+survey2d.SurvAngTol
	}
	k, err := survey2d.NewWallKernelBudget(budget, elems, containOnly, verts, alpha, wedgeS, wedgeSpans, math.Inf(1))
	if err != nil {
		return survey2d.WallReading{}, err
	}
	out, err := k.RunBudget(budget)
	if err != nil {
		return survey2d.WallReading{}, err
	}
	if !out.Ok {
		return survey2d.WallReading{}, nil
	}
	// The arms reduce through the same §9.2 aggregate prismWall uses. Here the
	// only non-zero arm is the section's spanning diameter — the pinch and the
	// axis-meeting cap pair each contribute an exact zero, which no
	// non-negative interval can reach below — so the aggregate agrees with the
	// held comparison on every body; it is written this way because one
	// reduction owning the reading is what keeps the two wall arms from
	// drifting apart.
	zeroArm := pinch || (wedgeSpans && axisTouch)
	agg := survey2d.MinAggregate()
	if zeroArm {
		// A pinch, or a partial sweep whose caps meet along the axis at the
		// sweep angle itself: a dihedral within the allowance, ground to zero.
		agg.Take(0, 0)
	}
	if out.SubTolFar && !zeroArm {
		// A dropped off-junction sub-tolerance disk could be a real web
		// thinner than the kernel resolves: only an exact zero (which
		// nothing can undercut) still decides; any other reading — or an
		// absence — is undecided.
		return survey2d.WallReading{}, nil
	}
	if out.HasSpan {
		agg.Take(out.Span, out.SpanBound)
	}
	if agg.Empty() && !agg.Unbounded {
		return survey2d.WallReading{Ok: true}, nil
	}
	best, bestBound, ok := agg.Resolve()
	if !ok {
		return survey2d.WallReading{}, nil
	}
	return survey2d.WallReading{Reading: &best, Bound: bestBound, Ok: true}, nil
}

// RadiusAggregate is the tightest concave radius over a revolved body's
// faces, from the two principal directions of a surface of revolution: the
// meridian's own curvature (an arc walked against the orientation — a
// groove's tube), and the parallel circle's, whose radius is ρ/|n_ρ|
// wherever the meridian normal points toward the axis (a hole wall, a
// waist). Both are closed-form over each walk's extent, and both arms feed the
// same survey2d.ExtremeAggregate: the reading is the interval it proves over every
// candidate, never the figures of whichever arm held the smallest value.
func RadiusAggregate(loops [][]survey2d.SideWalk, classify AxisClassifier) survey2d.ExtremeAggregate {
	agg := survey2d.MinAggregate()
	for _, loop := range loops {
		for _, w := range loop {
			if classify.IsAxis(w.SegmentWalk) {
				continue
			}
			if !w.IsCircular() {
				// The walk tangent is NOT a recorded coordinate: it is the
				// difference of two of them, which the evaluator computed and
				// which extrude.go's lineWalkTangentBound proved a bound on.
				// Both the length below and the normal component it divides
				// take that bound — the numerator as much as the denominator,
				// since the same rounded difference stands in both. w.startV/
				// endV are the axis frame's own re-expression of a recorded
				// coordinate, not a recorded one themselves, so they take
				// axisFrame.walk's own startVBound/endVBound rather than reading
				// as exact leaves. Those bounds carry every mechanism that walk
				// names: the axis direction and anchor's own representation
				// error (axisInPlane's dUBound/dVBound/aUBound/aVBound), and the
				// magnitude the axis snap discarded when it assigned an endpoint
				// exactly zero. Both are zero only where the walk's own
				// arithmetic committed neither, which is what leaves an
				// axis-aligned frame's untouched end reading Exact.
				tanU := proofbound.MeasuredScalar(w.TanInU, w.TanInBound)
				tanV := proofbound.MeasuredScalar(w.TanInV, w.TanInBound)
				lBS := proofbound.BoundedNorm2(tanU, tanV)
				nrBS := proofbound.BoundedQuotient(-w.TanInU, w.TanInBound, lBS.Value, lBS.Bound)
				// survey2d.SurvAngTol is the line this survey DECLARES, not a
				// measurement: a wall within 1e-9 radians of parallel to the
				// axis has a parallel-circle radius of at least 1e9·ρ, which is
				// no concave feature. The reading is taken on the component's
				// own proven interval, and a straddle admits, because the
				// candidate it lets through is that same 1e9·ρ — its interval's
				// lower end stays within a relative 1e-7 of it, so it cannot
				// become the aggregate's minimum beside a candidate of ordinary
				// size, and its denominator's clearance at the threshold keeps
				// its bound finite rather than leaving the survey undecided.
				if proofbound.AdmitBelow(nrBS, -survey2d.SurvAngTol) != proofbound.SurvReject {
					// The nearer end of the wall is the INTERVAL minimum of the
					// two ends' proven radial coordinates (proofbound.BoundedMin), never
					// the interval of whichever held value compared smaller:
					// the two ends carry their own independent bounds, so the
					// end that holds larger can still be the truly nearer one.
					minSV := proofbound.BoundedMin(
						proofbound.MeasuredScalar(w.StartV, w.StartVBound),
						proofbound.MeasuredScalar(w.EndV, w.EndVBound),
					)
					vBS := proofbound.BoundedQuotient(minSV.Value, minSV.Bound, -nrBS.Value, nrBS.Bound)
					agg.Take(vBS.Value, vBS.Bound)
				}
				continue
			}
			sigma := 1.0
			if w.Th1 < w.Th0 {
				sigma = -1
			}
			if sigma < 0 {
				// The meridian's own radius, with the walk's own proven bound
				// on it (extrude.go's arcWalkRadiusBound): zero for a recorded
				// CircleSeg radius, positive for an ArcSeg's math.Hypot.
				agg.Take(w.Radius, w.RadiusBound)
			}
			lo, hi := math.Min(w.Th0, w.Th1), math.Max(w.Th0, w.Th1)
			cands := []float64{lo, hi}
			for _, star := range []float64{math.Pi / 2, -math.Pi / 2} {
				for kk := math.Floor((lo-star)/(2*math.Pi)) * 2 * math.Pi; star+kk <= hi+1e-12; kk += 2 * math.Pi {
					if th := star + kk; th >= lo-1e-12 {
						cands = append(cands, th)
					}
				}
			}
			for _, th := range cands {
				// The sine's own certified enclosure (survey2d.RadianTrigBounds), not
				// math.Sin's undocumented accuracy — the same rational
				// bracket a Cone's own normal reads (internal/surfacenormal/normal_bound.go).
				sinBS, _ := survey2d.RadianTrigBounds(th)
				nrBS := proofbound.BoundedMul(proofbound.ExactScalar(sigma), sinBS)
				// The same declared line, read the same way: a straddling
				// meridian normal admits its candidate rather than dropping it,
				// and that candidate's own size keeps it out of the minimum.
				if proofbound.AdmitBelow(nrBS, -survey2d.SurvAngTol) == proofbound.SurvReject {
					continue
				}
				// w.cV is the axis frame's own re-expression of the circular
				// walk's recorded center, so it takes cVBound beside it rather
				// than reading as an exact leaf — the same account startV/endV
				// take above.
				rhoBS := proofbound.BoundedAdd(proofbound.MeasuredScalar(w.CV, w.CVBound), proofbound.BoundedMul(proofbound.MeasuredScalar(w.Radius, w.RadiusBound), sinBS))
				negNrBS := proofbound.MeasuredScalar(-nrBS.Value, nrBS.Bound)
				resultBS := proofbound.BoundedQuotient(rhoBS.Value, rhoBS.Bound, negNrBS.Value, negNrBS.Bound)
				agg.Take(resultBS.Value, resultBS.Bound)
			}
		}
	}
	return agg
}
