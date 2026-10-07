package stitchflux

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// PiScalar returns math.Pi as a proofbound.BoundedScalar. Go's math.Pi is the correctly
// rounded float64 nearest true pi, so its own representation error is at
// most one ulp — tiny, and nothing like proofbound.ConservativeValueError's structural
// "assume no cancellation at all" fallback, which is sized for a quantity
// this evaluator cannot otherwise bound and would overstate a well-known
// constant's own error by many orders of magnitude: wide enough, on a thin
// annulus, to starve stitchCurvedMass's own proofbound.BoundedQuotient calls of the
// clearance they need and refuse a perfectly sound thin fixture. Every
// multiply that follows charges its OWN rounding through proofbound.BoundedMul, so
// this is the one place pi's own representation error is spent.
func PiScalar() proofbound.BoundedScalar {
	ulp := math.Nextafter(math.Pi, math.Inf(1)) - math.Pi
	return proofbound.MeasuredScalar(math.Pi, ulp)
}

// CircleRadius reads a full circle's own radius as a proofbound.BoundedScalar,
// derived from the edge's already-proven length/lengthBound (circumference
// = 2·pi·R) rather than from Circle3.Radius/Cylinder.Radius directly.
//
// Circle3 and Cylinder carry Radius as a bare units.Value with no bound
// field of its own, and this evaluator's own construction does not make
// that value exact in general: revolve_axis.go's axisFrame.walk re-expresses
// a recorded profile point in axis coordinates through toAxisRhoBound, which
// folds in the resolved axis's own anchor and direction bounds
// (aUBound/aVBound/dUBound/dVBound) — exactly zero only when the axis is
// coordinate-aligned and passes through the origin (every product is by 0 or
// 1 and nothing rounds), and genuinely positive for any other axis. A
// revolve wall's or junction circle's radius (j.rho in revolve_build.go) is
// exactly this re-expressed value, so a face this arm admits can carry a
// radius this evaluator has already proven imperfect — the zero-normalBound
// gate says the face's geometry IS its tag (Plane/Cylinder, not some
// unpublished ruled approximation), never that every field of that tag is
// itself exact. Deriving the bound from the edge's own length sidesteps the
// need to re-derive axisFrame's own bound composition here: Edge.Length()'s
// public contract already states a sound enclosure of the true circumference
// for ANY Circle3 edge, from whichever builder made it, and R = length/2·pi
// inverts that same closed form.
func CircleRadius(length, lengthBound float64, lengthUnbounded bool) (proofbound.BoundedScalar, error) {
	if lengthUnbounded {
		return proofbound.BoundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a circle whose circumference this evaluator can bound (docs/surface-design.md Table R row R8)`,
			decaderr.ErrUnsupported,
		)
	}
	twoPi := proofbound.BoundedMul(proofbound.MeasuredScalar(2, 0), PiScalar())
	return proofbound.BoundedQuotient(length, lengthBound, twoPi.Value, twoPi.Bound), nil
}

// Dot returns a·b as a proofbound.BoundedScalar, charging every multiply and add
// this dot product's own arithmetic commits (proofbound.BoundedMul/proofbound.BoundedAdd) —
// never the operands' own uncertainty, which every caller here supplies as
// zero: every vector Dot is asked about (a placed face's Frame
// origin/axis/normal, a placed Circle3's own center) is treated the same way
// the rest of this evaluator treats a placed coordinate outside a delta > 0
// placement term — authoritative for this file's own arithmetic, with the
// placement's own rounding charged separately, once, through
// proofbound.SweptVolumeAllow/proofbound.SweptMomentAllow.
func Dot(a, b r3.Vec) proofbound.BoundedScalar {
	x := proofbound.BoundedMul(proofbound.MeasuredScalar(a.X, 0), proofbound.MeasuredScalar(b.X, 0))
	y := proofbound.BoundedMul(proofbound.MeasuredScalar(a.Y, 0), proofbound.MeasuredScalar(b.Y, 0))
	z := proofbound.BoundedMul(proofbound.MeasuredScalar(a.Z, 0), proofbound.MeasuredScalar(b.Z, 0))
	return proofbound.BoundedAdd(proofbound.BoundedAdd(x, y), z)
}

// ConeApex derives a Cone's apex as three boundedScalars, one per world
// coordinate. Radius and HalfAngle are read exactly as topology.go's own
// Cone doc states them: Radius is the cone's radius AT Origin (zero when
// Origin is already the apex), and the wall grows along Axis by HalfAngle
// from there. The general formula is Origin − Axis·(Radius/tan(HalfAngle)),
// but this function NEVER evaluates that division: it admits Radius == 0
// only, where the formula collapses to apex = Origin, Exact, independent of
// tan(HalfAngle)'s own value. Every construction site in this evaluator
// (revolve_build.go's wallCone, capblend_geom.go's coneSurface) sets Radius
// literally to 0 — Origin already IS the apex — so this restriction costs
// no reachable admitted face; the refusal it adds for a nonzero Radius is
// exercised at a hand-built internal fixture
// (TestStitchConeApexRefusesNonzeroRadius, stitch_internal_test.go), since
// no construction site here ever sets Radius otherwise.
//
// WHY THE GENERAL DIVISION IS NOT SOUNDLY BOUNDABLE HERE, so a future
// change does not silently reintroduce it. tan(HalfAngle) would need its
// own rounding charged before Radius/tan(HalfAngle) could be trusted, and
// this codebase has no tool for that: proofbound.AnalyticRoundBound's own doc comment
// states "Go deliberately gives Sin, Cos, Atan2 and Hypot no public ulp
// contract, so a result computed through them never trusts this helper's
// roundoff budget on its own" — the same posture capblend_moments.go and
// internal/survey2d/wall_kernel.go state independently, and proofbound.BoundedSqrt honors even for
// math.Sqrt, which IEEE 754 DOES guarantee correctly rounded. The
// natural-looking fix — compose tan from boundedSin/proofbound.BoundedCos and run it
// through proofbound.BoundedQuotient — was tried and fails outright:
// proofbound.ConservativeValueError's own structural bound on a Sin/Cos result
// (|value|+1) is wider than either function's own value, so it fails
// proofbound.BoundedQuotient's clearance check for EVERY angle, not only the
// degenerate ones, refusing the whole arm rather than only the cases a
// tangent gate should catch. Charging only "a few ulps" of math.Tan's own
// result instead would be a NEW error model this codebase does not have
// and, per the citations above, would contradict its own repeated stance —
// not a reuse of an existing one. Refusing the one construction that would
// need it is the reject-only alternative CLAUDE.md's own rule favors over
// carrying an unproven bound.
//
// The tan(HalfAngle) call below still runs, but only to validate the angle
// is non-degenerate, never to compute anything: a HalfAngle of 0 gives an
// exactly-zero tangent, a degenerate needle no reachable wallCone ever
// carries (revolve_axis.go's own analytic-walk requirement keeps HalfAngle
// finite and strictly between 0 and π/2 — docs/surface-design.md's own
// record of that requirement). The `proofbound.IsNonFinite(tanValue)` half of this
// check is a defensive backstop, not independently exercised: a NaN or Inf
// HalfAngle is refused one layer up, by units.Value.In's own ErrNotFinite,
// before this function's own tan(HalfAngle) call ever runs, and a
// HalfAngle near π/2 does not make math.Tan return an actual ±Inf for any
// finite input (π/2 itself has no exact float64 representation) —
// TestStitchConeHalfAngleRefusesDegenerateTangent
// (stitch_internal_test.go) shows the exactly-zero case failing red and
// records why the non-finite half of the guard is not similarly shown.
func ConeApex(halfAngle, radius units.Value, origin r3.Vec) (proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, error) {
	half, herr := halfAngle.In(units.Radian)
	if herr != nil {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(`decad: a cone's half angle is not an angle: %w`, herr)
	}
	// tan(HalfAngle) is read through a single math.Tan call purely to
	// validate the angle is non-degenerate — a HalfAngle of 0 gives an
	// exactly-zero tangent, no real wallCone geometry at all. Its VALUE is
	// never used to compute anything: this file has no sound way to bound
	// tan(HalfAngle)'s own rounding (this function's own doc comment states
	// why), so the apex formula below never divides by it. That is also why
	// this check alone cannot admit a nonzero Radius — see the gate below.
	tanValue := math.Tan(half)
	if !(tanValue > 0) || proofbound.IsNonFinite(tanValue) {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Cone whose half-angle has a positive, finite tangent (docs/surface-design.md Table R row R8)`,
			decaderr.ErrUnsupported,
		)
	}
	radiusValue, rerr := radius.In(units.Millimeter)
	if rerr != nil {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(`decad: a cone's radius is not a length: %w`, rerr)
	}
	if radiusValue != 0 {
		// The general apex formula is Origin − Axis·(Radius/tan(HalfAngle)),
		// and CHARGING that division soundly needs a proven bound on
		// tan(HalfAngle)'s own rounding — this function's own doc comment
		// explains why no such bound exists in this codebase: Go gives
		// Sin/Cos/Atan2/Hypot (and so Tan, composed from them) no public
		// ulp contract, and every other place this evaluator reads one of
		// those functions falls back to proofbound.ConservativeValueError's
		// deliberately wide structural bound rather than assume tighter
		// accuracy — which, as CircleRadius's own history in this
		// file already showed, fails proofbound.BoundedQuotient's clearance check
		// outright rather than merely widen the result. Composing tan from
		// boundedSin/proofbound.BoundedCos and running it through proofbound.BoundedQuotient hits
		// exactly that failure. So a nonzero Radius refuses rather than
		// publish an apex whose own bound this evaluator cannot prove:
		// Radius is EXACTLY zero at every construction site in this
		// evaluator (revolve_build.go's wallCone, capblend_geom.go's
		// coneSurface — Origin is already the apex), so refusing what
		// nothing builds costs no reachable fixture
		// (TestStitchConeApexRefusesNonzeroRadius, stitch_internal_test.go).
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Cone whose Origin is already its own apex (Radius exactly 0); this evaluator has no sound bound for tan(HalfAngle)'s own rounding, which a nonzero Radius would need to divide by (docs/surface-design.md Table R row R8)`,
			decaderr.ErrUnsupported,
		)
	}
	// Radius is EXACTLY zero (checked above), so the general formula's
	// division by tan(HalfAngle) is skipped entirely rather than computed
	// and discarded: Origin is already the apex, Exact, with no dependency
	// on tan(HalfAngle)'s own accuracy at all.
	apexX := proofbound.MeasuredScalar(origin.X, 0)
	apexY := proofbound.MeasuredScalar(origin.Y, 0)
	apexZ := proofbound.MeasuredScalar(origin.Z, 0)
	return apexX, apexY, apexZ, nil
}

// AxisDistanceFromApex returns Axis·(point − apex) as a proofbound.BoundedScalar,
// where apex carries its own per-coordinate bound (ConeApex's own doc
// comment) and point/Axis are treated as exact placed coordinates, the same
// convention Dot's own doc comment states for every other vector
// this file asks about.
func AxisDistanceFromApex(apexX, apexY, apexZ proofbound.BoundedScalar, axis, point r3.Vec) proofbound.BoundedScalar {
	dx := proofbound.BoundedSub(proofbound.MeasuredScalar(point.X, 0), apexX)
	dy := proofbound.BoundedSub(proofbound.MeasuredScalar(point.Y, 0), apexY)
	dz := proofbound.BoundedSub(proofbound.MeasuredScalar(point.Z, 0), apexZ)
	return proofbound.BoundedAdd(proofbound.BoundedAdd(
		proofbound.BoundedMul(proofbound.MeasuredScalar(axis.X, 0), dx),
		proofbound.BoundedMul(proofbound.MeasuredScalar(axis.Y, 0), dy)),
		proofbound.BoundedMul(proofbound.MeasuredScalar(axis.Z, 0), dz))
}

// SphereRadius derives a closed, boundary-less Sphere face's own
// radius from its already-proven area/areaBound (Area = 4πR², inverted)
// rather than trusting Sphere.Radius directly. CircleRadius (the
// Cylinder/Cone arms' own route) reads a RIM EDGE's own proven
// circumference, but sphereFaceFluxAndMoment's own scope restriction admits
// a Sphere face only when it carries NO boundary loop at all (this file's
// only reachable Sphere fixture — docs/surface-design.md's own T50 — mints
// no latitude circle for either of its two on-axis pole junctions,
// revolve_build.go's fullRevLoops), so there is no edge here to read a
// circumference from at all, not merely one this file declines to trust.
// The face's own area, by contrast, is ALREADY a proven reading regardless
// of loop count: it is set at construction time from the profile's own
// closed-form Pappus integral (revolve_build.go's faceArea, walkAxisMoment),
// the identical value docs/surface-design.md §6.4's Area-sum block already
// trusts unconditionally for every face kind. Inverting Area = 4πR² through
// proofbound.BoundedQuotient and proofbound.BoundedSqrt is therefore a reuse of an
// already-published, already-tested reading, never a fresh unproven one —
// proofbound.BoundedSqrt's own exact checks make the inversion itself sound: its
// rational bracket (proofbound.RatSqrtDown/proofbound.RatSqrtUp), or, for a zero-bound operand,
// exactFloatSquare's FMA residual proving the float root exact. Go's
// math.Sqrt carries no accuracy contract this file would otherwise have to
// lean on either.
func SphereRadius(area, areaBound float64) (proofbound.BoundedScalar, error) {
	fourPi := proofbound.BoundedMul(proofbound.MeasuredScalar(4, 0), PiScalar())
	rSq := proofbound.BoundedQuotient(area, areaBound, fourPi.Value, fourPi.Bound)
	rB := proofbound.BoundedSqrt(rSq)
	if !(rB.Value > 0) {
		return proofbound.BoundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Sphere face with a positive radius (docs/surface-design.md Table R row R8)`,
			decaderr.ErrUnsupported,
		)
	}
	return rB, nil
}

// AxisIsCoordinateAligned reports whether the torus axis line is exactly
// a signed coordinate axis through the world origin: Axis is bit-identical
// to one of the six signed unit basis vectors, and Center's own two
// components perpendicular to it are bit-identical to zero. This is the
// SAME condition CircleRadius's own doc comment names as making a
// revolve's axis-dependent re-expression bound exactly zero ("every
// product is by 0 or 1 and nothing rounds") — the reason the Cylinder/Cone/
// Sphere arms still derive their own radius from a rim or from area,
// regardless of axis alignment, is that their derivation has to work
// uniformly whether the axis is aligned or not, and doing so gives a bound
// that is automatically zero in the aligned case without a separate gate.
//
// A Torus has no such uniform, axis-independent derivation for Major and
// Minor (torusFaceFluxAndMoment's own doc comment proves why: the two rims
// of a symmetric tube zone do not determine them, even together with the
// face's own proven area — the system is genuinely under-determined, not
// merely hard). So this file cannot avoid reading Torus.Major/Torus.Minor
// from the tag directly the way it avoids reading Cylinder.Radius/
// Sphere.Radius directly; instead it GATES on the one condition under which
// that read carries the identical zero bound the other arms' own
// derivations already prove for a coordinate-aligned axis, and refuses
// (ErrUnsupported, R8) every other axis placement rather than publish a
// Major or Minor this file cannot bound.
func AxisIsCoordinateAligned(axis, center r3.Vec) bool {
	switch axis {
	case r3.NewVec(1, 0, 0), r3.NewVec(-1, 0, 0):
		return center.Y == 0 && center.Z == 0
	case r3.NewVec(0, 1, 0), r3.NewVec(0, -1, 0):
		return center.X == 0 && center.Z == 0
	case r3.NewVec(0, 0, 1), r3.NewVec(0, 0, -1):
		return center.X == 0 && center.Y == 0
	default:
		return false
	}
}
