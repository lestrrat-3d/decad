package stitchflux

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// CircleRim carries one admitted full-circle edge's geometric readings.
type CircleRim struct {
	Center              r3.Vec
	Length, LengthBound float64
	LengthUnbounded     bool
}

// LoopSource reads one topology loop when a flux arm reaches it.
// Circle is called only after EdgeCount reports one edge.
type LoopSource interface {
	EdgeCount() int
	IsOuter() bool
	Circle() (CircleRim, bool, string)
}

// FaceInput carries the readings needed by the curved-face flux formulas.
type FaceInput struct {
	Loops           []LoopSource
	Area, AreaBound float64
}

// ConeInput carries a cone's surface tag without importing the root package.
// ApexBound is a proven bound on how far each coordinate of Origin sits from
// the apex the face's record denotes; the arm widens its apex by it.
type ConeInput struct {
	Origin, Axis      r3.Vec
	Radius, HalfAngle units.Value
	ApexBound         float64
}

// TorusInput carries a torus's surface tag without importing the root package.
type TorusInput struct {
	Center, Axis r3.Vec
	Major, Minor units.Value
}

// PlaneFaceFluxAndMoment is the Plane arm: K_F = 0, so flux_F = (A_F −
// anchor)·S_F with A_F = Frame.Origin() and S_F = sign·n·Area — a constant
// normal over a flat region has no other vector area, so this needs no
// contour sum at all. Every loop must be a single full Circle3 edge (this
// file's own scope restriction, top-of-file doc comment); anything else is
// decaderr.ErrUnsupported. The first moment sums each loop's own disk (or, for a
// hole, negative-disk) contribution to the region's Iu, Iv, Iuu, Ivv, Iuv in
// the face's own local (u, v) frame when an active world-axis moment has an
// in-plane component. Otherwise the arm sums only disk areas. It folds the
// needed terms into the world-axis
// second moments through q_i = c_i + u·U_i + v·V_i (c_i = O_i − anchor_i):
//
//	∫_F q_i² dA = c_i²·Area + 2·c_i·U_i·Iu + 2·c_i·V_i·Iv
//	              + U_i²·Iuu + 2·U_i·V_i·Iuv + V_i²·Ivv
//
// and mx/my/mz = (sign·n_i/2)·that, for i = x, y, z.
func PlaneFaceFluxAndMoment(f FaceInput, frame r3.Frame, anchor r3.Vec, sign float64) (proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, error) {
	var flux, mx, my, mz proofbound.BoundedScalar
	origin := frame.Origin()
	u, v, n := frame.U(), frame.V(), frame.N()
	// A world-axis normal has no in-plane component on its active moment
	// axis. Its first moment needs only the disk areas, not their local
	// centers or second moments. The general tilted-plane path keeps them.
	needDiskMoments := (n.X != 0 && (u.X != 0 || v.X != 0)) ||
		(n.Y != 0 && (u.Y != 0 || v.Y != 0)) ||
		(n.Z != 0 && (u.Z != 0 || v.Z != 0))

	area := proofbound.BoundedScalar{}
	iu, iv := proofbound.BoundedScalar{}, proofbound.BoundedScalar{}
	iuu, ivv, iuv := proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}

	for _, l := range f.Loops {
		if l.EdgeCount() != 1 {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a Plane loop bounded by a single full circle (docs/surface-design.md Table R row R8)`,
				decaderr.ErrUnsupported,
			)
		}
		rim, ok, curveType := l.Circle()
		if !ok {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a Plane loop bounded by a full circle, not %s (docs/surface-design.md Table R row R8)`,
				decaderr.ErrUnsupported, curveType,
			)
		}
		rB, err := CircleRadius(rim.Length, rim.LengthBound, rim.LengthUnbounded)
		if err != nil {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, err
		}
		if !(rB.Value > 0) {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a positive circle radius (docs/surface-design.md Table R row R8)`,
				decaderr.ErrUnsupported,
			)
		}
		loopSign := 1.0
		if !l.IsOuter() {
			loopSign = -1.0
		}

		rr := proofbound.BoundedMul(rB, rB)
		diskArea := proofbound.BoundedMul(PiScalar(), rr)
		if loopSign < 0 {
			diskArea = proofbound.BoundedNeg(diskArea)
		}
		area = proofbound.BoundedAdd(area, diskArea)
		if !needDiskMoments {
			continue
		}

		local := frame.ToLocal(rim.Center)
		projScale := proofbound.AbsSumUpper(proofbound.VecMaxAbs(rim.Center), proofbound.VecMaxAbs(origin))
		projBound := proofbound.AnalyticRoundBound(projScale)
		cu := proofbound.MeasuredScalar(local.X, projBound)
		cv := proofbound.MeasuredScalar(local.Y, projBound)

		fourth := proofbound.BoundedMul(proofbound.MeasuredScalar(0.25, 0), proofbound.BoundedMul(PiScalar(), proofbound.BoundedMul(rr, rr)))
		if loopSign < 0 {
			fourth = proofbound.BoundedNeg(fourth)
		}

		iu = proofbound.BoundedAdd(iu, proofbound.BoundedMul(cu, diskArea))
		iv = proofbound.BoundedAdd(iv, proofbound.BoundedMul(cv, diskArea))
		iuu = proofbound.BoundedAdd(iuu, proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedMul(cu, cu), diskArea), fourth))
		ivv = proofbound.BoundedAdd(ivv, proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedMul(cv, cv), diskArea), fourth))
		iuv = proofbound.BoundedAdd(iuv, proofbound.BoundedMul(proofbound.BoundedMul(cu, cv), diskArea))
	}

	cx := proofbound.BoundedSub(proofbound.MeasuredScalar(origin.X, 0), proofbound.MeasuredScalar(anchor.X, 0))
	cy := proofbound.BoundedSub(proofbound.MeasuredScalar(origin.Y, 0), proofbound.MeasuredScalar(anchor.Y, 0))
	cz := proofbound.BoundedSub(proofbound.MeasuredScalar(origin.Z, 0), proofbound.MeasuredScalar(anchor.Z, 0))

	dot := proofbound.BoundedAdd(proofbound.BoundedAdd(
		proofbound.BoundedMul(cx, proofbound.MeasuredScalar(n.X, 0)),
		proofbound.BoundedMul(cy, proofbound.MeasuredScalar(n.Y, 0))),
		proofbound.BoundedMul(cz, proofbound.MeasuredScalar(n.Z, 0)),
	)
	flux = proofbound.BoundedMul(proofbound.MeasuredScalar(sign, 0), proofbound.BoundedMul(area, dot))

	half := proofbound.MeasuredScalar(0.5, 0)
	moment := func(ci proofbound.BoundedScalar, ni, ui, vi float64) proofbound.BoundedScalar {
		if ni == 0 {
			return proofbound.BoundedScalar{}
		}
		uiB, viB := proofbound.MeasuredScalar(ui, 0), proofbound.MeasuredScalar(vi, 0)
		term := proofbound.BoundedMul(ci, ci)
		term = proofbound.BoundedMul(term, area)
		if ui != 0 || vi != 0 {
			term = proofbound.BoundedAdd(term, proofbound.BoundedMul(proofbound.BoundedMul(proofbound.MeasuredScalar(2, 0), proofbound.BoundedMul(ci, uiB)), iu))
			term = proofbound.BoundedAdd(term, proofbound.BoundedMul(proofbound.BoundedMul(proofbound.MeasuredScalar(2, 0), proofbound.BoundedMul(ci, viB)), iv))
			term = proofbound.BoundedAdd(term, proofbound.BoundedMul(proofbound.BoundedMul(uiB, uiB), iuu))
			term = proofbound.BoundedAdd(term, proofbound.BoundedMul(proofbound.BoundedMul(proofbound.MeasuredScalar(2, 0), proofbound.BoundedMul(uiB, viB)), iuv))
			term = proofbound.BoundedAdd(term, proofbound.BoundedMul(proofbound.BoundedMul(viB, viB), ivv))
		}
		term = proofbound.BoundedMul(proofbound.MeasuredScalar(sign, 0), term)
		term = proofbound.BoundedMul(half, term)
		return proofbound.BoundedMul(proofbound.MeasuredScalar(ni, 0), term)
	}
	mx = moment(cx, n.X, u.X, v.X)
	my = moment(cy, n.Y, u.Y, v.Y)
	mz = moment(cz, n.Z, u.Z, v.Z)
	return flux, mx, my, mz, nil
}

// CylinderFaceFluxAndMoment is the Cylinder arm, scoped to a full-circumference
// tube segment: exactly two loops, each a single full Circle3 edge, at two
// distinct axial positions. K_F = σ·Radius·f.Area reuses the face's own
// already-proven area/areaBound (docs/surface-design.md's own reuse rule);
// the vector area S_F is exactly the ZERO vector for a full-circumference
// wall, proved below, so the (A_F − anchor)·S_F cross term vanishes and
// flux_F is exactly K_F.
//
// S_F = 0, proof. Parametrize the wall p(θ, z) = Origin + z·Axis +
// R·(cosθ·e1 + sinθ·e2), θ ∈ [0, 2π), e1/e2 an orthonormal basis of the
// plane ⊥ Axis. n(θ) = cosθ·e1 + sinθ·e2 does not depend on z, and
// ∫₀^2π cosθ dθ = ∫₀^2π sinθ dθ = 0, so S_F = ∫∫ n(θ)·R dθ dz = 0.
//
// THE FIRST MOMENT, derived and checked two ways (a z-axis-aligned cylinder
// centered on the anchor, where every component but the axial one must
// vanish by the x → −x / y → −y symmetry; and an anchor offset along a
// non-axis direction, checked against a direct single-variable integral of
// q_x² n_x over θ). For component i ∈ {x, y, z}: q_i = A_i + z·B_i + R·cosθ·E_i
// + R·sinθ·F_i (A_i = (Origin−anchor)_i, B_i = Axis_i, E_i = e1_i, F_i =
// e2_i), n_i = cosθ·E_i + sinθ·F_i. Expanding q_i²·n_i and integrating over a
// FULL θ period kills every term whose cos/sin exponents are not BOTH even
// (the standard "either exponent odd integrates to zero over a period"
// fact) — every term of q_i²·n_i survives that test in at most cos² or sin²,
// reducing ∫₀^2π q_i²·n_i dθ = 2π·(A_i + B_i·z)·R·(E_i² + F_i²). Since
// {e1, e2, Axis} is an orthonormal BASIS of ℝ³, the world unit vector ê_i
// decomposes as E_i·e1 + F_i·e2 + B_i·Axis, and |ê_i|² = 1 gives
// E_i² + F_i² = 1 − B_i². Integrating that linear-in-z result over
// z ∈ [zLo, zHi] and folding in the (R/2) from dA = R dθ dz and N_i = ∫(q_i²/2)n_i dA
// gives the closed form cylinderMomentTerm implements:
//
//	N_i = σ·π·R²·Δz·(1 − Axis_i²)·(A_i + Axis_i·zMid)
//
// Δz = |zHi − zLo|, zMid = (zLo + zHi)/2, both measured as Axis·(rim center
// − Origin) — a quantity this formula is invariant to shifting Origin along
// the axis by, checked directly: shifting Origin by t·Axis moves A_i by
// +t·Axis_i and zMid by −t, and Axis_i·(A_i + Axis_i·zMid) is unchanged by
// that shift for any t, so which point of the axis Origin happens to be
// never matters — the same invariance the plane's own A_F choice needs.
func CylinderFaceFluxAndMoment(f FaceInput, origin, axis r3.Vec, anchor r3.Vec, sign float64) (proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, error) {
	var flux, mx, my, mz proofbound.BoundedScalar
	if len(f.Loops) != 2 {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Cylinder face bounded by exactly two full circles (docs/surface-design.md Table R row R8)`,
			decaderr.ErrUnsupported,
		)
	}
	var centers [2]r3.Vec
	var rimEdges [2]CircleRim
	for i, l := range f.Loops {
		if l.EdgeCount() != 1 {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a Cylinder rim bounded by a single full circle (docs/surface-design.md Table R row R8)`,
				decaderr.ErrUnsupported,
			)
		}
		rim, ok, curveType := l.Circle()
		if !ok {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a Cylinder rim bounded by a full circle, not %s (docs/surface-design.md Table R row R8)`,
				decaderr.ErrUnsupported, curveType,
			)
		}
		centers[i] = rim.Center
		rimEdges[i] = rim
	}

	// rB is derived from the FIRST rim's own proven circumference
	// (boundedCircleRadius's own doc comment), never read off Cylinder.Radius
	// directly: both rims denote the same cylinder by construction, so
	// either rim's own proof covers the whole face.
	rB, err := CircleRadius(rimEdges[0].Length, rimEdges[0].LengthBound, rimEdges[0].LengthUnbounded)
	if err != nil {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, err
	}
	if !(rB.Value > 0) {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a positive cylinder radius (docs/surface-design.md Table R row R8)`,
			decaderr.ErrUnsupported,
		)
	}

	z0 := Dot(axis, centers[0].Sub(origin))
	z1 := Dot(axis, centers[1].Sub(origin))
	dz := proofbound.BoundedAbs(proofbound.BoundedSub(z1, z0))
	if !(dz.Value > 0) {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Cylinder face whose two rims sit at different axial positions (docs/surface-design.md Table R row R8)`,
			decaderr.ErrUnsupported,
		)
	}
	zMid := proofbound.BoundedMul(proofbound.MeasuredScalar(0.5, 0), proofbound.BoundedAdd(z0, z1))

	flux = proofbound.BoundedMul(proofbound.MeasuredScalar(sign, 0), proofbound.BoundedMul(rB, proofbound.MeasuredScalar(f.Area, f.AreaBound)))

	piR2 := proofbound.BoundedMul(PiScalar(), proofbound.BoundedMul(rB, rB))
	piR2Dz := proofbound.BoundedMul(piR2, dz)

	moment := func(oi, ai, axisI float64) proofbound.BoundedScalar {
		aTerm := proofbound.BoundedSub(proofbound.MeasuredScalar(oi, 0), proofbound.MeasuredScalar(ai, 0))
		axisZMid := proofbound.BoundedMul(proofbound.MeasuredScalar(axisI, 0), zMid)
		sumTerm := proofbound.BoundedAdd(aTerm, axisZMid)
		oneMinusAxis2 := proofbound.BoundedSub(proofbound.MeasuredScalar(1, 0), proofbound.BoundedMul(proofbound.MeasuredScalar(axisI, 0), proofbound.MeasuredScalar(axisI, 0)))
		term := proofbound.BoundedMul(piR2Dz, proofbound.BoundedMul(oneMinusAxis2, sumTerm))
		return proofbound.BoundedMul(proofbound.MeasuredScalar(sign, 0), term)
	}
	mx = moment(origin.X, anchor.X, axis.X)
	my = moment(origin.Y, anchor.Y, axis.Y)
	mz = moment(origin.Z, anchor.Z, axis.Z)
	return flux, mx, my, mz, nil
}

// ConeFaceFluxAndMoment is the Cone arm, scoped identically in shape to
// CylinderFaceFluxAndMoment: exactly two loops, each a single full Circle3
// edge, at two distinct positions along the cone's own growth Axis. K_F = 0
// — the vector from the apex to any surface point runs along a ruling, and
// NormalAt's own Cone formula (n = cosβ·radial − sinβ·Axis, topology.go)
// makes that ruling's own direction (cosβ·Axis-perpendicular component +
// sinβ·Axis, the same angle β off the radial as n is off Axis but
// complementary) orthogonal to n by construction — a ruling and the
// surface normal at any of its points are always perpendicular, the
// defining property of a ruled surface's normal. So flux_F reduces to the
// pure cross term (apex − anchor)·S_F.
//
// S_F, proof. Parametrize the wall p(θ, z) = apex + z·Axis +
// z·tanβ·(cosθ·e1 + sinθ·e2), z the distance from the apex along Axis,
// θ ∈ [0, 2π), {e1, e2, Axis} an orthonormal basis. The intrinsic normal is
// n(θ) = cosβ·(cosθ·e1 + sinθ·e2) − sinβ·Axis (NormalAt's own formula,
// θ-independent apart from the radial term), and dA = z·tanβ·secβ dθ dz.
// Over a FULL θ period ∫cosθ dθ = ∫sinθ dθ = 0, so only n's constant −sinβ·
// Axis term survives the θ integral, at weight ∫₀^2π dθ = 2π:
//
//	S_F = ∫_zLo^zHi (−2π·sinβ·Axis)·z·tanβ·secβ dz
//	    = −2π·tan²β·Axis · ∫_zLo^zHi z dz = −π·tan²β·(zHi²−zLo²)·Axis
//
// Since R = z·tanβ at any point on the wall, tan²β·z² = R², so
// tan²β·(zHi²−zLo²) = R_hi²−R_lo² (R_lo/R_hi the radii AT zLo/zHi), giving
// the closed form this function implements:
//
//	S_F = π·(R_lo² − R_hi²)·Axis
//
// — the standard cone-shadow identity, needing no trig at the call site:
// R_lo/R_hi come from boundedCircleRadius, never a fresh tan(β) evaluation.
// Verified independently against numeric double-quadrature over several
// random half-angles, anchors and apex placements before landing (this
// PR's own scratch verification; not carried into the tree).
//
// THE FIRST MOMENT, derived the same way (expand q_i²·n_i, integrate over a
// full θ period — every ODD power of cosθ/sinθ vanishes, so only the terms
// surviving in cos²θ+sin²θ = 1 remain — then integrate the z-weighted
// result over [zLo, zHi]). With A_i = apex_i − anchor_i, B_i = Axis_i,
// K = 1−B_i² (the {e1,e2} share of that world axis, from the orthonormal
// basis identity E_i²+F_i² = 1−B_i², CylinderFaceFluxAndMoment's own proof
// of the identical fact):
//
//	M_i = (π·tan²β/2) · [ −2·A_i²·B_i·I1 + 2·A_i·(1−3·B_i²)·I2
//	                      + (2·B_i·(1−2·B_i²) − B_i·K·tan²β)·I3 ]
//
// I1 = (zHi²−zLo²)/2, I2 = (zHi³−zLo³)/3, I3 = (zHi⁴−zLo⁴)/4. tan²β is read
// from the rims' OWN slope ((R_hi−R_lo)/(zHi−zLo))² rather than a second
// math.Tan evaluation of HalfAngle, so this arm calls coneApex's own
// math.Tan exactly once per face, for the apex alone. Verified
// independently against numeric double-quadrature the same way S_F was,
// across several random half-angles, anchors, axes and apex placements.
func ConeFaceFluxAndMoment(f FaceInput, cone ConeInput, anchor r3.Vec, sign float64) (proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, error) {
	var flux, mx, my, mz proofbound.BoundedScalar
	if len(f.Loops) != 2 {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Cone face bounded by exactly two full circles (docs/surface-design.md Table R row R8)`,
			decaderr.ErrUnsupported,
		)
	}
	var centers [2]r3.Vec
	var rimEdges [2]CircleRim
	for i, l := range f.Loops {
		if l.EdgeCount() != 1 {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a Cone rim bounded by a single full circle (docs/surface-design.md Table R row R8)`,
				decaderr.ErrUnsupported,
			)
		}
		rim, ok, curveType := l.Circle()
		if !ok {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a Cone rim bounded by a full circle, not %s (docs/surface-design.md Table R row R8)`,
				decaderr.ErrUnsupported, curveType,
			)
		}
		centers[i] = rim.Center
		rimEdges[i] = rim
	}

	apexX, apexY, apexZ, err := ConeApex(cone.HalfAngle, cone.Radius, cone.Origin)
	if err != nil {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, err
	}
	if cone.ApexBound != 0 {
		for _, c := range []*proofbound.BoundedScalar{&apexX, &apexY, &apexZ} {
			c.Bound = proofbound.AbsSumUpper(c.Bound, cone.ApexBound)
		}
	}

	var radii [2]proofbound.BoundedScalar
	for i := range rimEdges {
		rB, err := CircleRadius(rimEdges[i].Length, rimEdges[i].LengthBound, rimEdges[i].LengthUnbounded)
		if err != nil {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, err
		}
		if !(rB.Value > 0) {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a positive cone rim radius (docs/surface-design.md Table R row R8)`,
				decaderr.ErrUnsupported,
			)
		}
		radii[i] = rB
	}

	var zs [2]proofbound.BoundedScalar
	for i := range centers {
		zs[i] = AxisDistanceFromApex(apexX, apexY, apexZ, cone.Axis, centers[i])
	}
	loIdx, hiIdx := 0, 1
	if zs[0].Value > zs[1].Value {
		loIdx, hiIdx = 1, 0
	}
	zLo, zHi := zs[loIdx], zs[hiIdx]
	rLo, rHi := radii[loIdx], radii[hiIdx]
	if !(zLo.Value > 0) {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Cone face whose rims sit beyond the apex along its own axis (docs/surface-design.md Table R row R8)`,
			decaderr.ErrUnsupported,
		)
	}
	dz := proofbound.BoundedSub(zHi, zLo)
	if !(dz.Value > 0) {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Cone face whose two rims sit at different positions along its own axis (docs/surface-design.md Table R row R8)`,
			decaderr.ErrUnsupported,
		)
	}

	// S_F = pi*(rLo^2 - rHi^2)*Axis; the cross term is (apex-anchor).S_F,
	// which since S_F is a scalar multiple of the unit Axis reduces to
	// sMag * Axis.(apex-anchor).
	sMag := proofbound.BoundedMul(PiScalar(), proofbound.BoundedSub(proofbound.BoundedMul(rLo, rLo), proofbound.BoundedMul(rHi, rHi)))
	axisDotApexMinusAnchor := proofbound.BoundedNeg(AxisDistanceFromApex(apexX, apexY, apexZ, cone.Axis, anchor))
	flux = proofbound.BoundedMul(proofbound.MeasuredScalar(sign, 0), proofbound.BoundedMul(sMag, axisDotApexMinusAnchor))

	// tan^2(beta) from the rims' own slope, never a second HalfAngle trig
	// evaluation (this function's own doc comment).
	slopeNum := proofbound.BoundedSub(rHi, rLo)
	slope := proofbound.BoundedQuotient(slopeNum.Value, slopeNum.Bound, dz.Value, dz.Bound)
	tan2 := proofbound.BoundedMul(slope, slope)

	zHiSq, zLoSq := proofbound.BoundedMul(zHi, zHi), proofbound.BoundedMul(zLo, zLo)
	i1 := proofbound.BoundedMul(proofbound.MeasuredScalar(0.5, 0), proofbound.BoundedSub(zHiSq, zLoSq))
	zHiCu := proofbound.BoundedMul(zHiSq, zHi)
	zLoCu := proofbound.BoundedMul(zLoSq, zLo)
	i2 := proofbound.BoundedQuotient(proofbound.BoundedSub(zHiCu, zLoCu).Value, proofbound.BoundedSub(zHiCu, zLoCu).Bound, 3, 0)
	zHi4 := proofbound.BoundedMul(zHiSq, zHiSq)
	zLo4 := proofbound.BoundedMul(zLoSq, zLoSq)
	i3 := proofbound.BoundedMul(proofbound.MeasuredScalar(0.25, 0), proofbound.BoundedSub(zHi4, zLo4))

	one := proofbound.MeasuredScalar(1, 0)
	two := proofbound.MeasuredScalar(2, 0)
	three := proofbound.MeasuredScalar(3, 0)
	half := proofbound.MeasuredScalar(0.5, 0)
	piTan2 := proofbound.BoundedMul(PiScalar(), tan2)

	moment := func(apexI proofbound.BoundedScalar, anchorI, axisI float64) proofbound.BoundedScalar {
		ai := proofbound.BoundedSub(apexI, proofbound.MeasuredScalar(anchorI, 0))
		bi := proofbound.MeasuredScalar(axisI, 0)
		biSq := proofbound.BoundedMul(bi, bi)

		term1 := proofbound.BoundedNeg(proofbound.BoundedMul(proofbound.BoundedMul(two, proofbound.BoundedMul(ai, ai)), proofbound.BoundedMul(bi, i1)))

		oneMinus3BiSq := proofbound.BoundedSub(one, proofbound.BoundedMul(three, biSq))
		term2 := proofbound.BoundedMul(proofbound.BoundedMul(two, proofbound.BoundedMul(ai, oneMinus3BiSq)), i2)

		oneMinus2BiSq := proofbound.BoundedSub(one, proofbound.BoundedMul(two, biSq))
		partA := proofbound.BoundedMul(two, proofbound.BoundedMul(bi, oneMinus2BiSq))
		oneMinusBiSq := proofbound.BoundedSub(one, biSq)
		partB := proofbound.BoundedMul(bi, proofbound.BoundedMul(oneMinusBiSq, tan2))
		coef3 := proofbound.BoundedSub(partA, partB)
		term3 := proofbound.BoundedMul(coef3, i3)

		sum := proofbound.BoundedAdd(proofbound.BoundedAdd(term1, term2), term3)
		m := proofbound.BoundedMul(piTan2, proofbound.BoundedMul(half, sum))
		return proofbound.BoundedMul(proofbound.MeasuredScalar(sign, 0), m)
	}
	mx = moment(apexX, anchor.X, cone.Axis.X)
	my = moment(apexY, anchor.Y, cone.Axis.Y)
	mz = moment(apexZ, anchor.Z, cone.Axis.Z)
	return flux, mx, my, mz, nil
}

// SphereFaceFluxAndMoment is the Sphere arm, scoped to a face with NO
// boundary loop at all: a complete, closed spherical shell — the only shape
// this evaluator's reachable fixtures ever present (a semicircle revolved a
// full turn about its own diameter, docs/surface-design.md's own T50:
// fullRevLoops mints a latitude circle only for a junction OFF the revolve
// axis, and both of a diameter-revolved semicircle's own junctions are
// poles ON it, so neither junction contributes an edge and the face carries
// zero loops, zero edges, and — since Body.Vertices() derives from
// Body.Edges() — the WHOLE stitched body carries zero vertices too).
// Anything wider (a spherical zone or cap bounded by one or two rim
// circles) is refused rather than guessed at: this file never builds the
// general ½∮p×dr contour-sum machinery (top-of-file doc comment), and no
// reachable fixture today exercises that shape, so admitting it would be
// untested code with no fixture behind it.
//
// K_F. Anchored at the sphere's own Center, p−Center is parallel to n at
// every surface point (n = σ·(p−Center)/Radius, σ = ±1 from f.reversed), so
// (p−Center)·n = σ·Radius identically — the same identity the Cylinder arm
// uses, with the sphere's own Center standing in for a point on the
// cylinder's axis. So flux_F = K_F + (Center−anchor)·S_F, K_F =
// σ·Radius·f.Area — reusing the face's own already-proven area/areaBound
// exactly as the Cylinder arm's own doc comment states the reuse — with
// Radius itself read through boundedSphereRadius, never bare off
// Sphere.Radius (that function's own doc comment).
//
// S_F = 0, EXACTLY, rather than by a trig integral collapsing to zero: a
// face with no boundary loop has no contour to sum ∮p×dr over at all, so
// its vector area is the empty sum. (This also follows from an unrelated,
// more general fact — ∫_F n dA over ANY closed orientable surface is the
// zero vector, by the divergence theorem applied to a constant field — but
// the loop-free construction here makes it true by construction, needing no
// such argument.) So the (Center−anchor)·S_F cross term vanishes for EVERY
// anchor, not merely the sphere's own center, and flux_F is exactly K_F
// regardless of which anchor stitchCurvedMass passes in — the property that
// makes this arm safe to reach with the r3.Vec{} anchor stitch.go
// substitutes when the shared vertex table is empty (stitch.go's own doc
// comment at evalStitchContext, written against exactly this fixture before
// this arm landed).
//
// THE FIRST MOMENT. Because the face IS a complete closed boundary all on
// its own — never one of several faces sharing a boundary with others, by
// this arm's own zero-loop scope — its own moment is the WHOLE ball's own
// first moment, not a partial surface-integral term that only sums to
// something meaningful alongside sibling faces the way Plane/Cylinder/
// Cone's per-face terms do. The general shift-of-origin identity for a
// region Ω of volume V centered at Ĉ: ∫_Ω(x_i−a_i)dV = V·(Ĉ_i−a_i), applies
// directly with Ω the ball, Ĉ = Center, and V this face's own signed
// sub-volume flux_F/3 (flux_F is exactly 3·(signed volume) by construction,
// the same normalization stitchCurvedMass's own vol := fluxSum/3 uses for
// the total). So M_i = (flux_F/3)·(Center_i−anchor_i), checked directly
// against the divergence-theorem surface integral
// ∫_S((p_i−a_i)²/2)n_i dA by hand for a sphere: expanding
// (p_i−a_i)² = r_i²+2r_id_i+d_i² (r = p−Center, d = Center−anchor) and
// integrating each term over the sphere (∫r_i dA = ∫r_i³ dA = 0 by odd
// symmetry, ∫r_i² dA = (4/3)πR⁴ from ∫r_i²dA = (1/3)∫|r|²dA = (1/3)R²·Area)
// gives M_i = σ/(2R)·[2d_i·(4/3)πR⁴] = σ·d_i·(4/3)πR³ — the identical
// closed form, since (4/3)πR³ = R·Area/3 = K_F/(3σ) and σ²=1.
func SphereFaceFluxAndMoment(f FaceInput, center r3.Vec, anchor r3.Vec, sign float64) (proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, error) {
	var flux, mx, my, mz proofbound.BoundedScalar
	if len(f.Loops) != 0 {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Sphere face with no boundary at all (docs/surface-design.md Table R row R8)`,
			decaderr.ErrUnsupported,
		)
	}
	rB, err := SphereRadius(f.Area, f.AreaBound)
	if err != nil {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, err
	}

	flux = proofbound.BoundedMul(proofbound.MeasuredScalar(sign, 0), proofbound.BoundedMul(rB, proofbound.MeasuredScalar(f.Area, f.AreaBound)))

	third := proofbound.BoundedQuotient(flux.Value, flux.Bound, 3, 0)
	moment := func(centerI, anchorI float64) proofbound.BoundedScalar {
		d := proofbound.BoundedSub(proofbound.MeasuredScalar(centerI, 0), proofbound.MeasuredScalar(anchorI, 0))
		return proofbound.BoundedMul(third, d)
	}
	mx = moment(center.X, anchor.X)
	my = moment(center.Y, anchor.Y)
	mz = moment(center.Z, anchor.Z)
	return flux, mx, my, mz, nil
}

// TorusFaceFluxAndMoment is the Torus arm, scoped to the one shape this
// evaluator's own reachable fixture builds: a symmetric tube zone spanning
// EXACTLY the tube's own outer quarter-to-quarter window — φ ∈ [−π/2, π/2],
// measuring φ from the plane through Center perpendicular to the axis
// (φ = 0 there: the tube's own outermost point, ρ = Major+Minor, at
// Center's own axial position) — bounded by exactly two full Circle3
// rims, one Minor above Center's own axial position and one Minor below
// it. offAxisSemicircleSketch (docs/surface-design.md's own T53) is exactly
// this shape: a straight wall at ρ = Major (the chord, closing the tube's
// own equatorial diameter, revolved into the face's own Cylinder sibling)
// and a semicircular arc bulging OUTWARD from ρ = Major to ρ = Major+Minor
// and back (revolved into this Torus face).
//
// WHY THIS FILE CANNOT ADMIT A WIDER WINDOW, and why that is a genuine
// mathematical limit rather than a missing derivation. Parametrizing the
// torus surface by (θ, φ) — θ the revolve angle, φ the tube angle, with
// ρ(φ) = Major + Minor·cosφ and axial offset z(φ) = Minor·sinφ from
// Center's own axial position — the outward normal is
// n(φ) = cosφ·radial + sinφ·Axis, and (p−Center)·n = Major·cosφ + Minor
// pointwise (the identity this file's own top-of-file doc table states).
// Integrating that over a φ window [φlo, φhi] and a full θ turn gives
// K_F = 2π·Minor·∫ (Major·cosφ+Minor)(Major+Minor·cosφ) dφ, whose
// antiderivative carries a term LINEAR IN THE RAW ANGLE φ itself (from
// ∫cos²φ dφ = φ/2 + sin2φ/4), not reducible to sinφ/cosφ alone. Recovering
// φlo/φhi as raw angles from the two rims' own proven radius and axial
// position needs an inverse trig function of computed floating data — this
// evaluator has no sound bound for one (this file's own coneApex doc
// comment states why at length; the same limit applies here). Nor can
// Major/Minor/φlo/φhi be recovered algebraically without one: two rims of
// the SAME radius at axial offsets ±e from Center satisfy
// (ρ−Major)²+e² = Minor² for infinitely many (Major, Minor) pairs — e.g.
// Major=10, Minor=5 (φ=±90°, e=5) and Major=8, Minor=√29≈5.385 (φ≈±68.2°,
// e=5) both put a full circle of radius 10 at axial offset ±5 from the
// SAME Center — a genuine, checked-by-hand ambiguity, not a derivation this
// file merely has not found. So the ONLY φ window this file can integrate
// without an unbounded trig call is one whose endpoints are KNOWN constants
// rather than recovered ones — ±π/2 is the sole such window this
// evaluator's own construction ever reaches (a complete, zero-loop torus
// would be the OTHER such window, Δφ=2π, the same shape SphereFaceFluxAndMoment's
// own zero-loop scope takes for a sphere, but no reachable fixture in this
// tree ever builds one) — and detecting THAT specific window from held data
// is what the two gates below establish, structurally, never by measuring
// an angle.
//
// GATE 1 — torusAxisIsCoordinateAligned. Trusting Major and Minor bare off
// the tag at all needs the identical zero-bound condition boundedCircleRadius's
// own doc names for the OTHER radius fields' axis-dependent rounding.
//
// GATE 2 — the window check. Given Gate 1, Major and Minor carry a proven
// zero bound, so e_lo = Axis·(rimCenter_lo − Center) and e_hi similarly are
// EXACT (boundedDot's own zero-bound result for an axis-aligned Axis — every
// product is by 0 or 1). Checking {e_lo, e_hi} == {−Minor, +Minor} EXACTLY
// (bit-for-bit, never a tolerance) is what proves sinφ_lo = −1, sinφ_hi = +1
// EXACTLY — a pure algebraic consequence, not a measurement — and therefore
// cosφ_lo = cosφ_hi = 0 and φ_lo, φ_hi = ∓π/2 EXACTLY, since sin = ±1 pins φ
// uniquely on the tube's own principal branch. This is reject-only: a face
// that fails the check is refused, never admitted on a near-match — a small
// residual here would prove nothing (CLAUDE.md's own rule), so none is
// accepted; only exact equality of already-exact quantities is.
//
// THE CLOSED FORMS, verified against independent numeric double integration
// (θ, φ grid, ~6×10⁵ samples, this PR's own scratch verification — not
// carried into the tree) for both a coordinate-aligned and a deliberately
// oblique Torus (arbitrary Center, Axis and anchor), agreeing to 4–5
// significant digits at that grid's own resolution:
//
//	K_F     = 3·π²·Major·Minor² + 4·π·Minor·(Major² + Minor²)
//	M_i     = π·Minor·(Center_i − anchor_i)·
//	            (2·Major²·(1 − Axis_i²) + π·Major·Minor + (4/3)·Minor²)
//
// K_F needs no cross term: the two rims share the SAME radius (Major, by
// Gate 2), so — exactly as CylinderFaceFluxAndMoment's own full-circumference
// argument shows for two equal-radius circles — the face's own vector area
// S_F is the zero vector, and flux_F is exactly K_F for every anchor. M_i's
// own derivation integrates (p_i−anchor_i)²·n_i over a full θ period first
// (every odd power of cosθ/sinθ vanishing, the same fact
// CylinderFaceFluxAndMoment's and ConeFaceFluxAndMoment's own moment proofs
// use), then over φ ∈ [−π/2, π/2] — where ∫cosφ dφ = 2, ∫sinφ dφ = 0,
// ∫cos²φ dφ = ∫sin²φ dφ = π/2, ∫cos³φ dφ = 4/3, ∫sin²φ·cosφ dφ = 2/3, and
// every ODD-in-φ integral (sinφ alone, sinφ·cos²φ, sin³φ, sinφ·cosφ) is
// zero over the symmetric interval — collapsing every term but two, whose
// (1 − Axis_i²)-weighted and constant parts combine to the M_i form above
// (this PR's own scratch derivation, hand-expanded and cross-checked
// against the numeric integral).
func TorusFaceFluxAndMoment(f FaceInput, t TorusInput, anchor r3.Vec, sign float64) (proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, error) {
	var flux, mx, my, mz proofbound.BoundedScalar
	refuse := func(msg string) (proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, error) {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
			`%w: %s (docs/surface-design.md Table R row R8)`, decaderr.ErrUnsupported, msg,
		)
	}
	if len(f.Loops) != 2 {
		return refuse(`Stitch's flux path needs a Torus face bounded by exactly two full circles`)
	}
	if !AxisIsCoordinateAligned(t.Axis, t.Center) {
		return refuse(`Stitch's flux path needs a Torus revolved about a coordinate-aligned axis through the origin; this evaluator has no sound bound for Major/Minor's own rounding otherwise`)
	}
	majorValue, merr := t.Major.In(units.Millimeter)
	if merr != nil {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(`decad: a torus's major radius is not a length: %w`, merr)
	}
	minorValue, nerr := t.Minor.In(units.Millimeter)
	if nerr != nil {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(`decad: a torus's minor radius is not a length: %w`, nerr)
	}
	if !(majorValue > 0) || !(minorValue > 0) {
		return refuse(`Stitch's flux path needs a Torus with a positive Major and Minor radius`)
	}

	var centers [2]r3.Vec
	for i, l := range f.Loops {
		if l.EdgeCount() != 1 {
			return refuse(`Stitch's flux path needs a Torus rim bounded by a single full circle`)
		}
		rim, ok, curveType := l.Circle()
		if !ok {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a Torus rim bounded by a full circle, not %s (docs/surface-design.md Table R row R8)`,
				decaderr.ErrUnsupported, curveType,
			)
		}
		centers[i] = rim.Center
	}

	e0 := Dot(t.Axis, centers[0].Sub(t.Center))
	e1 := Dot(t.Axis, centers[1].Sub(t.Center))
	halfWindow := (e0.Value == -minorValue && e1.Value == minorValue) ||
		(e0.Value == minorValue && e1.Value == -minorValue)
	if !halfWindow {
		return refuse(`Stitch's flux path needs a Torus face bounded by the tube's own two equatorial rims (its ±π/2 window); this evaluator has no sound way to recover a narrower angular window without an unbounded trig computation`)
	}

	major := proofbound.MeasuredScalar(majorValue, 0)
	minor := proofbound.MeasuredScalar(minorValue, 0)
	majorSq := proofbound.BoundedMul(major, major)
	minorSq := proofbound.BoundedMul(minor, minor)
	pi := PiScalar()
	piSq := proofbound.BoundedMul(pi, pi)

	term1 := proofbound.BoundedMul(proofbound.MeasuredScalar(3, 0), proofbound.BoundedMul(piSq, proofbound.BoundedMul(major, minorSq)))
	term2 := proofbound.BoundedMul(proofbound.MeasuredScalar(4, 0), proofbound.BoundedMul(pi, proofbound.BoundedMul(minor, proofbound.BoundedAdd(majorSq, minorSq))))
	kf := proofbound.BoundedAdd(term1, term2)
	flux = proofbound.BoundedMul(proofbound.MeasuredScalar(sign, 0), kf)

	majMinorPi := proofbound.BoundedMul(pi, proofbound.BoundedMul(major, minor))
	fourThirdsMinorSq := proofbound.BoundedMul(proofbound.BoundedQuotient(4, 0, 3, 0), minorSq)
	moment := func(centerI, anchorI, axisI float64) proofbound.BoundedScalar {
		axisISq := proofbound.BoundedMul(proofbound.MeasuredScalar(axisI, 0), proofbound.MeasuredScalar(axisI, 0))
		oneMinusAxisISq := proofbound.BoundedSub(proofbound.MeasuredScalar(1, 0), axisISq)
		bracket := proofbound.BoundedAdd(
			proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.MeasuredScalar(2, 0), proofbound.BoundedMul(majorSq, oneMinusAxisISq)), majMinorPi),
			fourThirdsMinorSq,
		)
		d := proofbound.BoundedSub(proofbound.MeasuredScalar(centerI, 0), proofbound.MeasuredScalar(anchorI, 0))
		m := proofbound.BoundedMul(pi, proofbound.BoundedMul(minor, proofbound.BoundedMul(d, bracket)))
		return proofbound.BoundedMul(proofbound.MeasuredScalar(sign, 0), m)
	}
	mx = moment(t.Center.X, anchor.X, t.Axis.X)
	my = moment(t.Center.Y, anchor.Y, t.Axis.Y)
	mz = moment(t.Center.Z, anchor.Z, t.Axis.Z)
	return flux, mx, my, mz, nil
}
