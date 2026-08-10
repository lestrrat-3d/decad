package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the cap-blend payload's DX7/DX8 surveys
// (docs/modify-reach-design.md Table DX): per-patch normal ranges against a
// pull (undercut) and the minimum concave principal radius over the patch set
// (Table DX row DX8).
//
// Both rows turn on one fact about the band: a circular patch at a mitered
// corner is RULED between two differently-swept directrices, so the Cone it
// publishes is its surface only to within a proven bound (§8.3). DX7 widens
// its own reading by that bound. A provenly opposing point lists its patch;
// only a patch with no such point and a straddling range is undecided. DX8
// does not answer for such a band at all.
//
// DX7's reading owes a SECOND term, on every band and not only a mitered one:
// it reads each patch's normal through Face.NormalAt, whose answer is a
// direction the arm computed rather than the surface's own — a Cone's arm
// takes a float cosine and sine of the held half angle, which no held pair
// satisfies exactly — and which therefore publishes its own proven bound
// (normal_bound.go). A decision that keeps the sampled value and drops that
// bound answers for a direction the face never claimed, so DX7 carries it
// into the component range and into the allowance the decision reads. It is
// what keeps a pull the reading cannot separate from the patch's own tangent
// undecided rather than cleared, and no all-clear is proven outright unless
// BOTH terms are proven zero.
//
// DX9 (the wall survey) stays deliberately Suspect: a cap blend is not one
// constant section at one height, and the existing 2D spanning-disk proof
// does not decide it (runSurveys, survey.go).

// capBlendUndercuts surveys a cap-blend body's patches against the pull
// (DX7): each patch's published normal range is read from its OWN built Face —
// Face.NormalAt already carries the correct outward sign (the .reversed bit
// each patch sets at build time, capblend_geom.go) — sampled at enough
// azimuths to recover the patch's normal as A*cos(theta)+B*sin(theta)+C
// exactly (a Cone's normal is that form in its own local azimuth; a Plane's
// is the degenerate A=B=0 case) and then read over the window [th0, th1]
// with the same closed-form trigRange the revolve undercut survey already
// uses. The ordinary (unchanged) side walls and caps are surveyed by the
// SAME per-role wallNormalRange logic prismUndercuts already runs, since a
// cap-blend body's non-patch faces are built exactly like a prism's.
//
// That range is the patch's own only where the patch's surface IS the one it
// publishes. A mitered circular patch's is not, so its reading is widened by
// its own proven departure (capPatchNormalAllow), and every patch's is widened
// by the bound its own sampled readings publish (capPatchNormalRange). The
// existential listing and universal all-clear rules then decide it per point:
// see the loop below.
func capBlendUndercuts(b *Body, cbp capBlendPayload, pull r3.Vec) undercutOutcome {
	p, ok := pull.Normalize()
	if !ok {
		return undercutOutcome{}
	}
	roles := facesByRole(b)

	// The ordinary side walls and caps: the same per-role normal-range read
	// prismUndercuts runs, over the RECEIVER's own recorded profile — a
	// chamfered loop's unchanged (non-band) portion has the same wall role
	// and the same normal as an untouched one.
	pl := cbp.prismLike(0, 0)
	du := pl.dir(1, 0, 0).Dot(p)
	dv := pl.dir(0, 1, 0).Dot(p)
	dn := pl.dir(0, 0, 1).Dot(p)
	loops, err := recordLoops(nil, cbp.profile)
	if err != nil {
		return undercutOutcome{}
	}
	// Non-nil from the start: an EMPTY listing is this survey's proven
	// all-clear and a nil one is the undecided answer (BodyReport.Undercuts,
	// verify.go), so the two shapes must stay distinguishable — the same
	// distinction prismUndercuts already keeps.
	faces := []*Face{}
	for li, loop := range loops {
		for _, w := range loop {
			f := roles[fmt.Sprintf("side(%d,%d)", li, w.segs[0])]
			if f == nil {
				return undercutOutcome{}
			}
			mn, mx := wallNormalRange(w, du, dv)
			if opposesPull(mn, mx) {
				faces = append(faces, f)
			}
		}
	}
	for _, cap := range []struct {
		role string
		v    float64
	}{{role: roleCapStart, v: -dn}, {role: roleCapEnd, v: dn}} {
		f := roles[cap.role]
		if f == nil {
			return undercutOutcome{}
		}
		if opposesPull(cap.v, cap.v) {
			faces = append(faces, f)
		}
	}

	// The new patches: read each one's OWN built Face.NormalAt, walked in the
	// payload's own deterministic patch order (Table BX row BX3), so the faces
	// this survey reports — public output through Report.Bodies[i].Undercuts —
	// come back in the same sequence on every call.
	undecided := false
	for _, patch := range cbp.patches {
		f := roles[patch.role]
		if f == nil {
			return undercutOutcome{}
		}
		mn, mx, reading, ok := capPatchNormalRange(f, pl, patch.geom, p)
		if !ok {
			return undercutOutcome{}
		}
		// The two terms answer different questions, and the patch's own
		// departure answers only the second. reading covers how far each
		// SAMPLED component can sit from the component of the face's own
		// normal at the point sampled — the arithmetic the arm ran, which
		// Face.NormalAt publishes and this survey may not drop. The departure
		// covers the built surface at every OTHER azimuth of the window, which
		// no sample reaches at all. Face.NormalAt composes the departure into
		// each sample's own bound too, so it is charged twice at the sampled
		// azimuths; that only widens the reading, and paying it once would mean
		// reaching past the public measurement into the arm bound it
		// deliberately does not break out.
		allow := absSumUpper(capPatchNormalAllow(patch.geom), reading)
		if allow <= 0 {
			// The patch's own surface IS the Cone (or Plane) it publishes AND
			// every reading it was assembled from is exact, so the range above
			// is exact and decides the patch outright.
			if opposesPull(mn, mx) {
				faces = append(faces, f)
			}
			continue
		}
		// Every point of the patch carries an azimuth inside this window, and
		// its own normal component sits within allow of the reading at that
		// azimuth: a MITERED circular patch because the surface it publishes is
		// its own only to within its departure (capblend_geom.go,
		// docs/modify-reach-design.md §8.3), and any patch at all because the
		// reading was assembled from bounded samples. A point proven to oppose
		// lists this patch; only an all-clear needs every point to clear. A
		// remaining straddle makes this patch undecided without discarding
		// other patches already proven to oppose.
		switch {
		case mn > allow:
			// Every point's component is above zero: nothing opposes the pull.
		case mn+allow < 0 && mx-allow > -1:
			// One point is proven below zero, and a point is proven above -1,
			// so this is a genuine opposition rather than opposesPull's
			// exactly-antiparallel carve-out.
			faces = append(faces, f)
		default:
			undecided = true
		}
	}
	if undecided && len(faces) == 0 {
		// Keep an entirely undecided result distinct from a proven all-clear.
		faces = nil
	}
	return undercutOutcome{faces: faces, ok: true, undecided: undecided}
}

// capPatchNormalRange is one patch's published normal-component range against
// the unit pull p, read off its own Face.NormalAt (which already carries the
// correct outward sign), beside a proven allowance on that range: a Plane's is
// a single value; a Cone's (regular or apex) is A*cos(phi)+B*sin(phi)+C in the
// azimuth phi = theta - th0 measured from the window's own start, recovered
// from three NormalAt evaluations at phi = 0, pi/2, pi — f(0)=A+C, f(pi/2)=B+C,
// f(pi)=-A+C solves uniquely for A, B, C — then read with the closed-form
// trigRange over [0, th1-th0], the window IN THAT SAME azimuth. Reading the
// recovered local coefficients over the global [th0, th1] instead scans the
// wrong arc of the circle whenever th0 is not zero, and reports a range the
// patch never takes.
//
// The recovery is exact in the coefficients, never in the samples: each
// NormalAt hands back a direction its own arm computed, under the bound that
// arm proved (normal_bound.go), so the returned allowance carries those bounds
// through the solve. Writing e0, e90, e180 for the three sampled components'
// own allowances (pullComponent), the exact coefficients A, B, C sit within
// e0 + (e0+e180)/2, e90 + (e0+e180)/2 and (e0+e180)/2 of the recovered a, b, c
// — the halved pair is the mean's own share — and a difference of two
// A*cos+B*sin+C forms is bounded everywhere by the sum of its three
// coefficient differences, which is 2.5*e0 + e90 + 1.5*e180. The recovery's own
// float rounding is charged beside it, measured against the exact rational
// solve rather than estimated (recoveryRound), as is each end's final addition.
func capPatchNormalRange(f *Face, pl prismPayload, g capPatchGeom, p r3.Vec) (float64, float64, float64, bool) {
	pLen, okLen := pullLengthUpper(p)
	if !okLen {
		return 0, 0, 0, false
	}
	sampleAt := func(pt r3.Vec) (float64, float64, bool) {
		n, err := f.NormalAt(pt)
		if err != nil {
			return 0, 0, false
		}
		return pullComponent(n, p, pLen)
	}
	if !g.circular {
		v, allow, ok := sampleAt(pl.point(g.sideA.U, g.sideA.V, g.sideZ))
		if !ok {
			return 0, 0, 0, false
		}
		return v, v, allow, true
	}
	// A regular Cone patch's normal is independent of position along its own
	// ruling (azimuth alone determines it), so sampling at the cap radius —
	// which an apex patch's own zero side radius forces anyway — serves both.
	r := g.capRadius
	atAzimuth := func(theta float64) (float64, float64, bool) {
		sin, cos := math.Sincos(theta)
		return sampleAt(pl.point(g.cU+r*cos, g.cV+r*sin, g.capZ))
	}
	f0, e0, ok0 := atAzimuth(g.th0)
	f90, e90, ok90 := atAzimuth(g.th0 + math.Pi/2)
	f180, e180, ok180 := atAzimuth(g.th0 + math.Pi)
	if !ok0 || !ok90 || !ok180 {
		return 0, 0, 0, false
	}
	c := (f0 + f180) / 2
	a := f0 - c
	b := f90 - c
	solve, okSolve := recoveryRound(f0, f90, f180, a, b, c)
	if !okSolve {
		return 0, 0, 0, false
	}
	mn, mx := trigRange(a, b, 0, g.th1-g.th0)
	lo, hi := mn+c, mx+c
	allow := absSumUpper(
		productUpper(2.5, e0), e90, productUpper(1.5, e180),
		solve, sumRound(mn, c), sumRound(mx, c),
	)
	if isNonFinite(allow) {
		return 0, 0, 0, false
	}
	return lo, hi, allow, true
}

// pullComponent is one sampled normal's component against the pull, beside a
// proven allowance on it. Face.NormalAt publishes a bound on the direction it
// hands back, so the component of the face's own exact normal sits within that
// bound — carried through the pull's own length, since |dn·p| <= |dn|*|p| and
// a normalized pull is only near-unit — of the component returned here. The
// dot product's own rounding is charged beside it, measured against the exact
// rational product rather than estimated: every coordinate is a held float, so
// that product is an exact rational.
//
// A survey that kept only n.Value.Dot(p) would decide against a direction the
// face never claimed, which is the whole reason this returns a pair.
func pullComponent(n VecMeasurement, p r3.Vec, pLen float64) (float64, float64, bool) {
	v := n.Value.Dot(p)
	bound, err := magnitudeIn(n.Bound, units.Dimensionless, units.One, "a normal's own bound")
	if err != nil || isNonFinite(bound) || isNonFinite(v) {
		return 0, 0, false
	}
	nv, okN := ivVec3Of(n.Value)
	pv, okP := ivVec3Of(p)
	if !okN || !okP {
		return 0, 0, false
	}
	allow := absSumUpper(productUpper(bound, pLen), intervalFloatError(ivVec3Dot(nv, pv), v))
	if isNonFinite(allow) {
		return 0, 0, false
	}
	return v, allow, true
}

// pullLengthUpper is an upper bound on the normalized pull's own length. It is
// one ulp either side of one, and stating it beats assuming it: a bound scaled
// by a length that is 1+e is a bound, and one scaled by an assumed 1 is not.
func pullLengthUpper(p r3.Vec) (float64, bool) {
	pv, ok := ivVec3Of(p)
	if !ok {
		return 0, false
	}
	length, okSqrt := intervalSqrt(ivVec3NormSq(pv))
	if !okSqrt {
		return 0, false
	}
	up := ratFloatUp(length.hi)
	if isNonFinite(up) || up <= 0 {
		return 0, false
	}
	return up, true
}

// recoveryRound charges the three-sample coefficient solve its OWN float
// rounding: the same solve is redone over exact rationals from the same held
// samples, and each held coefficient is compared against it. That measures the
// rounding rather than estimating it, so a caller composing it with the
// samples' own allowances (capPatchNormalRange) is charging both mechanisms
// exactly once.
func recoveryRound(f0, f90, f180, a, b, c float64) (float64, bool) {
	rf0, rf90, rf180 := floatRat(f0), floatRat(f90), floatRat(f180)
	if rf0 == nil || rf90 == nil || rf180 == nil {
		return 0, false
	}
	rc := new(big.Rat).Quo(new(big.Rat).Add(rf0, rf180), big.NewRat(2, 1))
	ra := new(big.Rat).Sub(rf0, rc)
	rb := new(big.Rat).Sub(rf90, rc)
	round := absSumUpper(
		intervalFloatError(pointInterval(ra), a),
		intervalFloatError(pointInterval(rb), b),
		intervalFloatError(pointInterval(rc), c),
	)
	if isNonFinite(round) {
		return 0, false
	}
	return round, true
}

// sumRound is one float addition's own rounding, against the exact rational
// sum of its two held operands.
func sumRound(x, y float64) float64 {
	rx, ry := floatRat(x), floatRat(y)
	if rx == nil || ry == nil {
		return math.Inf(1)
	}
	return intervalFloatError(pointInterval(new(big.Rat).Add(rx, ry)), x+y)
}

// capBlendMinRadius is the tightest concave principal radius over a
// cap-blend body (Table DX, DX8).
//
// It answers only for a band whose every patch's own surface IS the Plane or
// Cone it publishes. A MITERED circular patch is not: the build rules it
// between two differently-swept directrices (docs/modify-reach-design.md
// §8.3), and a straight-ruled surface between two skewed arcs is not
// developable at all — it carries curvature in both principal directions,
// tightening as the corner's own rulings converge, and neither the Cone
// argument below nor the receiver's own section says anything about it. That
// band is UNDECIDED here (`Suspect`, through runSurveys' own refusal
// diagnostic) rather than answered with a proven absence the patch set does
// not support.
//
// For every other band this slice's patches are Plane and Cone only — a
// chamfer produces no rolling-ball surface, so there is no Torus or Sphere
// case here at all — and NEITHER kind ever tightens the answer beyond what the
// receiver's own unchanged section already gives:
//
//   - a Plane patch has zero curvature in both principal directions (flat),
//     so it contributes no radius at all, exactly as a straight prism wall
//     never does (prismMinRadius only ever reads circular walks);
//   - a regular Cone patch's ruling direction is a straight line (zero
//     curvature there too); its azimuthal principal radius is
//     R(z)/cos(halfAngle) >= R(z), so its tightest point (at the SIDE
//     boundary, R = the original wall's own radius) is never smaller than
//     that same wall's own unchanged radius — and SX7's band-reach gate
//     (buildCapBlend) guarantees a chamfered loop always keeps a strictly
//     positive unchanged run of that same wall, which prismMinRadius reads
//     directly off the untouched RECEIVER profile;
//   - an apex-cone patch's radius shrinks to exactly zero only at its own
//     boundary VERTEX (the untouched original corner point) — a sharp
//     corner/edge feature, which this survey's own convention already
//     excludes everywhere else ("the survey reads faces' principal radii,
//     not edges", shell_cup.go's cupMinRadius) — so reporting it here would
//     single out a reflex corner's un-rounded tip for a reading the SAME
//     corner, unchamfered, never received either.
//
// So for a band of those patches the correct answer is exactly "no new
// concave principal radius" and the whole survey reduces to prismMinRadius on
// the receiver's own untouched profile.
func capBlendMinRadius(cbp capBlendPayload) (radiusOutcome, bool) {
	for _, patch := range cbp.patches {
		if capPatchWindowSkew(patch.geom) > 0 {
			return radiusOutcome{}, false
		}
	}
	return prismMinRadius(prismPayload{profile: cbp.profile})
}
