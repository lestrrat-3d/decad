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
// Both rows turn on one fact about the band: a circular patch is RULED between
// two directrices, so the Cone it publishes is its surface only to within a
// proven bound (§8.3, capblend_departure.go). DX7 widens its own reading by
// that bound. A provenly opposing point lists its patch; only a patch with no
// such point and a straddling range is undecided.
//
// The two rows read that fact through different halves of it. DX7's bound is a
// WORLD-space one and never reaches zero on a placed band: a mitered corner's
// angular skew is one half of it, and the placement's own independent rounding
// of every coordinate the build emits is the other, which leaves the tag even
// where the two windows coincide exactly. DX8 asks only whether the patch is a
// developable cone sector at all — a plane-local question the windows decide
// outright — and refuses for a band holding a mitered patch.
//
// DX7's reading owes a SECOND term, on every band and not only a mitered one:
// it reads each patch's normal through Face.NormalAt, whose answer is a
// direction the arm computed rather than the surface's own — a Cone's arm
// takes a float cosine and sine of the held half angle, which no held pair
// satisfies exactly — and it reads it at a POINT the survey itself computed,
// which is not the azimuth the reading is then treated as. A decision that
// keeps the sampled value and drops either gap answers for a direction the face
// never claimed, so capPatchNormalRange charges the whole distance from its
// sampled reading to the patch's own exactly enclosed one (capblend_normal.go)
// as part of the allowance it returns. It is what keeps a pull the reading
// cannot separate from the patch's own tangent undecided rather than cleared,
// and no all-clear is proven outright unless BOTH terms are proven zero.
//
// DX9 (the wall survey) stays deliberately Suspect: a cap blend is not one
// constant section at one height, and the existing 2D spanning-disk proof
// does not decide it (runSurveys, survey.go).

// capBlendUndercuts surveys a cap-blend body's patches against the pull
// (DX7): each patch's published normal range is read from its OWN built Face —
// Face.NormalAt already carries the correct outward sign (the .reversed bit
// each patch sets at build time, capblend_geom.go) — sampled at enough
// azimuths to recover the patch's normal as A*cos(theta)+B*sin(theta)+C (a
// Cone's normal is that form in its own local azimuth; a Plane's is the
// degenerate A=B=0 case) and then read over the window [th0, th1] through a
// proven enclosure of that form's own extremes (capblend_normal.go). The
// ordinary (unchanged) side walls and caps are surveyed by the SAME per-role
// wallNormalRange logic prismUndercuts already runs, since a cap-blend body's
// non-patch faces are built exactly like a prism's.
//
// That range is the patch's own only where the patch's surface IS the one it
// publishes. A circular patch's is not, so its reading is widened by the
// departure the build already stamped on the face (capblend_departure.go);
// every patch's reading is also widened by the whole distance its own readings
// can sit from the surface it publishes. Both widenings are composed inside
// capPatchNormalRange, so the reading it returns is already the complete
// allowance — this survey reads it rather than composing it further. The
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
		// reading already arrives complete: capPatchNormalRange's circular arm
		// composes the patch's own departure from the surface it publishes
		// (capblend_departure.go) into it directly, and its flat arm takes
		// that same departure already composed into Face.NormalAt's own
		// published bound (topology.go), since a flat patch has only the one
		// reading to widen. Composing f.normalBound again here would charge a
		// flat patch's departure twice.
		allow := reading
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
		// azimuth: a CIRCULAR patch because the surface it publishes is its own
		// only to within its departure (capblend_departure.go,
		// docs/modify-reach-design.md §8.3), and any patch at all because the
		// reading was assembled from bounded readings. A point proven to oppose
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
// correct outward sign), beside a proven allowance on that range.
//
// The two arms compose the patch's own departure from the surface it
// publishes (capblend_departure.go) differently, because they widen a
// different number of readings. A flat (non-circular) patch has exactly one
// reading, so its allowance IS that reading's own published bound —
// Face.NormalAt already composes the departure into it (normalMeasurement,
// topology.go) before this function ever sees it. A circular patch has three
// readings assembled into a recovered form, so its allowance is assembled
// here, and it composes the departure itself as one of its own terms.
//
// A Plane patch's is a single value under that one reading's own bound. A
// Cone's (regular or apex) is A*cos(phi)+B*sin(phi)+C in the azimuth
// phi = theta - th0 measured from the window's own start, recovered from three
// NormalAt evaluations at phi = 0, pi/2, pi — f(0)=A+C, f(pi/2)=B+C, f(pi)=-A+C
// solves uniquely for A, B, C — and read over [0, th1-th0], the window IN THAT
// SAME azimuth. Reading the recovered local coefficients over the global
// [th0, th1] instead scans the wrong arc of the circle whenever th0 is not
// zero, and reports a range the patch never takes.
//
// Neither the recovery nor the reading of it is taken at face value, and the
// allowance is measured rather than estimated in both places
// (capblend_normal.go):
//
//   - The recovered a, b, c are charged their WHOLE distance from the
//     coefficients the patch's own tag and placed frame really give, enclosed
//     exactly by capPatchNormalModel. That one term covers every way the three
//     readings depart from the form they are read as, and there are three: the
//     arm's own arithmetic (normal_bound.go), the displacement of the point the
//     survey sampled at from the azimuth it asked for — a float sine and cosine
//     and two rounded maps, which no reading's own bound speaks about, since
//     Face.NormalAt bounds the normal AT the point it is handed — and the
//     rounded pi/2 and pi spacing between the three azimuths. The model's own
//     departure from a single harmonic, which a placed frame's near-circle
//     leaves, is charged beside it as slop.
//   - The recovered form's extremes over the window are ENCLOSED
//     (harmonicWindowRange) rather than evaluated in float64, so the sine,
//     cosine, arctangent and multiply-add a float reading would round are not
//     merely bounded but absent, and each extreme's own remaining enclosure
//     width is charged.
//
// So the patch's exact component at every azimuth of its window lies within the
// returned allowance of [lo, hi], and each end of that interval lies within the
// same allowance of the extreme it stands for — which is what lets DX7 read the
// minimum in both directions (capBlendUndercuts).
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
	atAzimuth := func(theta float64) (float64, bool) {
		sin, cos := math.Sincos(theta)
		v, _, ok := sampleAt(pl.point(g.cU+r*cos, g.cV+r*sin, g.capZ))
		return v, ok
	}
	f0, ok0 := atAzimuth(g.th0)
	f90, ok90 := atAzimuth(g.th0 + math.Pi/2)
	f180, ok180 := atAzimuth(g.th0 + math.Pi)
	if !ok0 || !ok90 || !ok180 {
		return 0, 0, 0, false
	}
	c := (f0 + f180) / 2
	a := f0 - c
	b := f90 - c
	ra, rb, rc := floatRat(a), floatRat(b), floatRat(c)
	rth0, rth1 := floatRat(g.th0), floatRat(g.th1)
	if ra == nil || rb == nil || rc == nil || rth0 == nil || rth1 == nil {
		return 0, 0, 0, false
	}
	ext, okExt := harmonicWindowRange(ra, rb, rc, new(big.Rat).Sub(rth1, rth0), g.wholeTurn)
	model, okModel := capPatchNormalModel(f, pl, g, p)
	if !okExt || !okModel {
		return 0, 0, 0, false
	}
	lo, hi := ratFloatDown(ext.minLo), ratFloatUp(ext.maxHi)
	rlo, rhi := floatRat(lo), floatRat(hi)
	if rlo == nil || rhi == nil {
		return 0, 0, 0, false
	}
	allow := absSumUpper(
		// The BUILT surface's departure from the published one, which no
		// reading of the published surface reaches. The flat branch above
		// takes this same term through Face.NormalAt's own published bound
		// instead (topology.go), so it is composed once here and once there,
		// never both.
		f.normalBound,
		// How far the recovered form can sit from the patch's own exact one,
		// everywhere on the window at once.
		intervalFloatError(model.a, a),
		intervalFloatError(model.b, b),
		intervalFloatError(model.c, c),
		ratFloatUp(model.slop),
		// How far each reported end can sit from the extreme it stands for:
		// the extreme's own enclosure width, and the float conversion's own
		// outward step.
		ratFloatUp(new(big.Rat).Sub(ext.minHi, ext.minLo)),
		ratFloatUp(new(big.Rat).Sub(ext.minLo, rlo)),
		ratFloatUp(new(big.Rat).Sub(ext.maxHi, ext.maxLo)),
		ratFloatUp(new(big.Rat).Sub(rhi, ext.maxHi)),
	)
	if isNonFinite(allow) || isNonFinite(lo) || isNonFinite(hi) {
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

// capBlendMinRadius is the tightest concave principal radius over a
// cap-blend body (Table DX, DX8).
//
// The reduction below is a claim about the patch the BUILD assembled, not
// merely about the tag it publishes: a Plane or Cone argument about "the
// patch's radius" is only a fact about the built surface where the built
// surface really is that Plane or Cone. The build already proves and stamps
// how far a patch's ruled surface can point away from the surface it
// publishes (`f.normalBound`, `capblend_geom.go`'s `setPatchReadings`, derived
// in `capblend_departure.go`), and this survey answers only where that stamp
// is an EXACT zero — a zero there means the built normal agrees with the
// tag's everywhere on the patch, so the two surfaces share every tangent plane
// and a boundary point, hence are the same surface, and the Plane/Cone
// argument below is a statement about the built patch rather than an
// assumption about it.
//
// `capPatchWindowSkew` decides only the SKEW half of that departure — a
// question about the patch's own kind, whether the build rules it between two
// congruent windows at all. A MITERED circular patch fails even that: the
// build rules it between two differently-swept directrices
// (docs/modify-reach-design.md §8.3), and a straight-ruled surface between two
// skewed arcs is not developable at all — it carries curvature in both
// principal directions, tightening as the corner's own rulings converge, and
// neither the Cone argument below nor the receiver's own section says
// anything about it. But a coinciding window buys back only that half; the
// placement's own independent rounding of every emitted coordinate is the
// other half, and it is nonzero on every placed band, and a whole-turn Cone
// patch owes a further term even unplaced, since its own held half-angle only
// encloses a cosine and sine rather than fixing them exactly. So the skew
// check stays (it still rules out a mitered patch outright), but it is no
// longer sufficient on its own: a band is UNDECIDED here (`Suspect`, through
// runSurveys' own refusal diagnostic) unless every patch's own stamped
// departure is an exact zero, rather than answered with a proven absence the
// patch set does not support.
//
// For a band that passes, this slice's patches are Plane and Cone only — a
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
func capBlendMinRadius(b *Body, cbp capBlendPayload) (radiusOutcome, bool) {
	roles := facesByRole(b)
	for _, patch := range cbp.patches {
		f := roles[patch.role]
		if f == nil {
			return radiusOutcome{}, false
		}
		if capPatchWindowSkew(patch.geom) > 0 || f.normalBound != 0 {
			return radiusOutcome{}, false
		}
	}
	return prismMinRadius(prismPayload{profile: cbp.profile})
}
