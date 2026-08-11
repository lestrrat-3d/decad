package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the analytic survey layer of
// docs/evaluator-design.md §10 and docs/verification-design.md §6: the wall,
// undercut and minimum-radius questions answered outright on the analytic
// prism, revolve and cup bodies. Each survey reads the evaluator's own payload —
// the recorded 2D region a prism lifts or a revolve sweeps — because the
// spanning-ball and curvature readings are closed-form facts of that section:
// a prism's inscribed balls are its profile's inscribed disks with the height
// as the vertical fit, and a solid of revolution's are the disks of its
// meridian section (mirrored across the axis for a full turn, wedge-bounded
// by the caps for a partial one). A prism, cup and cap-blend body's undercut
// survey reads each face's normal-component range enclosed over the rationals
// and decides it three-valued — clear, opposing, or undecided
// (survey_undercut.go) — rather than taking a float range at face value;
// revolveUndercuts is not yet converted to it, a deliberate scope line rather
// than an oversight. A body this file cannot decide — a payload no shipped
// feature builds — leaves every asked question undecided, which reads
// Suspect, never a silent pass.

// surveyReason retains why a survey was not decided. The zero reason is a
// numerical or geometric undecided result; surveyFacetedUnsupported records
// the known payload capability gap before runSurveys maps it to a diagnostic.
type surveyReason int

const (
	surveyUndecided surveyReason = iota
	surveyFacetedUnsupported
)

// wallOutcome is one body's wall reading: ok=false is an undecided survey;
// reason distinguishes its cause. reading == nil (with ok) is the proven
// determination that no wall exists. bound is the PROVEN error bound beside
// reading (millimetres), zero only where the arm that produced reading is
// itself exactly representable — never asserted, always the arm's own
// computed figure.
type wallOutcome struct {
	reading *float64 // millimetres
	bound   float64  // millimetres; meaningful only when reading != nil
	ok      bool
	reason  surveyReason
}

// undercutOutcome is one body's undercut listing: ok=false is an unrecoverable
// undecided survey; reason distinguishes its cause. undecided records a
// straddling face that coexists with any faces already proven to oppose. faces
// is non-nil when every face is decided or any face is proven to oppose; an
// empty listing is the proven all-clear.
type undercutOutcome struct {
	faces     []*Face
	ok        bool
	undecided bool
	reason    surveyReason
}

// radiusOutcome is one body's tightest concave radius: ok=false is
// undecided and reason distinguishes its cause; reading == nil (with ok) is
// the proven all-convex answer. bound is the PROVEN error bound beside
// reading (millimetres), zero only where the winning walk's own radius is
// exactly representable — a recorded CircleSeg's millimetre number, never a
// math.Hypot evaluation.
type radiusOutcome struct {
	reading *float64 // millimetres
	bound   float64  // millimetres; meaningful only when reading != nil
	ok      bool
	reason  surveyReason
}

// recordLoops resolves the recorded profile into coalesced walk loops,
// exactly as the prism evaluator builds its side faces — the surveys must
// see the same face decomposition the topology carries.
func recordLoops(budget *workBudget, profile ProfileRecord) ([][]sideWalk, error) {
	// One free-form counter for the whole record: the surveys read a built body's
	// own section with no preflight counter in hand, so the ceiling starts here
	// and spans every loop below.
	work := newFreeformWork()
	var out [][]sideWalk
	for _, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
		if err := wallBudgetStep(budget); err != nil {
			return nil, err
		}
		raw := make([]sideWalk, len(loop.Segments))
		for i, seg := range loop.Segments {
			if err := wallBudgetStep(budget); err != nil {
				return nil, err
			}
			w, err := walkOf(seg, work)
			if err != nil {
				return nil, err
			}
			if err := requireAnalyticWalk(w, "the wall survey"); err != nil {
				return nil, err
			}
			raw[i] = sideWalk{segmentWalk: w, segs: []int{i}}
		}
		walks, err := coalesceWalksBudget(raw, budget)
		if err != nil {
			return nil, err
		}
		out = append(out, walks)
	}
	return out, nil
}

// recordLoopsBudget names the operation-budget form used by cancellation
// probes and by callers that distinguish profile scanning from other surveys.
func recordLoopsBudget(budget *workBudget, profile ProfileRecord) ([][]sideWalk, error) {
	if err := wallBudgetErr(budget); err != nil {
		return nil, err
	}
	return recordLoops(budget, profile)
}

// revolveLoops resolves the loops into axis coordinates (the U fields carry
// z, the V fields ρ), mirroring buildRevolveLoop.
func revolveLoops(budget *workBudget, rp revolvePayload) ([][]sideWalk, error) {
	// One free-form counter for the whole record, as recordLoops opens.
	work := newFreeformWork()
	var out [][]sideWalk
	loops := append([]LoopRecord{rp.profile.Outer}, rp.profile.Holes...)
	for _, loop := range loops {
		if err := wallBudgetStep(budget); err != nil {
			return nil, err
		}
		raw := make([]sideWalk, len(loop.Segments))
		for i, seg := range loop.Segments {
			if err := wallBudgetStep(budget); err != nil {
				return nil, err
			}
			w, err := walkOf(seg, work)
			if err != nil {
				return nil, err
			}
			if err := requireAnalyticWalk(w, "the survey boundary walk"); err != nil {
				return nil, err
			}
			raw[i] = sideWalk{segmentWalk: rp.ax.walk(w), segs: []int{i}}
		}
		walks, err := coalesceWalksBudget(raw, budget)
		if err != nil {
			return nil, err
		}
		out = append(out, walks)
	}
	return out, nil
}

// walkElem turns one walk into a kernel boundary element — the ONE conversion
// from a walk into survey2d's 2D vocabulary, shared by the wall survey, the
// modify section audit and the clearance kernel's planar trims. The material
// side is irrelevant to ray parity, so an arc's walk sense is not consulted.
//
// The switch is total over walkKind on purpose: a free-form walk has no
// surveyElem to become, and answering false is what keeps it from silently
// converting into the straight line between its endpoints. Its element awaits
// docs/spline-design.md §8's free-form kernels.
func walkElem(w segmentWalk) (surveyElem, bool) {
	switch w.kind {
	case walkCircular:
		return arcElem(w.cU, w.cV, w.radius, w.th0, w.th1, w.closed)
	case walkLine:
		return lineElem(w.startU, w.startV, w.endU, w.endV)
	default:
		return surveyElem{}, false
	}
}

// mirrorElem reflects an element across the axis (y → −y), reversing the
// walk so the material stays on its left.
func mirrorElem(e surveyElem) surveyElem {
	if e.kind == surveyLine {
		m, _ := lineElem(e.bx, -e.by, e.ax, -e.ay)
		return m
	}
	m, _ := arcElem(e.qx, -e.qy, e.rr, -e.th1, -e.th0, e.closed)
	return m
}

// junctionPinch reports whether the material corner between an incoming and
// an outgoing walk tangent is a dihedral within the allowance — a wedge that
// is a wall by the caller's own line, pinched to zero (verification §6:
// δ ≤ α spans at every size an edge starves a ball to; a tangency is the
// knife edge's genuine zero).
func junctionPinch(inU, inV, outU, outV, alpha float64) bool {
	cross := inU*outV - inV*outU
	dot := inU*outU + inV*outV
	turn := math.Atan2(cross, dot) // left turn positive, (−π, π]
	delta := math.Pi - turn        // interior material angle; reflex > π
	return delta <= alpha+survAngTol
}

// prismWall is the spanning-ball reading of a prism: the profile's spanning
// disks lift to balls when their diameter fits the height, the parallel caps
// span whenever a disk of half the height fits the section, and a profile
// corner within the allowance pinches to zero.
func prismWall(budget *workBudget, pp prismPayload, alpha float64) (wallOutcome, error) {
	if err := wallBudgetErr(budget); err != nil {
		return wallOutcome{}, err
	}
	if pp.sectionDelta != 0 {
		// A spanning ball fitted to the RECORDED section is not a proof about a
		// section that section is only within the payload's own displacement of
		// (docs/prism-boolean-design.md §7), and the reading carries no bound to
		// widen. Undecided, which reads Suspect — never a silent pass.
		return wallOutcome{}, nil
	}
	loops, err := recordLoops(budget, pp.profile)
	if err != nil {
		return wallOutcome{}, err
	}
	if err := wallBudgetErr(budget); err != nil {
		return wallOutcome{}, err
	}
	var elems []surveyElem
	var verts [][2]float64
	pinch := false
	for _, loop := range loops {
		single := len(loop) == 1 && loop[0].closed
		for i, w := range loop {
			if err := wallBudgetStep(budget); err != nil {
				return wallOutcome{}, err
			}
			el, ok := walkElem(w.segmentWalk)
			if !ok {
				return wallOutcome{}, nil
			}
			elems = append(elems, el)
			if single {
				continue
			}
			verts = append(verts, [2]float64{w.startU, w.startV})
			prev := loop[(i+len(loop)-1)%len(loop)]
			if junctionPinch(prev.tanOutU, prev.tanOutV, w.tanInU, w.tanInV, alpha) {
				pinch = true
			}
		}
	}
	h := pp.z1 - pp.z0
	k, err := newWallKernelBudget(budget, elems, nil, verts, alpha, 0, false, h)
	if err != nil {
		return wallOutcome{}, err
	}
	out, err := k.runBudget(budget)
	if err != nil {
		return wallOutcome{}, err
	}
	if !out.ok {
		return wallOutcome{}, nil
	}
	best := math.Inf(1)
	bestBound := 0.0
	if pinch {
		best, bestBound = 0, 0
	}
	if out.subTolFar && best != 0 {
		// Same rule as the revolve path: a dropped off-junction
		// sub-tolerance disk could be a real web thinner than the kernel
		// resolves — only an exact zero still decides.
		return wallOutcome{}, nil
	}
	if out.hasSpan && out.span < best {
		best = out.span
		bestBound = math.Max(out.spanBound, out.maxCandBound)
	}
	if out.inradius >= h/2-k.tol && h < best {
		best = h
		bestBound = heightArmBound(pp)
	}
	if math.IsInf(best, 1) {
		return wallOutcome{ok: true}, nil
	}
	if isNonFinite(bestBound) {
		// A candidate this arm relied on could not be bounded: the reading
		// is not one this evaluator can stand behind (see runBudget's own
		// doc comment), so it is undecided rather than published with an
		// unusable bound.
		return wallOutcome{}, nil
	}
	return wallOutcome{reading: &best, bound: bestBound, ok: true}, nil
}

// heightArmBound is prismWall's height-arm bound: the payload's own two axial
// displacements (docs/evaluator-design.md §5), one per end, SUMMED rather
// than maxed — the two ends move independently and in opposite senses for a
// height reading (CLAUDE.md's "every level-derived reading takes it") — plus
// the float subtraction z1 − z0's own rounding, measured exactly against the
// rational difference the same way boolean_body.go's own volume/centroid
// readings measure a held float against its exact rational.
func heightArmBound(pp prismPayload) float64 {
	z0R, z1R := floatRat(pp.z0), floatRat(pp.z1)
	if z0R == nil || z1R == nil {
		return math.Inf(1)
	}
	h := pp.z1 - pp.z0
	subErr := ratAbsDiff(new(big.Rat).Sub(z1R, z0R), h)
	return absSumUpper(pp.z0Delta, pp.z1Delta, subErr)
}

// revolveWall is the spanning-ball reading of a revolved body, computed in
// its meridian section: a full revolution's balls are the disks of the
// section mirrored across the axis (a ball's contacts with surfaces of
// revolution lie in the meridian plane through its center), and a partial
// sweep's are the plain section's disks constrained by the cap wedge —
// whose own two contacts span exactly when the sweep is within the
// allowance, the on-axis cap edge (dihedral = the sweep itself) included.
func revolveWall(budget *workBudget, rp revolvePayload, alpha float64) (wallOutcome, error) {
	if err := wallBudgetErr(budget); err != nil {
		return wallOutcome{}, err
	}
	loops, err := revolveLoops(budget, rp)
	if err != nil {
		return wallOutcome{}, err
	}
	if err := wallBudgetErr(budget); err != nil {
		return wallOutcome{}, err
	}
	dphi := rp.phi1 - rp.phi0
	var elems, containOnly []surveyElem
	var verts [][2]float64
	pinch := false
	axisTouch := false
	for _, loop := range loops {
		n := len(loop)
		single := n == 1 && loop[0].closed
		kinds := make([]wallKind, n)
		for i, w := range loop {
			if err := wallBudgetStep(budget); err != nil {
				return wallOutcome{}, err
			}
			kinds[i] = rp.ax.classify(w.segmentWalk)
		}
		for i, w := range loop {
			if err := wallBudgetStep(budget); err != nil {
				return wallOutcome{}, err
			}
			if kinds[i] == wallAxis {
				axisTouch = true
				if !rp.full {
					if el, ok := walkElem(w.segmentWalk); ok {
						containOnly = append(containOnly, el)
					}
				}
				continue
			}
			el, ok := walkElem(w.segmentWalk)
			if !ok {
				return wallOutcome{}, nil
			}
			elems = append(elems, el)
			if rp.full {
				elems = append(elems, mirrorElem(el))
			}
			if single {
				continue
			}
			if w.startV == 0 || w.endV == 0 {
				axisTouch = true
			}
			verts = append(verts, [2]float64{w.startU, w.startV})
			if rp.full && w.startV > 0 {
				verts = append(verts, [2]float64{w.startU, -w.startV})
			}
			prev := loop[(i+n-1)%n]
			prevAxis := kinds[(i+n-1)%n] == wallAxis
			nextAxis := kinds[(i+1)%n] == wallAxis
			switch {
			case !prevAxis:
				if junctionPinch(prev.tanOutU, prev.tanOutV, w.tanInU, w.tanInV, alpha) {
					pinch = true
				}
			case rp.full:
				// The walk continues as its own mirror image across the axis:
				// the dihedral there is the corner the symmetrized section
				// shows (a cone's apex reads its full apex angle).
				if junctionPinch(-w.tanInU, w.tanInV, w.tanInU, w.tanInV, alpha) {
					pinch = true
				}
			}
			if nextAxis && rp.full {
				if junctionPinch(w.tanOutU, w.tanOutV, -w.tanOutU, w.tanOutV, alpha) {
					pinch = true
				}
			}
		}
	}
	wedgeS := 0.0
	wedgeSpans := false
	if !rp.full {
		// A mid-sweep ball at meridian radius ρ clears each cap HALF-plane
		// by ρ·sin(dphi/2) only while dphi/2 ≤ 90°: past that (a reflex
		// sweep) the perpendicular foot leaves the half-plane, the nearest
		// cap point is its edge — the axis — and the clearance is ρ itself.
		// So the factor saturates at sin(π/2) and never shrinks again;
		// shrinking it past 90° would erase real walls
		// (TestWallReflexSweep pins the hand-checked case).
		wedgeS = math.Sin(math.Min(dphi/2, math.Pi/2))
		wedgeSpans = dphi <= alpha+survAngTol
	}
	k, err := newWallKernelBudget(budget, elems, containOnly, verts, alpha, wedgeS, wedgeSpans, math.Inf(1))
	if err != nil {
		return wallOutcome{}, err
	}
	out, err := k.runBudget(budget)
	if err != nil {
		return wallOutcome{}, err
	}
	if !out.ok {
		return wallOutcome{}, nil
	}
	best := math.Inf(1)
	bestBound := 0.0
	if pinch {
		best, bestBound = 0, 0
	}
	if wedgeSpans && axisTouch {
		// The partial sweep's caps meet along the axis at the sweep angle
		// itself: a dihedral within the allowance, ground to zero.
		best, bestBound = 0, 0
	}
	if out.subTolFar && best != 0 {
		// A dropped off-junction sub-tolerance disk could be a real web
		// thinner than the kernel resolves: only an exact zero (which
		// nothing can undercut) still decides; any other reading — or an
		// absence — is undecided.
		return wallOutcome{}, nil
	}
	if out.hasSpan && out.span < best {
		best = out.span
		bestBound = math.Max(out.spanBound, out.maxCandBound)
	}
	if math.IsInf(best, 1) {
		return wallOutcome{ok: true}, nil
	}
	if isNonFinite(bestBound) {
		return wallOutcome{}, nil
	}
	return wallOutcome{reading: &best, bound: bestBound, ok: true}, nil
}

// facesByRole indexes a body's faces by their own-step feature role.
func facesByRole(b *Body) map[string]*Face {
	step := b.origin.Step
	m := make(map[string]*Face)
	for _, f := range b.Faces() {
		for _, o := range f.origins {
			if o.Step == step {
				m[o.Role] = f
			}
		}
	}
	return m
}

// opposesPull decides the pointwise membership rule of verification §6 over
// a face's normal-component range [m, M] against the unit pull, read as a
// bare float with no allowance: a point opposes when its normal has a
// component against the pull — exactly perpendicular is not opposed — and a
// face whose normals are EXACTLY antiparallel everywhere separates under the
// pull rather than hooking it (the flat base a straight prism pulls off of).
// The carve-out is exact, never a tolerance band: a pull tilted by any real
// angle hooks under the base, however slightly, and §6 lists it. The clamp
// below absorbs only float overshoot past −1 in a unit dot; it never widens
// the exception.
//
// revolveUndercuts is this function's only remaining caller: prismUndercuts,
// cupUndercuts and capBlendUndercuts's receiver-wall loop instead read
// through decidePull's exact-rational sibling (survey_undercut.go), which
// this function's own zero-allowance behaviour matches exactly
// (TestDecidePullMatchesOpposesPullAtZeroAllowance).
func opposesPull(m, M float64) bool {
	if M < -1 {
		M = -1
	}
	return m < 0 && M > -1
}

// trigRange is the exact range of a·cosθ + b·sinθ over [lo, hi].
func trigRange(a, b, lo, hi float64) (float64, float64) {
	mn := math.Min(a*math.Cos(lo)+b*math.Sin(lo), a*math.Cos(hi)+b*math.Sin(hi))
	mx := math.Max(a*math.Cos(lo)+b*math.Sin(lo), a*math.Cos(hi)+b*math.Sin(hi))
	if a == 0 && b == 0 {
		return 0, 0
	}
	star := math.Atan2(b, a)
	for _, cand := range []float64{star, star + math.Pi} {
		for kk := math.Floor((lo-cand)/(2*math.Pi)) * 2 * math.Pi; cand+kk <= hi+1e-12; kk += 2 * math.Pi {
			th := cand + kk
			if th < lo-1e-12 {
				continue
			}
			v := a*math.Cos(th) + b*math.Sin(th)
			mn = math.Min(mn, v)
			mx = math.Max(mx, v)
		}
	}
	return mn, mx
}

// prismUndercuts surveys a prism's faces against the pull, through the exact
// three-valued reader survey_undercut.go shares with cupUndercuts and
// capBlendUndercuts's receiver-wall loop: planar sides and caps carry one
// normal each, cylindrical sides sweep their walk's exact angular range. A
// straddling face sets undecided without discarding a face already proven to
// oppose (docs/verification-design.md §6).
func prismUndercuts(b *Body, pp prismPayload, pull r3.Vec) undercutOutcome {
	if _, ok := pull.Normalize(); !ok {
		return undercutOutcome{}
	}
	m, okM := newPlacedFrameMap(pp)
	if !okM {
		return undercutOutcome{}
	}
	roles := facesByRole(b)
	loops, err := recordLoops(nil, pp.profile)
	if err != nil {
		return undercutOutcome{}
	}
	faces := []*Face{}
	undecided := false
	for li, loop := range loops {
		for _, w := range loop {
			f := roles[fmt.Sprintf("side(%d,%d)", li, w.segs[0])]
			if f == nil {
				return undercutOutcome{}
			}
			verdict, ok := wallNormalDecision(w, m, pull)
			if !listVerdict(&faces, &undecided, f, verdict, ok) {
				return undercutOutcome{}
			}
		}
	}
	for _, cap := range []struct {
		role string
		sign float64
	}{{role: roleCapStart, sign: -1}, {role: roleCapEnd, sign: 1}} {
		f := roles[cap.role]
		if f == nil {
			return undercutOutcome{}
		}
		verdict, ok := capNormalDecision(m, pull, cap.sign)
		if !listVerdict(&faces, &undecided, f, verdict, ok) {
			return undercutOutcome{}
		}
	}
	if undecided && len(faces) == 0 {
		// Keep an entirely undecided result distinct from a proven all-clear.
		faces = nil
	}
	return undercutOutcome{faces: faces, ok: true, undecided: undecided}
}

// revolveUndercuts surveys a revolved body's faces against the pull: each
// wall's normal is n_ρ·radial(φ) + n_z·ŵ over the meridian range its walk
// sweeps and the azimuth range of the sweep — both exact.
func revolveUndercuts(b *Body, rp revolvePayload, pull r3.Vec) undercutOutcome {
	p, ok := pull.Normalize()
	if !ok {
		return undercutOutcome{}
	}
	bas := rp.basis()
	pw := rp.xform.ApplyDir(bas.w).Dot(p)
	c0 := rp.xform.ApplyDir(bas.e0).Dot(p)
	c1 := rp.xform.ApplyDir(bas.e1).Dot(p)
	glo, ghi := sweepExtremes(c0, c1, rp.phi0, rp.phi1, rp.full)
	roles := facesByRole(b)
	loops, err := revolveLoops(nil, rp)
	if err != nil {
		return undercutOutcome{}
	}
	faces := []*Face{}
	for li, loop := range loops {
		for _, w := range loop {
			if rp.ax.classify(w.segmentWalk) == wallAxis {
				continue
			}
			f := roles[fmt.Sprintf("side(%d,%d)", li, w.segs[0])]
			if f == nil {
				return undercutOutcome{}
			}
			mn, mx := math.Inf(1), math.Inf(-1)
			if w.isCircular() {
				sigma := 1.0
				if w.th1 < w.th0 {
					sigma = -1
				}
				lo, hi := math.Min(w.th0, w.th1), math.Max(w.th0, w.th1)
				for _, g := range []float64{glo, ghi} {
					// n·p = σ(cosθ·pw + sinθ·g) over the meridian range.
					a, bb := trigRange(pw, g, lo, hi)
					mn = math.Min(mn, math.Min(sigma*a, sigma*bb))
					mx = math.Max(mx, math.Max(sigma*a, sigma*bb))
				}
			} else {
				l := math.Hypot(w.tanInU, w.tanInV)
				nz := w.tanInV / l
				nr := -w.tanInU / l
				for _, g := range []float64{glo, ghi} {
					v := nz*pw + nr*g
					mn = math.Min(mn, v)
					mx = math.Max(mx, v)
				}
			}
			if opposesPull(mn, mx) {
				faces = append(faces, f)
			}
		}
	}
	if !rp.full {
		sin0, cos0 := math.Sincos(rp.phi0)
		sin1, cos1 := math.Sincos(rp.phi1)
		for _, cap := range []struct {
			role string
			v    float64
		}{
			// A cap's outward normal is the sweep-velocity direction at its
			// own angle — against the sweep on the start cap, along it on
			// the end cap.
			{role: roleCapStart, v: -(c1*cos0 - c0*sin0)},
			{role: roleCapEnd, v: c1*cos1 - c0*sin1},
		} {
			f := roles[cap.role]
			if f == nil {
				return undercutOutcome{}
			}
			if opposesPull(cap.v, cap.v) {
				faces = append(faces, f)
			}
		}
	}
	return undercutOutcome{faces: faces, ok: true}
}

// prismMinRadius is the tightest concave radius over a prism's faces: only
// a side swept from an arc walked against the section's orientation — a
// hole wall, a notch — curves away from the material, and its radius is the
// walk's own.
func prismMinRadius(pp prismPayload) (radiusOutcome, bool) {
	if pp.sectionDelta != 0 {
		// The reading is a radius READ OFF the recorded section, and a payload
		// carrying a section displacement (docs/prism-boolean-design.md §7)
		// denotes a section its own record is only within that displacement of.
		// The survey answers a measurement with no bound of its own, so it leaves
		// the question undecided rather than publishing a radius for the wrong
		// section.
		return radiusOutcome{}, false
	}
	loops, err := recordLoops(nil, pp.profile)
	if err != nil {
		return radiusOutcome{}, false
	}
	recLoops := append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...)
	best := math.Inf(1)
	bestBound := 0.0
	for li, loop := range loops {
		for _, w := range loop {
			if w.isCircular() && w.th1 < w.th0 && w.radius < best {
				best = w.radius
				bestBound = concaveWalkRadiusBound(recLoops[li], w)
			}
		}
	}
	if math.IsInf(best, 1) {
		return radiusOutcome{ok: true}, true
	}
	if isNonFinite(bestBound) {
		return radiusOutcome{}, false
	}
	return radiusOutcome{reading: &best, bound: bestBound, ok: true}, true
}

// concaveWalkRadiusBound is the proven bound on one concave walk's own
// radius: a CircleSeg's radius is the record's own millimetre number
// (extrude.go's walkOf, CircleSeg arm), so it is exact; an ArcSeg's is
// math.Hypot(Start−Center) (extrude.go's ArcSeg arm), so it carries the same
// rational sqrt bracket circularLengthInterval already reads that segment
// kind's radius through — computed here directly rather than through that
// function's length-scaled result. A coalesced walk's own several original
// segments share one center and radius by construction, so its first
// original segment speaks for the whole walk.
func concaveWalkRadiusBound(loop LoopRecord, w sideWalk) float64 {
	if len(w.segs) == 0 {
		return math.Inf(1)
	}
	idx := w.segs[0]
	if idx < 0 || idx >= len(loop.Segments) {
		return math.Inf(1)
	}
	seg, err := normalizeSegment(loop.Segments[idx])
	if err != nil {
		return math.Inf(1)
	}
	arc, ok := seg.(ArcSeg)
	if !ok {
		// A CircleSeg (or anything else this survey ever admits as a
		// concave walk): the record's own millimetre radius, exact.
		return 0
	}
	dx := exactCoordinateDelta(arc.Start.U, arc.Center.U)
	dy := exactCoordinateDelta(arc.Start.V, arc.Center.V)
	r2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
	rLo, rHi := ratSqrtDown(r2), ratSqrtUp(r2)
	if isNonFinite(rLo) || isNonFinite(rHi) {
		return math.Inf(1)
	}
	return math.Max(upRound(w.radius-rLo), upRound(rHi-w.radius))
}

// revolveMinRadius is the tightest concave radius over a revolved body's
// faces, from the two principal directions of a surface of revolution: the
// meridian's own curvature (an arc walked against the orientation — a
// groove's tube), and the parallel circle's, whose radius is ρ/|n_ρ|
// wherever the meridian normal points toward the axis (a hole wall, a
// waist). Both are closed-form over each walk's extent.
func revolveMinRadius(rp revolvePayload) (radiusOutcome, bool) {
	loops, err := revolveLoops(nil, rp)
	if err != nil {
		return radiusOutcome{}, false
	}
	best := math.Inf(1)
	bestBound := 0.0
	found := false
	take := func(v, vBound float64) {
		found = true
		if v < best {
			best = v
			bestBound = vBound
		}
	}
	for _, loop := range loops {
		for _, w := range loop {
			if rp.ax.classify(w.segmentWalk) == wallAxis {
				continue
			}
			if !w.isCircular() {
				// Neither w.tanInU/V nor w.startV/endV carries its own bound
				// (they are the walk's own recorded coordinates, read as
				// exact leaves throughout this survey); the composition
				// below — a Hypot and two quotients — is what this arm's
				// own arithmetic contributes.
				lBS := boundedHypot(w.tanInU, w.tanInV)
				nrBS := boundedQuotient(-w.tanInU, 0, lBS.value, lBS.bound)
				if nrBS.value < -survAngTol {
					minSV := math.Min(w.startV, w.endV)
					vBS := boundedQuotient(minSV, 0, -nrBS.value, nrBS.bound)
					take(vBS.value, vBS.bound)
				}
				continue
			}
			sigma := 1.0
			if w.th1 < w.th0 {
				sigma = -1
			}
			if sigma < 0 {
				take(w.radius, 0) // the meridian's own recorded radius, exact
			}
			lo, hi := math.Min(w.th0, w.th1), math.Max(w.th0, w.th1)
			cands := []float64{lo, hi}
			for _, star := range []float64{math.Pi / 2, -math.Pi / 2} {
				for kk := math.Floor((lo-star)/(2*math.Pi)) * 2 * math.Pi; star+kk <= hi+1e-12; kk += 2 * math.Pi {
					if th := star + kk; th >= lo-1e-12 {
						cands = append(cands, th)
					}
				}
			}
			for _, th := range cands {
				// The sine's own certified enclosure (radianTrigBounds), not
				// math.Sin's undocumented accuracy — the same rule
				// revolveBoundsContext's own sweep extremes take.
				sinBS, _ := radianTrigBounds(th)
				nrBS := boundedMul(exactScalar(sigma), sinBS)
				if nrBS.value >= -survAngTol {
					continue
				}
				rhoBS := boundedAdd(exactScalar(w.cV), boundedMul(exactScalar(w.radius), sinBS))
				negNrBS := measuredScalar(-nrBS.value, nrBS.bound)
				resultBS := boundedQuotient(rhoBS.value, rhoBS.bound, negNrBS.value, negNrBS.bound)
				take(resultBS.value, resultBS.bound)
			}
		}
	}
	if !found {
		return radiusOutcome{ok: true}, true
	}
	if isNonFinite(bestBound) {
		return radiusOutcome{}, false
	}
	return radiusOutcome{reading: &best, bound: bestBound, ok: true}, true
}

// cupWalks resolves one cup region loop into its coalesced walks — the same
// decomposition evalCup's wall build uses.
func cupWalks(loop LoopRecord) ([]sideWalk, error) {
	return cupWalksBudget(nil, loop)
}

func cupWalksBudget(budget *workBudget, loop LoopRecord) ([]sideWalk, error) {
	loops, err := recordLoopsBudget(budget, ProfileRecord{Outer: loop})
	if err != nil {
		return nil, err
	}
	return loops[0], nil
}

// cupWall returns the exact shell-wall theorem of
// docs/payload-verification-design.md §4: an accepted cup is exactly its
// shell thickness unless one of its material junctions is within the caller's
// draft allowance, in which case the closure-under-limits rule makes the
// reading exactly zero. The theorem consumes the payload's morphology, not the
// recipe value: it rebuilds and audits the offset relation before trusting it.
func cupWall(budget *workBudget, cp cupPayload, alpha float64) (wallOutcome, error) {
	finite := func(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }
	isCancellation := func(err error) bool {
		return errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded)
	}
	if err := wallBudgetErr(budget); err != nil {
		return wallOutcome{}, err
	}
	t := cp.thickness
	if !finite(t) || t <= 0 || (cp.sense != Inward && cp.sense != Outward) {
		return wallOutcome{}, nil
	}
	dOuter := cp.zOpen - cp.zOuter
	dCavity := cp.zOpen - cp.zCav
	if !finite(dOuter) || !finite(dCavity) || dOuter == 0 || dCavity == 0 ||
		math.Signbit(dOuter) != math.Signbit(dCavity) || math.Abs(dCavity) >= math.Abs(dOuter) {
		return wallOutcome{}, nil
	}
	openDir := 1.0
	if dOuter < 0 {
		openDir = -1
	}
	if cp.sense == Inward && cp.zCav != cp.zOuter+openDir*t {
		return wallOutcome{}, nil
	}
	if cp.sense == Outward && cp.zOuter != cp.zCav-openDir*t {
		return wallOutcome{}, nil
	}

	oLoops := append([]LoopRecord{cp.outer.Outer}, cp.outer.Holes...)
	cLoops := append([]LoopRecord{cp.cavity.Outer}, cp.cavity.Holes...)
	if len(oLoops) != len(cLoops) {
		return wallOutcome{}, nil
	}
	if oi, err := cp.outer.integralsBudget(budget); err != nil {
		if isCancellation(err) {
			return wallOutcome{}, err
		}
		return wallOutcome{}, nil
	} else if oi.area <= 0 || !finite(oi.area) {
		return wallOutcome{}, nil
	}
	if ci, err := cp.cavity.integralsBudget(budget); err != nil {
		if isCancellation(err) {
			return wallOutcome{}, err
		}
		return wallOutcome{}, nil
	} else if ci.area <= 0 || !finite(ci.area) {
		return wallOutcome{}, nil
	}

	// Inward cups store C = O ⊖ t. Outward cups store O = C ⊕ t. Structural
	// equality is the exact claim: a residual or tolerance match could only
	// guess that the regions correspond.
	offsetMatches := func(orig, want ProfileRecord, sense float64) (bool, error) {
		got, err := offsetProfile(budget, orig, sense, t)
		if err != nil {
			if isCancellation(err) {
				return false, err
			}
			return false, nil
		}
		same, err := profileRecordsEqual(budget, got, want)
		if err != nil {
			if isCancellation(err) {
				return false, err
			}
			return false, nil
		}
		if !same {
			return false, nil
		}
		if err := auditOffsetSectionBudget(budget, orig, got); err != nil {
			if isCancellation(err) {
				return false, err
			}
			return false, nil
		}
		return true, nil
	}
	matches, err := offsetMatches(cp.outer, cp.cavity, 1)
	if err != nil {
		return wallOutcome{}, err
	}
	if cp.sense == Outward {
		matches, err = offsetMatches(cp.cavity, cp.outer, -1)
		if err != nil {
			return wallOutcome{}, err
		}
	}
	if !matches {
		return wallOutcome{}, nil
	}

	hasPinch := func(loops [][]sideWalk) (bool, bool, error) {
		for _, loop := range loops {
			if err := wallBudgetStep(budget); err != nil {
				return false, false, err
			}
			if len(loop) == 0 {
				return false, false, nil
			}
			if len(loop) == 1 && loop[0].closed {
				continue
			}
			for i, w := range loop {
				if err := wallBudgetStep(budget); err != nil {
					return false, false, err
				}
				prev := loop[(i+len(loop)-1)%len(loop)]
				if junctionPinch(prev.tanOutU, prev.tanOutV, w.tanInU, w.tanInV, alpha) {
					return true, true, nil
				}
			}
		}
		return false, true, nil
	}

	outerWalks, err := recordLoopsBudget(budget, cp.outer)
	if err != nil {
		return wallOutcome{}, err
	}
	pinch, ok, err := hasPinch(outerWalks)
	if err != nil {
		return wallOutcome{}, err
	}
	if !ok {
		return wallOutcome{}, nil
	} else if pinch {
		zero := 0.0
		return wallOutcome{reading: &zero, ok: true}, nil
	}

	// The cavity boundary is a void skin, so reverse every recorded loop to
	// restore the same material-left walk convention junctionPinch expects.
	var cavityWalks [][]sideWalk
	for _, loop := range cLoops {
		reversed, err := reverseLoopRecordBudget(budget, loop)
		if err != nil {
			return wallOutcome{}, err
		}
		walks, err := cupWalksBudget(budget, reversed)
		if err != nil {
			return wallOutcome{}, err
		}
		cavityWalks = append(cavityWalks, walks)
	}
	pinch, ok, err = hasPinch(cavityWalks)
	if err != nil {
		return wallOutcome{}, err
	}
	if !ok {
		return wallOutcome{}, nil
	} else if pinch {
		zero := 0.0
		return wallOutcome{reading: &zero, ok: true}, nil
	}

	return wallOutcome{reading: &t, bound: cp.thicknessDelta, ok: true}, nil
}

// cupUndercuts surveys a cup's faces against the pull (docs/modify-design.md
// D2): the outer walls over EVERY loop of region O (role side(i,j)), the cavity
// walls over every loop of the reversed region C (role shellSide(i,j)) — each
// read by the same exact three-valued reader the prism uses
// (survey_undercut.go), so a post wall is surveyed like any other — and the
// planar faces (the kept cap, the pocket floor, one rim per loop). The cavity
// being a pocket, the pocket floor (shellCap) and the rims face the open end
// and the kept cap (capStart) faces away from it, so their outward normals
// are ±N by which side of the outer floor the open end lies on.
func cupUndercuts(b *Body, cp cupPayload, pull r3.Vec) undercutOutcome {
	if _, ok := pull.Normalize(); !ok {
		return undercutOutcome{}
	}
	base := cp.basePrism()
	m, okM := newPlacedFrameMap(base)
	if !okM {
		return undercutOutcome{}
	}
	roles := facesByRole(b)

	faces := []*Face{}
	undecided := false
	survey := func(loop LoopRecord, role string) bool {
		walks, err := cupWalks(loop)
		if err != nil {
			return false
		}
		for _, w := range walks {
			f := roles[fmt.Sprintf(role, w.segs[0])]
			if f == nil {
				return false
			}
			verdict, ok := wallNormalDecision(w, m, pull)
			if !listVerdict(&faces, &undecided, f, verdict, ok) {
				return false
			}
		}
		return true
	}
	oLoops := append([]LoopRecord{cp.outer.Outer}, cp.outer.Holes...)
	cLoops := append([]LoopRecord{cp.cavity.Outer}, cp.cavity.Holes...)
	for i, loop := range oLoops {
		if !survey(loop, fmt.Sprintf("side(%d,%%d)", i)) {
			return undercutOutcome{}
		}
	}
	for i, loop := range cLoops {
		crev, err := reverseLoopRecord(loop)
		if err != nil {
			return undercutOutcome{}
		}
		if !survey(crev, fmt.Sprintf("shellSide(%d,%%d)", i)) {
			return undercutOutcome{}
		}
	}

	// The planar faces. sOpen is +1 when the open end lies above the outer
	// floor: the kept cap then faces −N and the pocket floor and every rim
	// face +N. All rims share the open-end plane, so each has the same normal.
	sOpen := -1.0
	if cp.zOpen > cp.zOuter {
		sOpen = 1
	}
	caps := []struct {
		role string
		sign float64
	}{
		{role: roleCapStart, sign: -sOpen},
		{role: "shellCap", sign: sOpen},
	}
	for i := range oLoops {
		caps = append(caps, struct {
			role string
			sign float64
		}{role: fmt.Sprintf("rim(%d)", i), sign: sOpen})
	}
	for _, c := range caps {
		f := roles[c.role]
		if f == nil {
			return undercutOutcome{}
		}
		verdict, ok := capNormalDecision(m, pull, c.sign)
		if !listVerdict(&faces, &undecided, f, verdict, ok) {
			return undercutOutcome{}
		}
	}
	if undecided && len(faces) == 0 {
		// Keep an entirely undecided result distinct from a proven all-clear.
		faces = nil
	}
	return undercutOutcome{faces: faces, ok: true, undecided: undecided}
}

// cupMinRadius is the tightest concave radius over a cup's faces (D3): the same
// walk the prism runs, over every loop of the outer region O and every loop of
// the cavity region C read reversed. A wall that curves away from the material
// — the cavity's reversed outer (a pocket wall), every hole of O (a tunnel) —
// contributes its concave radius; a wall that curves toward it — the outer
// region's own convex rounds, a post's outer cylinder (the reversed cavity
// hole) — is not a concave feature and rightly does not appear. Each loop is
// placed with the same walk sense evalCup builds it in, so prismMinRadius reads
// concavity off the walk direction, exactly as on a prism. The sharp concave
// edge where a wall meets the floor carries no radius — the survey reads faces'
// principal radii, not edges.
func cupMinRadius(cp cupPayload) (radiusOutcome, bool) {
	profile := ProfileRecord{Outer: cp.outer.Outer}
	profile.Holes = append(profile.Holes, cp.outer.Holes...)
	cLoops := append([]LoopRecord{cp.cavity.Outer}, cp.cavity.Holes...)
	for _, loop := range cLoops {
		crev, err := reverseLoopRecord(loop)
		if err != nil {
			return radiusOutcome{}, false
		}
		profile.Holes = append(profile.Holes, crev)
	}
	return prismMinRadius(prismPayload{profile: profile})
}

// runSurveys answers the asked opt-in questions on one proven-solid body,
// filling the report fields and returning one diagnostic per non-Sound
// outcome: DiagWallTooThin / DiagUndercut (Violating) when a stated spec is
// proven to fail, and the per-survey DiagUndecided* (Suspect) when an asked
// question is undecided or a stated spec is straddled (verification §1.1/§6).
// A Sound survey emits nothing. Scalar readings are closed-form, each Exact
// only where its own arm proved a zero bound — a pinch reading or a sweep
// height with no axial displacement — and Approximate with the bound its own
// arithmetic derived otherwise; a cap-blend undercut survey can instead use a
// bounded normal range and leave an individual patch undecided.
func runSurveys(budget *workBudget, br *BodyReport, cfg verifyConfig) ([]Diagnostic, error) {
	if err := wallBudgetErr(budget); err != nil {
		return nil, err
	}
	var diags []Diagnostic
	b := br.Body

	if cfg.wall != nil {
		out := wallOutcome{}
		var err error
		switch pl := b.payload.(type) {
		case prismPayload:
			out, err = prismWall(budget, pl, cfg.allowRad)
		case revolvePayload:
			out, err = revolveWall(budget, pl, cfg.allowRad)
		case cupPayload:
			out, err = cupWall(budget, pl, cfg.allowRad)
		case capBlendPayload:
			// DX9 (docs/modify-reach-design.md Table DX): a cap blend is not
			// one constant section at one height, so the existing 2D
			// spanning-disk proof does not decide it. A deliberate evaluator
			// limit — the asked reading is Suspect, never fabricated.
		case facetedPayload:
			out.reason = surveyFacetedUnsupported
		}
		if err != nil {
			return nil, err
		}
		switch {
		case !out.ok:
			diags = append(diags, surveyRefusalDiagnostic(
				b,
				out.reason,
				DiagUndecidedWall,
				"the wall survey could neither answer nor prove no wall exists",
				"facetedPayload wall survey support is not implemented; use an analytic body or wait for faceted wall support",
			))
		case out.reading != nil:
			m := lengthMeasurement(*out.reading, out.bound)
			br.MinWallThickness = &m
			tool := cfg.wall.tool
			switch intervalVerdict(*out.reading, out.bound, cfg.toolMM) {
			case -1:
				obs := m
				diags = append(diags, Diagnostic{
					Code:     DiagWallTooThin,
					Status:   Violating,
					Body:     b,
					Reading:  ReadingWall,
					Observed: &obs,
					Required: &tool,
					Message:  "the minimum wall thickness is proven below the tool",
				})
			case 0:
				obs := m
				diags = append(diags, Diagnostic{
					Code:     DiagUndecidedWall,
					Status:   Suspect,
					Body:     b,
					Reading:  ReadingWall,
					Observed: &obs,
					Required: &tool,
					Message:  "the minimum wall thickness interval straddles the tool",
				})
			}
		}
		if err := wallBudgetErr(budget); err != nil {
			return nil, err
		}
	}

	if cfg.pull != nil {
		out := undercutOutcome{}
		switch pl := b.payload.(type) {
		case prismPayload:
			out = prismUndercuts(b, pl, *cfg.pull)
		case revolvePayload:
			out = revolveUndercuts(b, pl, *cfg.pull)
		case cupPayload:
			out = cupUndercuts(b, pl, *cfg.pull)
		case capBlendPayload:
			out = capBlendUndercuts(b, pl, *cfg.pull)
		case facetedPayload:
			out.reason = surveyFacetedUnsupported
		}
		if out.ok {
			br.Undercuts = out.faces
			if len(out.faces) > 0 {
				// An undercut is a predicate, not a scalar; the report's
				// Undercuts slice already lists the faces, so the pair emits
				// one DiagUndercut naming the body.
				diags = append(diags, Diagnostic{
					Code:    DiagUndercut,
					Status:  Violating,
					Body:    b,
					Reading: ReadingNone,
					Message: "a face is a proven undercut against the pull",
				})
			}
		}
		if !out.ok || out.undecided {
			diags = append(diags, surveyRefusalDiagnostic(
				b,
				out.reason,
				DiagUndecidedUndercut,
				"the pull survey could neither prove nor exclude an undercut",
				"facetedPayload pull survey support is not implemented; use an analytic body or wait for faceted undercut support",
			))
		}
		if err := wallBudgetErr(budget); err != nil {
			return nil, err
		}
	}

	if cfg.minRadius {
		out := radiusOutcome{}
		ok := false
		switch pl := b.payload.(type) {
		case prismPayload:
			out, ok = prismMinRadius(pl)
		case revolvePayload:
			out, ok = revolveMinRadius(pl)
		case cupPayload:
			out, ok = cupMinRadius(pl)
		case capBlendPayload:
			out, ok = capBlendMinRadius(b, pl)
		case facetedPayload:
			out.reason = surveyFacetedUnsupported
		}
		switch {
		case !ok || !out.ok:
			diags = append(diags, surveyRefusalDiagnostic(
				b,
				out.reason,
				DiagUndecidedMinRadius,
				"the concave-radius survey could neither measure nor exclude a concave feature",
				"facetedPayload concave-radius survey support is not implemented; use an analytic body or wait for faceted radius support",
			))
		case out.reading != nil:
			m := lengthMeasurement(*out.reading, out.bound)
			br.MinRadius = &m
		}
		if err := wallBudgetErr(budget); err != nil {
			return nil, err
		}
	}

	return diags, nil
}

func surveyRefusalDiagnostic(
	body *Body,
	reason surveyReason,
	undecidedCode DiagnosticCode,
	undecidedMessage string,
	unsupportedMessage string,
) Diagnostic {
	code, message := undecidedCode, undecidedMessage
	if reason == surveyFacetedUnsupported {
		code, message = DiagUnsupportedSurveyPayload, unsupportedMessage
	}
	return Diagnostic{
		Code:    code,
		Status:  Suspect,
		Body:    body,
		Reading: ReadingNone,
		Message: message,
	}
}

// lengthMeasurement wraps a survey reading with the PROVEN bound its own arm
// computed: Exact only where that arm's own arithmetic proved bound zero,
// Approximate otherwise — never asserted (docs/verification-design.md §6).
func lengthMeasurement(mm, bound float64) Measurement {
	return Measurement{Value: units.Millimeters(mm), Exactness: exactnessOf(bound), Bound: units.Millimeters(bound)}
}

// intervalVerdict decides a stated spec on the proven interval
// [value − bound, value + bound] against the tool (verification §6): −1 is
// proven thin (every admissible thickness under the tool), +1 is met
// (exactly tool-thick is not thinner), 0 is a straddle — undecided.
func intervalVerdict(value, bound, tool float64) int {
	if value+bound < tool {
		return -1
	}
	if value-bound >= tool {
		return 1
	}
	return 0
}
