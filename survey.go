package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the increment-5 analytic survey layer of
// docs/evaluator-design.md §10 and docs/verification-design.md §6: the wall,
// undercut and minimum-radius questions answered outright on the analytic
// prism and revolve bodies. Each survey reads the evaluator's own payload —
// the recorded 2D region a prism lifts or a revolve sweeps — because the
// spanning-ball and curvature readings are closed-form facts of that section:
// a prism's inscribed balls are its profile's inscribed disks with the height
// as the vertical fit, and a solid of revolution's are the disks of its
// meridian section (mirrored across the axis for a full turn, wedge-bounded
// by the caps for a partial one). Undercuts are decided per face from the
// exact normal range each walk sweeps. A body this file cannot decide — a
// payload no shipped feature builds — leaves every asked question undecided,
// which reads Suspect, never a silent pass.

// wallOutcome is one body's wall reading: ok=false is an undecided survey;
// reading == nil (with ok) is the proven determination that no wall exists.
type wallOutcome struct {
	reading *float64 // millimetres
	ok      bool
}

// undercutOutcome is one body's undercut listing: ok=false is undecided;
// faces is non-nil when decided, and empty is the proven all-clear.
type undercutOutcome struct {
	faces []*Face
	ok    bool
}

// radiusOutcome is one body's tightest concave radius: ok=false is
// undecided; reading == nil (with ok) is the proven all-convex answer.
type radiusOutcome struct {
	reading *float64 // millimetres
	ok      bool
}

// recordLoops resolves the recorded profile into coalesced walk loops,
// exactly as the prism evaluator builds its side faces — the surveys must
// see the same face decomposition the topology carries.
func recordLoops(profile ProfileRecord) ([][]sideWalk, error) {
	var out [][]sideWalk
	for _, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
		raw := make([]sideWalk, len(loop.Segments))
		for i, seg := range loop.Segments {
			w, err := walkOf(seg)
			if err != nil {
				return nil, err
			}
			raw[i] = sideWalk{segmentWalk: w, segs: []int{i}}
		}
		out = append(out, coalesceWalks(raw))
	}
	return out, nil
}

// revolveLoops resolves the loops into axis coordinates (the U fields carry
// z, the V fields ρ), mirroring buildRevolveLoop.
func revolveLoops(rp revolvePayload) ([][]sideWalk, error) {
	var out [][]sideWalk
	loops := append([]LoopRecord{rp.profile.Outer}, rp.profile.Holes...)
	for _, loop := range loops {
		raw := make([]sideWalk, len(loop.Segments))
		for i, seg := range loop.Segments {
			w, err := walkOf(seg)
			if err != nil {
				return nil, err
			}
			raw[i] = sideWalk{segmentWalk: rp.ax.walk(w), segs: []int{i}}
		}
		out = append(out, coalesceWalks(raw))
	}
	return out, nil
}

// walkElem turns one walk into a kernel boundary element.
func walkElem(w segmentWalk) (surveyElem, bool) {
	if w.circular {
		return arcElem(w.cU, w.cV, w.radius, w.th0, w.th1, w.closed)
	}
	return lineElem(w.startU, w.startV, w.endU, w.endV)
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
func prismWall(pp prismPayload, alpha float64) wallOutcome {
	loops, err := recordLoops(pp.profile)
	if err != nil {
		return wallOutcome{}
	}
	var elems []surveyElem
	var verts [][2]float64
	pinch := false
	for _, loop := range loops {
		single := len(loop) == 1 && loop[0].closed
		for i, w := range loop {
			el, ok := walkElem(w.segmentWalk)
			if !ok {
				return wallOutcome{}
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
	k := newWallKernel(elems, nil, verts, alpha, 0, false, h)
	out := k.run()
	if !out.ok {
		return wallOutcome{}
	}
	best := math.Inf(1)
	if pinch {
		best = 0
	}
	if out.subTolFar && best != 0 {
		// Same rule as the revolve path: a dropped off-junction
		// sub-tolerance disk could be a real web thinner than the kernel
		// resolves — only an exact zero still decides.
		return wallOutcome{}
	}
	if out.hasSpan && out.span < best {
		best = out.span
	}
	if out.inradius >= h/2-k.tol && h < best {
		best = h
	}
	if math.IsInf(best, 1) {
		return wallOutcome{ok: true}
	}
	return wallOutcome{reading: &best, ok: true}
}

// revolveWall is the spanning-ball reading of a revolved body, computed in
// its meridian section: a full revolution's balls are the disks of the
// section mirrored across the axis (a ball's contacts with surfaces of
// revolution lie in the meridian plane through its center), and a partial
// sweep's are the plain section's disks constrained by the cap wedge —
// whose own two contacts span exactly when the sweep is within the
// allowance, the on-axis cap edge (dihedral = the sweep itself) included.
func revolveWall(rp revolvePayload, alpha float64) wallOutcome {
	loops, err := revolveLoops(rp)
	if err != nil {
		return wallOutcome{}
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
			kinds[i] = rp.ax.classify(w.segmentWalk)
		}
		for i, w := range loop {
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
				return wallOutcome{}
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
	k := newWallKernel(elems, containOnly, verts, alpha, wedgeS, wedgeSpans, math.Inf(1))
	out := k.run()
	if !out.ok {
		return wallOutcome{}
	}
	best := math.Inf(1)
	if pinch {
		best = 0
	}
	if wedgeSpans && axisTouch {
		// The partial sweep's caps meet along the axis at the sweep angle
		// itself: a dihedral within the allowance, ground to zero.
		best = 0
	}
	if out.subTolFar && best != 0 {
		// A dropped off-junction sub-tolerance disk could be a real web
		// thinner than the kernel resolves: only an exact zero (which
		// nothing can undercut) still decides; any other reading — or an
		// absence — is undecided.
		return wallOutcome{}
	}
	if out.hasSpan && out.span < best {
		best = out.span
	}
	if math.IsInf(best, 1) {
		return wallOutcome{ok: true}
	}
	return wallOutcome{reading: &best, ok: true}
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
// a face's exact normal-component range [m, M] against the unit pull: a
// point opposes when its normal has a component against the pull — exactly
// perpendicular is not opposed — and a face whose normals are EXACTLY
// antiparallel everywhere separates under the pull rather than hooking it
// (the flat base a straight prism pulls off of). The carve-out is exact,
// never a tolerance band: a pull tilted by any real angle hooks under the
// base, however slightly, and §6 lists it. The clamp below absorbs only
// float overshoot past −1 in a unit dot; it never widens the exception.
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

// wallNormalRange is the exact range of a side wall's outward-normal component
// against the pull, given the pull's in-plane components (du, dv): a planar
// wall carries one normal (its walk tangent rotated), a cylindrical wall sweeps
// its walk's exact angular range. The walk's own sense decides the material
// side, so it serves an outer wall (counter-clockwise) and a cavity wall
// (clockwise, the reversed loop) alike.
func wallNormalRange(w sideWalk, du, dv float64) (float64, float64) {
	if !w.circular {
		l := math.Hypot(w.tanInU, w.tanInV)
		v := (w.tanInV*du - w.tanInU*dv) / l
		return v, v
	}
	sigma := 1.0
	if w.th1 < w.th0 {
		sigma = -1
	}
	lo, hi := math.Min(w.th0, w.th1), math.Max(w.th0, w.th1)
	mn, mx := trigRange(du, dv, lo, hi)
	mn, mx = sigma*mn, sigma*mx
	if mn > mx {
		mn, mx = mx, mn
	}
	return mn, mx
}

// prismUndercuts surveys a prism's faces against the pull: planar sides and
// caps carry one normal each, cylindrical sides sweep their walk's exact
// angular range.
func prismUndercuts(b *Body, pp prismPayload, pull r3.Vec) undercutOutcome {
	p, ok := pull.Normalize()
	if !ok {
		return undercutOutcome{}
	}
	du := pp.dir(1, 0, 0).Dot(p)
	dv := pp.dir(0, 1, 0).Dot(p)
	dn := pp.dir(0, 0, 1).Dot(p)
	roles := facesByRole(b)
	loops, err := recordLoops(pp.profile)
	if err != nil {
		return undercutOutcome{}
	}
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
	return undercutOutcome{faces: faces, ok: true}
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
	loops, err := revolveLoops(rp)
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
			if w.circular {
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
	loops, err := recordLoops(pp.profile)
	if err != nil {
		return radiusOutcome{}, false
	}
	best := math.Inf(1)
	for _, loop := range loops {
		for _, w := range loop {
			if w.circular && w.th1 < w.th0 && w.radius < best {
				best = w.radius
			}
		}
	}
	if math.IsInf(best, 1) {
		return radiusOutcome{ok: true}, true
	}
	return radiusOutcome{reading: &best, ok: true}, true
}

// revolveMinRadius is the tightest concave radius over a revolved body's
// faces, from the two principal directions of a surface of revolution: the
// meridian's own curvature (an arc walked against the orientation — a
// groove's tube), and the parallel circle's, whose radius is ρ/|n_ρ|
// wherever the meridian normal points toward the axis (a hole wall, a
// waist). Both are closed-form over each walk's extent.
func revolveMinRadius(rp revolvePayload) (radiusOutcome, bool) {
	loops, err := revolveLoops(rp)
	if err != nil {
		return radiusOutcome{}, false
	}
	best := math.Inf(1)
	found := false
	take := func(v float64) {
		found = true
		if v < best {
			best = v
		}
	}
	for _, loop := range loops {
		for _, w := range loop {
			if rp.ax.classify(w.segmentWalk) == wallAxis {
				continue
			}
			if !w.circular {
				l := math.Hypot(w.tanInU, w.tanInV)
				nr := -w.tanInU / l
				if nr < -survAngTol {
					take(math.Min(w.startV, w.endV) / -nr)
				}
				continue
			}
			sigma := 1.0
			if w.th1 < w.th0 {
				sigma = -1
			}
			if sigma < 0 {
				take(w.radius) // the meridian itself curves away from material
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
				nr := sigma * math.Sin(th)
				if nr >= -survAngTol {
					continue
				}
				rho := w.cV + w.radius*math.Sin(th)
				take(rho / -nr)
			}
		}
	}
	if !found {
		return radiusOutcome{ok: true}, true
	}
	return radiusOutcome{reading: &best, ok: true}, true
}

// cupWalks resolves one cup region loop into its coalesced walks — the same
// decomposition evalCup's wall build uses.
func cupWalks(loop LoopRecord) ([]sideWalk, error) {
	loops, err := recordLoops(ProfileRecord{Outer: loop})
	if err != nil {
		return nil, err
	}
	return loops[0], nil
}

// cupUndercuts surveys a cup's faces against the pull (docs/modify-design.md
// D2): the outer walls over EVERY loop of region O (role side(i,j)), the cavity
// walls over every loop of the reversed region C (role shellSide(i,j)) — each
// read by the same exact normal ranges the prism uses, so a post wall is
// surveyed like any other — and the planar faces (the kept cap, the pocket
// floor, one rim per loop). The cavity being a pocket, the pocket floor
// (shellCap) and the rims face the open end and the kept cap (capStart) faces
// away from it, so their outward normals are ±N by which side of the outer
// floor the open end lies on.
func cupUndercuts(b *Body, cp cupPayload, pull r3.Vec) undercutOutcome {
	p, ok := pull.Normalize()
	if !ok {
		return undercutOutcome{}
	}
	base := cp.prismLike(0, 0)
	du := base.dir(1, 0, 0).Dot(p)
	dv := base.dir(0, 1, 0).Dot(p)
	dn := base.dir(0, 0, 1).Dot(p)
	roles := facesByRole(b)

	faces := []*Face{}
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
			mn, mx := wallNormalRange(w, du, dv)
			if opposesPull(mn, mx) {
				faces = append(faces, f)
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
		v    float64
	}{
		{role: roleCapStart, v: -sOpen * dn},
		{role: "shellCap", v: sOpen * dn},
	}
	for i := range oLoops {
		caps = append(caps, struct {
			role string
			v    float64
		}{role: fmt.Sprintf("rim(%d)", i), v: sOpen * dn})
	}
	for _, c := range caps {
		f := roles[c.role]
		if f == nil {
			return undercutOutcome{}
		}
		if opposesPull(c.v, c.v) {
			faces = append(faces, f)
		}
	}
	return undercutOutcome{faces: faces, ok: true}
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
// filling the report fields and returning the spec verdicts: violating when
// a stated spec is proven to fail, suspect when an asked question is
// undecided or a stated spec is straddled (verification §6). Every reading
// this evaluator produces is closed-form — Exact, zero bound.
func runSurveys(br *BodyReport, cfg verifyConfig) (bool, bool) {
	violating := false
	suspect := false
	b := br.Body

	if cfg.wall != nil {
		out := wallOutcome{}
		switch pl := b.payload.(type) {
		case prismPayload:
			out = prismWall(pl, cfg.allowRad)
		case revolvePayload:
			out = revolveWall(pl, cfg.allowRad)
		}
		if !out.ok {
			suspect = true
		} else if out.reading != nil {
			m := exactLengthMeasurement(*out.reading)
			br.MinWallThickness = &m
			switch intervalVerdict(*out.reading, 0, cfg.toolMM) {
			case -1:
				violating = true
			case 0:
				suspect = true
			}
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
		}
		if !out.ok {
			suspect = true
		} else {
			br.Undercuts = out.faces
			if len(out.faces) > 0 {
				violating = true
			}
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
		}
		if !ok || !out.ok {
			suspect = true
		} else if out.reading != nil {
			m := exactLengthMeasurement(*out.reading)
			br.MinRadius = &m
		}
	}

	return violating, suspect
}

// exactLengthMeasurement wraps a closed-form millimetre reading.
func exactLengthMeasurement(mm float64) Measurement {
	return Measurement{Value: units.Millimeters(mm), Exactness: Exact, Bound: units.Millimeters(0)}
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
