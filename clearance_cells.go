package decad

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// This file is the candidate enumeration of docs/clearance-design.md §3 and
// the face-pair table of §4: six stationarity tiers over unordered boundary
// pairs, candidates computed on the unbounded carriers and admitted only when
// both feet lie within their faces' trims. Three of the five surfaces are
// constant offsets of a spine, so their cells reduce to spine-pair criticals
// (closed form, or P4/P8 certified brackets) with the four offset
// combinations per critical; the plane row is elementary; cone-involved
// pairs and spindle-torus pairs are off PR 1's solvable set and contribute a
// coarse conservative enclosure instead (§8 — proven disjoint with a wide
// honest row when even the coarse lower bound clears zero, undecided when it
// does not). A discarded candidate is a candidate whose feet provably leave
// the trims — a lower tier holds its minimum; a candidate whose admission is
// in doubt is kept for the lower bound and never counted toward exactness.

// gapContrib is one contribution to a pair's distance interval: a proven
// [lo, hi] with representative feet, hi = +Inf for a lower-bound-only
// (straddle-admitted or unresolvable) contribution.
type gapContrib struct {
	lo, hi float64
	exact  bool
	pa, pb r3.Vec
}

// cellSink accumulates contributions and the undecidable findings.
type cellSink struct {
	contribs []gapContrib
	// unsure is set when a cell meets a question it cannot decide: an
	// admitted-or-ambiguous carrier crossing, an uncertified contact, an
	// equality where a branch demands strictness (§4: equality routes to §6,
	// and PR 1 certifies only the coplanar plane pair).
	unsure bool
}

// candidate folds an admission state into a contribution: rejected feet are
// discarded (a lower tier holds the minimum), a straddle keeps only the
// lower bound, and a near-zero value that is not cleanly rejected is a
// possible contact — undecided.
func (s *cellSink) candidate(k *pairKernel, admit int, lo, hi float64, exact bool, pa, pb r3.Vec) {
	if admit == -1 {
		return
	}
	if lo <= k.tol {
		s.unsure = true
		return
	}
	if admit == 0 {
		s.contribs = append(s.contribs, gapContrib{lo: lo, hi: math.Inf(1)})
		return
	}
	s.contribs = append(s.contribs, gapContrib{lo: lo, hi: hi, exact: exact, pa: pa, pb: pb})
}

// loOnly contributes a bare proven lower bound.
func (s *cellSink) loOnly(lo float64) {
	s.contribs = append(s.contribs, gapContrib{lo: math.Max(0, lo), hi: math.Inf(1)})
}

// coarse contributes a conservative enclosure for a pair no shipped cell can
// solve: the boxes' distance below, the closest admitted witness pair above
// (§5 — enclosure distance never exceeds true distance, a witness is always
// an upper bound).
func (s *cellSink) coarse(boxA, boxB [2]r3.Vec, witA, witB []r3.Vec) {
	lo := clrBoxDist(boxA, boxB)
	hi := math.Inf(1)
	var pa, pb r3.Vec
	for _, wa := range witA {
		for _, wb := range witB {
			if d := wa.Sub(wb).Len(); d < hi {
				hi, pa, pb = d, wa, wb
			}
		}
	}
	s.contribs = append(s.contribs, gapContrib{lo: math.Max(0, lo), hi: hi, pa: pa, pb: pb})
}

// enumerate runs every tier over the pair.
func (k *pairKernel) enumerate() *cellSink {
	sink := &cellSink{}
	for _, fa := range k.a.faces {
		for _, fb := range k.b.faces {
			k.ffCell(fa, fb, sink)
		}
	}
	for _, fa := range k.a.faces {
		for _, eb := range k.b.edges {
			k.feCell(fa, eb, sink)
		}
	}
	for _, fb := range k.b.faces {
		for _, ea := range k.a.edges {
			k.feCell(fb, ea, sink)
		}
	}
	for _, ea := range k.a.edges {
		for _, eb := range k.b.edges {
			k.eeCell(ea, eb, sink)
		}
	}
	for _, va := range k.a.verts {
		k.vertexTier(va, k.b, sink)
	}
	for _, vb := range k.b.verts {
		k.vertexTier(vb, k.a, sink)
	}
	for _, va := range k.a.verts {
		for _, vb := range k.b.verts {
			d := va.Sub(vb).Len()
			sink.candidate(k, 1, d, d, true, va, vb)
		}
	}
	return sink
}

// ffCell dispatches one face pair through the §4 table.
func (k *pairKernel) ffCell(f, g *cFace, sink *cellSink) {
	if g.kind < f.kind {
		f, g = g, f
	}
	switch {
	case f.kind == ckPlane && g.kind == ckPlane:
		k.planePlane(f, g, sink)
	case f.kind == ckPlane && g.kind == ckCylinder:
		k.planeCylinder(f, g, sink)
	case f.kind == ckPlane && g.kind == ckCone:
		k.planeCone(f, g, sink)
	case f.kind == ckPlane && g.kind == ckSphere:
		k.planeSphere(f, g, sink)
	case f.kind == ckPlane && g.kind == ckTorus:
		k.planeTorus(f, g, sink)
	case f.kind == ckCone || g.kind == ckCone:
		// The genuinely iterative cells (§4): cone × sphere stays closed
		// form, everything else cone-involved is the PR 2 BB path.
		if f.kind == ckSphere || g.kind == ckSphere {
			k.coneSphere(f, g, sink)
			return
		}
		sink.coarse(f.box, g.box, f.wit, g.wit)
	case (f.kind == ckTorus && f.spindle) || (g.kind == ckTorus && g.spindle):
		// A Minor ≥ Major torus leaves the polynomial path (§4): the BB
		// downgrade is PR 2, so the pair is coarse here.
		sink.coarse(f.box, g.box, f.wit, g.wit)
	default:
		k.offsetPair(f, g, sink)
	}
}

// spineCrit is one spine-pair critical: a proven value interval and the
// spine feet it is attained at.
type spineCrit struct {
	lo, hi float64
	exact  bool
	fa, fb r3.Vec // feet on f's and g's spines
}

// spineOffset is a face's spine offset radius.
func spineOffset(f *cFace) float64 { return f.radius }

// spineOf reports the spine kind: 0 point, 1 line, 2 circle.
func spineOf(f *cFace) int {
	switch f.kind {
	case ckSphere:
		return 0
	case ckCylinder:
		return 1
	default:
		return 2
	}
}

// offsetPair is the spine-offset reduction of §4 for cylinder/sphere/torus
// pairs: enumerate the spine-pair criticals, emit every offset combination
// per critical, then decide whether a carrier crossing is excluded — by the
// strict exterior branch, by a certified containment (§4's d_sup list: an
// inner point spine, parallel cylinder axes, coaxial spines), or by face-box
// separation; otherwise the pair is undecided.
func (k *pairKernel) offsetPair(f, g *cFace, sink *cellSink) {
	crits, ok := k.spineCriticals(f, g)
	if !ok {
		sink.coarse(f.box, g.box, f.wit, g.wit)
		return
	}
	minLo, maxHi := math.Inf(1), math.Inf(-1)
	for _, c := range crits {
		minLo = math.Min(minLo, c.lo)
		maxHi = math.Max(maxHi, c.hi)
		k.emitOffsetCombos(sink, f, g, c)
	}
	rf, rg := spineOffset(f), spineOffset(g)
	if minLo > rf+rg+k.tol {
		return // strict exterior: the carriers never meet (§4)
	}
	if k.certifiedContainment(f, g, minLo, maxHi) {
		return
	}
	if clrBoxDist(f.box, g.box) > k.tol {
		return // the trimmed faces provably cannot touch
	}
	sink.unsure = true
}

// certifiedContainment is §4's nested-branch certification, restricted to
// the doc's list: an inner POINT spine has d_sup = d trivially (the sup over
// one point is that point's distance — minLo), and parallel cylinder axes
// and coaxial spines have constant spine distance (minLo == maxHi). Any
// other configuration never borrows this branch. Strict inequalities
// throughout — equality is a tangency and routes to §6.
func (k *pairKernel) certifiedContainment(f, g *cFace, minLo, maxHi float64) bool {
	rf, rg := spineOffset(f), spineOffset(g)
	constD := k.constantSpineDist(f, g)
	fSup, gSup := math.Inf(1), math.Inf(1)
	if spineOf(f) == 0 {
		fSup = minLo
	} else if constD {
		fSup = maxHi
	}
	if spineOf(g) == 0 {
		gSup = minLo
	} else if constD {
		gSup = maxHi
	}
	if rg > fSup+rf+k.tol {
		return true // f's carrier strictly inside g's
	}
	return rf > gSup+rg+k.tol
}

// constantSpineDist reports the configurations whose spine distance is
// provably constant: parallel cylinder axes, coaxial circle spines, a circle
// spine coaxial with a cylinder axis, and any point-spine pair.
func (k *pairKernel) constantSpineDist(f, g *cFace) bool {
	sf, sg := spineOf(f), spineOf(g)
	if sf == 0 || sg == 0 {
		// A point spine's supremum is the one distance it has — constant
		// against ANY partner spine (the doc's "an inner POINT spine has
		// d_sup = d trivially"), not only against another point.
		return true
	}
	if f.axis.Cross(g.axis).Len() > clrAngTol {
		return false
	}
	rel := g.anchor.Sub(f.anchor)
	perp := rel.Sub(f.axis.Scale(rel.Dot(f.axis))).Len()
	if sf == 1 && sg == 1 {
		return true // parallel cylinder axes: any offset
	}
	return perp <= k.tol // circle spines (or circle × line): coaxial only
}

// spineCriticals encloses every critical of the spine-pair distance; ok is
// false when the configuration is off the shipped path (handled coarse).
func (k *pairKernel) spineCriticals(f, g *cFace) ([]spineCrit, bool) {
	sf, sg := spineOf(f), spineOf(g)
	if sg < sf {
		out, ok := k.spineCriticals(g, f)
		for i := range out {
			out[i].fa, out[i].fb = out[i].fb, out[i].fa
		}
		return out, ok
	}
	switch {
	case sf == 0 && sg == 0:
		return []spineCrit{exactCrit(f.anchor, g.anchor)}, true
	case sf == 0 && sg == 1:
		foot := linePoint(g.anchor, g.axis, f.anchor)
		return []spineCrit{exactCrit(f.anchor, foot)}, true
	case sf == 0 && sg == 2:
		return pointCircleCrits(f.anchor, g.anchor, g.axis, g.refU, g.refV, g.major, k.tol), true
	case sf == 1 && sg == 1:
		return k.lineLineCrits(f, g), true
	case sf == 1 && sg == 2:
		cp := circleParam{
			c: [3]float64{g.anchor.X, g.anchor.Y, g.anchor.Z},
			u: [3]float64{g.refU.X, g.refU.Y, g.refU.Z},
			v: [3]float64{g.refV.X, g.refV.Y, g.refV.Z},
			r: g.major,
		}
		return k.lineCircleBracketCrits(cp, g.anchor, g.refU, g.refV, f.anchor, f.axis)
	default:
		return k.circleCircleCrits(f, g)
	}
}

func exactCrit(a, b r3.Vec) spineCrit {
	d := a.Sub(b).Len()
	return spineCrit{lo: d, hi: d, exact: true, fa: a, fb: b}
}

// linePoint is the foot of p on the line (a, unit d).
func linePoint(a, d, p r3.Vec) r3.Vec {
	return a.Add(d.Scale(p.Sub(a).Dot(d)))
}

// pointCircleCrits are the near and far criticals of a point against a
// circle — closed form; a point on the circle's axis has constant distance,
// represented at a deterministic azimuth.
func pointCircleCrits(p, c, axis, refU, refV r3.Vec, rad, tol float64) []spineCrit {
	rel := p.Sub(c)
	perp := rel.Sub(axis.Scale(rel.Dot(axis)))
	dir, ok := perp.Normalize()
	if !ok || perp.Len() <= tol {
		q := c.Add(refU.Scale(rad))
		_ = refV
		return []spineCrit{exactCrit(p, q)}
	}
	near := c.Add(dir.Scale(rad))
	far := c.Sub(dir.Scale(rad))
	return []spineCrit{exactCrit(p, near), exactCrit(p, far)}
}

// lineLineCrits: parallel axes carry one constant-distance family
// (represented at the axial-overlap midpoint); skew or crossing axes carry
// the single common-perpendicular critical.
func (k *pairKernel) lineLineCrits(f, g *cFace) []spineCrit {
	cross := f.axis.Cross(g.axis)
	rel := g.anchor.Sub(f.anchor)
	if cross.Len() <= clrAngTol {
		// Parallel: the representative sits at the overlap midpoint of the
		// two axial windows, projected onto each axis.
		sign := 1.0
		if g.axis.Dot(f.axis) < 0 {
			sign = -1
		}
		gLoOnF := rel.Dot(f.axis) + sign*g.zWin.lo
		gHiOnF := rel.Dot(f.axis) + sign*g.zWin.hi
		gw := newLinWindow(gLoOnF, gHiOnF)
		lo := math.Max(f.zWin.lo, gw.lo)
		hi := math.Min(f.zWin.hi, gw.hi)
		z := (lo + hi) / 2
		if lo > hi {
			z = math.Max(f.zWin.lo, math.Min(f.zWin.hi, (gw.lo+gw.hi)/2))
		}
		fa := f.anchor.Add(f.axis.Scale(z))
		return []spineCrit{exactCrit(fa, linePoint(g.anchor, g.axis, fa))}
	}
	// Common perpendicular of two non-parallel lines.
	a := f.axis
	b := g.axis
	ab := a.Dot(b)
	den := 1 - ab*ab
	ra := rel.Dot(a)
	rb := rel.Dot(b)
	s := (ra - ab*rb) / den
	t := (ab*ra - rb) / den
	fa := f.anchor.Add(a.Scale(s))
	fb := g.anchor.Add(b.Scale(t))
	return []spineCrit{exactCrit(fa, fb)}
}

// lineCircleBracketCrits runs the P4 machinery for an explicit circle.
func (k *pairKernel) lineCircleBracketCrits(cp circleParam, center, refU, refV, la, ld r3.Vec) ([]spineCrit, bool) {
	brs, ok := lineCircleBrackets(cp, [3]float64{la.X, la.Y, la.Z}, [3]float64{ld.X, ld.Y, ld.Z}, k.slack)
	if !ok {
		// Constant distance over the circle (a coaxial configuration):
		// closed form at a deterministic azimuth.
		q := center.Add(refU.Scale(cp.r))
		d := q.Sub(linePoint(la, ld, q)).Len()
		return []spineCrit{{lo: d, hi: d, exact: true, fa: linePoint(la, ld, q), fb: q}}, true
	}
	var out []spineCrit
	for _, br := range brs {
		s, c := math.Sincos(br.mid())
		q := center.Add(refU.Scale(cp.r * c)).Add(refV.Scale(cp.r * s))
		out = append(out, spineCrit{lo: br.lo, hi: br.hi, fa: linePoint(la, ld, q), fb: q})
	}
	return out, true
}

// circleCircleCrits is the P8 spine cell: coaxial spines are the certified
// constant-distance closed form; otherwise the Sturm brackets, guarded
// against the ρ = 0 kink (a spine meeting the other's axis, where the
// distance is not differentiable — off the shipped path, handled coarse).
func (k *pairKernel) circleCircleCrits(f, g *cFace) ([]spineCrit, bool) {
	rel := f.anchor.Sub(g.anchor)
	if f.axis.Cross(g.axis).Len() <= clrAngTol &&
		rel.Sub(g.axis.Scale(rel.Dot(g.axis))).Len() <= k.tol {
		// Coaxial: constant distance hypot(dz, ΔR) at matched azimuths.
		dz := rel.Dot(g.axis)
		d := math.Hypot(dz, f.major-g.major)
		pf := f.anchor.Add(f.refU.Scale(f.major))
		perp := pf.Sub(g.anchor).Sub(g.axis.Scale(pf.Sub(g.anchor).Dot(g.axis)))
		dir, ok := perp.Normalize()
		if !ok {
			return nil, false
		}
		pg := g.anchor.Add(dir.Scale(g.major))
		return []spineCrit{{lo: d, hi: d, exact: true, fa: pf, fb: pg}}, true
	}
	// Kink guard: the P8 stationarity is smooth only while f's spine stays
	// clear of g's axis (§4's foot-map caveat, at the spine level); a spine
	// that can reach the axis is off the shipped path.
	axisDist := rel.Sub(g.axis.Scale(rel.Dot(g.axis))).Len()
	if axisDist-f.major <= k.tol {
		return nil, false
	}
	c1 := circleParam{
		c: [3]float64{f.anchor.X, f.anchor.Y, f.anchor.Z},
		u: [3]float64{f.refU.X, f.refU.Y, f.refU.Z},
		v: [3]float64{f.refV.X, f.refV.Y, f.refV.Z},
		r: f.major,
	}
	c2 := circleParam{
		c: [3]float64{g.anchor.X, g.anchor.Y, g.anchor.Z},
		u: [3]float64{g.refU.X, g.refU.Y, g.refU.Z},
		v: [3]float64{g.refV.X, g.refV.Y, g.refV.Z},
		r: g.major,
	}
	brs, ok := circleCircleBrackets(c1, c2, [3]float64{g.axis.X, g.axis.Y, g.axis.Z}, k.slack)
	if !ok {
		return nil, false
	}
	var out []spineCrit
	for _, br := range brs {
		s, c := math.Sincos(br.mid())
		pf := f.anchor.Add(f.refU.Scale(f.major * c)).Add(f.refV.Scale(f.major * s))
		// The matching foot on g's spine: the nearest spine point.
		relP := pf.Sub(g.anchor)
		perp := relP.Sub(g.axis.Scale(relP.Dot(g.axis)))
		dir, dok := perp.Normalize()
		if !dok {
			return nil, false
		}
		pg := g.anchor.Add(dir.Scale(g.major))
		out = append(out, spineCrit{lo: br.lo, hi: br.hi, fa: pf, fb: pg})
	}
	return out, true
}

// emitOffsetCombos emits the four offset combinations of one spine-pair
// critical: the joining line meets each offset surface twice, and every
// carrier-pair stationary point lies among the combinations, so admission
// (§3) decides each in isolation.
func (k *pairKernel) emitOffsetCombos(sink *cellSink, f, g *cFace, c spineCrit) {
	sep := c.fb.Sub(c.fa)
	d := sep.Len()
	if d <= k.tol {
		// Coincident spine feet: the joining direction degenerates into a
		// whole ring. The certified constant-distance configurations carry
		// it (concentric shells, coaxial cylinders — the peg-in-hole
		// reading); anything else is a contact question this cell cannot
		// decide.
		if k.constantSpineDist(f, g) && spineOf(f) <= 1 && spineOf(g) <= 1 {
			k.emitRingCombos(sink, f, g, c)
			return
		}
		sink.unsure = true
		return
	}
	dir := sep.Scale(1 / d)
	rf, rg := spineOffset(f), spineOffset(g)
	margin := k.tol + (c.hi - c.lo)
	for _, sf := range []float64{1, -1} {
		for _, sg := range []float64{1, -1} {
			pf := c.fa.Add(dir.Scale(sf * rf))
			pg := c.fb.Sub(dir.Scale(sg * rg))
			rawLo := c.lo - sf*rf - sg*rg
			rawHi := c.hi - sf*rf - sg*rg
			admit := admitState(f.admitPoint(pf, margin), g.admitPoint(pg, margin))
			if rawLo <= k.tol && rawHi >= -k.tol {
				if admit != -1 {
					sink.unsure = true
				}
				continue
			}
			lo, hi := math.Abs(rawLo), math.Abs(rawHi)
			if lo > hi {
				lo, hi = hi, lo
			}
			sink.candidate(k, admit, lo, hi, c.exact, pf, pg)
		}
	}
}

// emitRingCombos handles the concentric family of a coincident-spine
// critical over point/line spines: the annular gap |rf − rg| and the
// far-side rf + rg pairings, represented at deterministic radial directions
// and admitted per trim like every candidate.
func (k *pairKernel) emitRingCombos(sink *cellSink, f, g *cFace, c spineCrit) {
	axis := f.axis
	if spineOf(f) == 0 && spineOf(g) == 1 {
		axis = g.axis
	}
	if spineOf(f) == 0 && spineOf(g) == 0 {
		axis = perpTo(c.fa.Sub(c.fb).Add(r3.NewVec(0, 0, 1)))
	}
	u := perpTo(axis)
	v := axis.Cross(u)
	rf, rg := spineOffset(f), spineOffset(g)
	dirs := make([]r3.Vec, 0, 8)
	for _, th := range ringAngles(f, g, u, v) {
		dirs = append(dirs, u.Scale(math.Cos(th)).Add(v.Scale(math.Sin(th))))
	}
	if spineOf(f) == 0 && spineOf(g) == 0 {
		dirs = append(dirs, axis, axis.Scale(-1))
	}
	for _, dir := range dirs {
		pf := c.fa.Add(dir.Scale(rf))
		near := c.fb.Add(dir.Scale(rg))
		far := c.fb.Sub(dir.Scale(rg))
		margin := k.tol + (c.hi - c.lo)
		sink.candidate(k, admitState(f.admitPoint(pf, margin), g.admitPoint(near, margin)),
			math.Abs(rf-rg), math.Abs(rf-rg), c.exact, pf, near)
		sink.candidate(k, admitState(f.admitPoint(pf, margin), g.admitPoint(far, margin)),
			rf+rg, rf+rg, c.exact, pf, far)
	}
}

// ringAngles proposes representative azimuths: the faces' own window mids
// plus the quarters — deterministic, and covering every interval overlap the
// windows can form.
func ringAngles(f, g *cFace, u, v r3.Vec) []float64 {
	out := []float64{0, math.Pi / 2, math.Pi, 3 * math.Pi / 2}
	for _, fc := range []*cFace{f, g} {
		if fc.sweep.full {
			continue
		}
		mid := (fc.sweep.lo + fc.sweep.hi) / 2
		dir := fc.refU.Scale(math.Cos(mid)).Add(fc.refV.Scale(math.Sin(mid)))
		out = append(out, math.Atan2(dir.Dot(v), dir.Dot(u)))
	}
	return out
}

// planePlane is the plane-row cell for two planes: a coplanar pair reaching
// here is uncertified (the PR 1 coplanar contact certificate runs at the
// pair level first); a parallel distinct pair carries its plateau exactly
// where the trims overlap in projection (§3's projection rule); crossing
// carriers are excluded through the trims or read as the §6/§7-routed
// contact, never quietly as a boundary answer.
func (k *pairKernel) planePlane(f, g *cFace, sink *cellSink) {
	// The plateau exists only for EXACT parallelism: a tolerance here
	// blesses a tilted pair's mid-face height as an Exact minimum the true
	// gap undercuts (or exceeds). A tilt too small for the crossing-line
	// path to certify falls to unsure there — never a wrong Exact.
	if f.n.Cross(g.n).Len() == 0 {
		h := g.o.Sub(f.o).Dot(f.n)
		rel, wit := k.coplanarRelation(f, g)
		if math.Abs(h) <= k.tol {
			if rel != -1 {
				sink.unsure = true
			}
			return
		}
		switch rel {
		case 1:
			pa := f.o.Add(f.u.Scale(wit[0])).Add(f.v.Scale(wit[1]))
			pb := pa.Add(f.n.Scale(h))
			sink.candidate(k, 1, math.Abs(h), math.Abs(h), true, pa, pb)
		case 0:
			sink.loOnly(math.Abs(h))
		}
		return
	}
	if clrBoxDist(f.box, g.box) > k.tol {
		return
	}
	// The intersection line, clipped by both trims on a shared parameter.
	dir, ok := f.n.Cross(g.n).Normalize()
	if !ok {
		sink.unsure = true
		return
	}
	p0, ok := planesIntersect(f, g)
	if !ok {
		sink.unsure = true
		return
	}
	fx, fy := f.planeCoords(p0)
	gx, gy := g.planeCoords(p0)
	// Conservative supersets of each trim's intersection with the line: a
	// clean miss between them excludes the crossing; anything else is the
	// §6/§7-routed contact question, undecided in PR 1.
	supF := f.region.lineIntervalsSuperset(fx, fy, dir.Dot(f.u), dir.Dot(f.v))
	supG := g.region.lineIntervalsSuperset(gx, gy, dir.Dot(g.u), dir.Dot(g.v))
	if intervalsMeet(supF, supG, k.tol) != -1 {
		sink.unsure = true
	}
}

// planesIntersect returns a point on the two planes' intersection line.
func planesIntersect(f, g *cFace) (r3.Vec, bool) {
	// Solve within the pencil: p = f.o + s·d with d in f's plane,
	// perpendicular to the intersection direction.
	dir := f.n.Cross(g.n)
	inPlane, ok := f.n.Cross(dir).Normalize()
	if !ok {
		return r3.Vec{}, false
	}
	den := inPlane.Dot(g.n)
	if math.Abs(den) < 1e-12 {
		return r3.Vec{}, false
	}
	s := g.o.Sub(f.o).Dot(g.n) / den
	return f.o.Add(inPlane.Scale(s)), true
}

// intervalsMeet classifies two interval sets on a shared line: +1 provably
// overlapping, −1 provably apart, 0 ambiguous.
func intervalsMeet(a, b []clrIv, tol float64) int {
	if len(a) == 0 || len(b) == 0 {
		return -1
	}
	out := -1
	for _, ia := range a {
		for _, ib := range b {
			lo := math.Max(ia.lo, ib.lo)
			hi := math.Min(ia.hi, ib.hi)
			if hi-lo > tol {
				return 1
			}
			if hi-lo > -tol {
				out = 0
			}
		}
	}
	return out
}

// coplanarRelation classifies two parallel-plane trims in projection along
// the normal: +1 proven positive-area overlap (with a witness point in f's
// frame), −1 provenly apart, 0 ambiguous. Sample-based in the sufficient
// direction, boundary-clearance-based in the exclusion direction — never a
// blessed ambiguity.
func (k *pairKernel) coplanarRelation(f, g *cFace) (int, [2]float64) {
	ge := make([]surveyElem, 0, len(g.region.elems))
	for _, e := range g.region.elems {
		ge = append(ge, transformElem(e, g, f))
	}
	greg := newRegion2(ge)
	tol := math.Max(f.region.tol(), greg.tol())

	probeInto := func(src, dst region2) (int, [2]float64) {
		if p, ok := src.interiorPoint(); ok {
			if dst.classify(p[0], p[1], tol) == 1 {
				return 1, p
			}
		}
		for _, s := range src.samples() {
			if dst.classify(s[0], s[1], tol) == 1 {
				return 1, s
			}
		}
		return 0, [2]float64{}
	}
	if r, w := probeInto(greg, f.region); r == 1 {
		return 1, w
	}
	if r, w := probeInto(f.region, greg); r == 1 {
		return 1, w
	}
	// Exclusion: boundaries clear each other and neither contains the other.
	clearing := math.Inf(1)
	for _, ef := range f.region.elems {
		for _, eg := range greg.elems {
			if d := elemElemDistLB(ef, eg); d < clearing {
				clearing = d
			}
		}
	}
	if clearing > tol &&
		regionSampleOutside(f.region, greg, tol) &&
		regionSampleOutside(greg, f.region, tol) {
		return -1, [2]float64{}
	}
	return 0, [2]float64{}
}

// regionSampleOutside reports whether a sample of src is cleanly outside dst
// — with boundaries provably apart, one clean sample settles containment.
func regionSampleOutside(src, dst region2, tol float64) bool {
	for _, s := range src.samples() {
		switch dst.classify(s[0], s[1], tol) {
		case -1:
			return true
		case 1:
			return false
		}
	}
	return false
}

// transformElem maps a boundary element from one plane face's 2D frame into
// another parallel face's frame (projection along the shared normal — a
// planar rigid motion, possibly reflected).
func transformElem(e surveyElem, from, to *cFace) surveyElem {
	mapPt := func(x, y float64) (float64, float64) {
		w := from.o.Add(from.u.Scale(x)).Add(from.v.Scale(y))
		return to.planeCoords(w)
	}
	if e.kind == surveyLine {
		ax, ay := mapPt(e.ax, e.ay)
		bx, by := mapPt(e.bx, e.by)
		out, _ := lineElem(ax, ay, bx, by)
		return out
	}
	cx, cy := mapPt(e.qx, e.qy)
	delta := math.Atan2(from.u.Dot(to.v), from.u.Dot(to.u))
	det := from.u.Cross(from.v).Dot(to.u.Cross(to.v))
	var th0, th1 float64
	if det >= 0 {
		th0, th1 = e.th0+delta, e.th1+delta
	} else {
		th0, th1 = delta-e.th1, delta-e.th0
	}
	out, _ := arcElem(cx, cy, e.rr, math.Min(th0, th1), math.Max(th0, th1), e.closed)
	return out
}

// elemElemDistLB is a lower bound on the distance between two 2D boundary
// elements (arcs bounded through their full circles — an underestimate, the
// sound direction for exclusion proofs).
func elemElemDistLB(a, b surveyElem) float64 {
	if a.kind == surveyLine && b.kind == surveyLine {
		return segSegDist(a.ax, a.ay, a.bx, a.by, b.ax, b.ay, b.bx, b.by)
	}
	if a.kind == surveyLine {
		return segElemDistLB(b, a.ax, a.ay, a.bx, a.by)
	}
	if b.kind == surveyLine {
		return segElemDistLB(a, b.ax, b.ay, b.bx, b.by)
	}
	d := math.Hypot(b.qx-a.qx, b.qy-a.qy)
	if d >= a.rr+b.rr {
		return d - a.rr - b.rr
	}
	if d <= math.Abs(a.rr-b.rr) {
		return math.Abs(a.rr-b.rr) - d
	}
	return 0
}

// circleRegionHits classifies a full 2D circle against a region: +1 provably
// meeting it, −1 provably clear of it, 0 ambiguous.
func circleRegionHits(r region2, cx, cy, rad float64) int {
	tol := r.tol()
	for i := range 8 {
		th := float64(i) * math.Pi / 4
		if r.classify(cx+rad*math.Cos(th), cy+rad*math.Sin(th), tol) == 1 {
			return 1
		}
	}
	clearing := math.Inf(1)
	probe := surveyElem{kind: surveyArc, qx: cx, qy: cy, rr: rad, th0: 0, th1: 2 * math.Pi, closed: true}
	for _, e := range r.elems {
		if d := elemElemDistLB(e, probe); d < clearing {
			clearing = d
		}
	}
	if clearing > tol && r.classify(cx+rad, cy, tol) == -1 {
		return -1
	}
	return 0
}

// planeCylinder is the plane-row cell for a cylinder: an axis-parallel pair
// carries the constant ruling plateau; a crossing carrier is excluded
// through the trims (the two crossing rulings when the axis parallels the
// plane, the exact axial crossing range otherwise) or routed to §6/§7.
func (k *pairKernel) planeCylinder(f, g *cFace, sink *cellSink) {
	s := f.n.Dot(g.axis)
	if s != 0 {
		// The constant-ruling plateau exists only for EXACT axis
		// parallelism: a tolerance here would bless a tilted cylinder's
		// plateau as an Exact reading the true minimum undercuts. A tilt
		// too small for the crossing machinery to certify falls through
		// to unsure there — never a wrong Exact.
		k.planeCrossesRevolved(f, g, sink)
		return
	}
	hAxis := g.anchor.Sub(f.o).Dot(f.n)
	d := math.Abs(hAxis)
	side := 1.0
	if hAxis < 0 {
		side = -1
	}
	toward := f.n.Scale(-side) // radial direction from the axis toward the plane
	switch {
	case d-g.radius > k.tol:
		for _, dirSign := range []float64{1, -1} {
			radial := toward.Scale(dirSign)
			v := d - dirSign*g.radius
			k.rulingCandidate(f, g, radial, v, sink)
		}
	case g.radius-d > k.tol:
		// Crossing along two rulings at ±acos(d/R) around the toward
		// direction.
		half := math.Acos(math.Max(-1, math.Min(1, d/g.radius)))
		base := math.Atan2(toward.Dot(g.refV), toward.Dot(g.refU))
		for _, dth := range []float64{half, -half} {
			th := base + dth
			radial := g.refU.Scale(math.Cos(th)).Add(g.refV.Scale(math.Sin(th)))
			if !k.rulingExcluded(f, g, radial) {
				sink.unsure = true
				return
			}
		}
	default:
		// Tangency at distance zero: certified only in PR 3 (§6); excluded
		// through the trims or undecided.
		if !k.rulingExcluded(f, g, toward) {
			sink.unsure = true
		}
	}
}

// rulingCandidate emits the plateau candidate carried by one cylinder
// ruling at radial direction `radial` and plane distance v.
func (k *pairKernel) rulingCandidate(f, g *cFace, radial r3.Vec, v float64, sink *cellSink) {
	th := math.Atan2(radial.Dot(g.refV), radial.Dot(g.refU))
	angAdmit := g.sweep.classify(th, k.tol/math.Max(g.radius, 1e-30))
	if angAdmit == -1 {
		return
	}
	p0 := g.anchor.Add(radial.Scale(g.radius)).Add(g.axis.Scale(g.zWin.lo))
	p1 := g.anchor.Add(radial.Scale(g.radius)).Add(g.axis.Scale(g.zWin.hi))
	x0, y0 := f.planeCoords(p0)
	x1, y1 := f.planeCoords(p1)
	hit, w := f.region.segmentHits(x0, y0, x1, y1)
	if hit == -1 {
		return
	}
	admit := admitState(angAdmit, hit)
	pa := f.o.Add(f.u.Scale(w[0])).Add(f.v.Scale(w[1]))
	h := f.n.Dot(p0.Sub(f.o))
	pb := pa.Add(f.n.Scale(h))
	sink.candidate(k, admit, v, v, true, pa, pb)
}

// rulingExcluded proves one carrier-crossing ruling never meets both trims.
func (k *pairKernel) rulingExcluded(f, g *cFace, radial r3.Vec) bool {
	th := math.Atan2(radial.Dot(g.refV), radial.Dot(g.refU))
	if g.sweep.classify(th, k.tol/math.Max(g.radius, 1e-30)) == -1 {
		return true
	}
	p0 := g.anchor.Add(radial.Scale(g.radius)).Add(g.axis.Scale(g.zWin.lo))
	p1 := g.anchor.Add(radial.Scale(g.radius)).Add(g.axis.Scale(g.zWin.hi))
	x0, y0 := f.planeCoords(p0)
	x1, y1 := f.planeCoords(p1)
	hit, _ := f.region.segmentHits(x0, y0, x1, y1)
	return hit == -1
}

// planeCrossesRevolved excludes (or reports) the crossing of a plane with a
// tilted revolution carrier: face boxes first, then the exact axial range of
// the carrier crossing against the axial window, then — for a perpendicular
// axis — the exact crossing circle against the plane trim.
func (k *pairKernel) planeCrossesRevolved(f, g *cFace, sink *cellSink) {
	if clrBoxDist(f.box, g.box) > k.tol {
		return
	}
	s := f.n.Dot(g.axis)
	c := f.planeOffset()
	base := f.n.Dot(g.anchor)
	lo, hi := 0.0, 2*math.Pi
	if !g.sweep.full {
		lo, hi = g.sweep.lo, g.sweep.hi
	}
	mn, mx := trigRange(f.n.Dot(g.refU)*g.radius, f.n.Dot(g.refV)*g.radius, lo, hi)
	z0 := (c - base - mx) / s
	z1 := (c - base - mn) / s
	zw := newLinWindow(z0, z1)
	if zw.hi < g.zWin.lo-k.tol || zw.lo > g.zWin.hi+k.tol {
		return
	}
	if math.Abs(s) >= 1-clrAngTol {
		// Axis perpendicular to the plane: the crossing is the circle at the
		// plane's own axial station, exact against the region.
		center := g.anchor.Add(g.axis.Scale((c - base) / s))
		cx, cy := f.planeCoords(center)
		if circleRegionHits(f.region, cx, cy, g.radius) == -1 {
			return
		}
	}
	sink.unsure = true
}

// planeCone is the plane-row cone cell: the plateau exists exactly when the
// plane parallels a ruling (|n·axis| = sin α), at the apex's own plane
// distance; otherwise the cell only owes the crossing exclusion — the exact
// range of n·x over the trimmed face.
func (k *pairKernel) planeCone(f, g *cFace, sink *cellSink) {
	s := f.n.Dot(g.axis)
	sinA := math.Sin(g.half)
	apexH := g.anchor.Sub(f.o).Dot(f.n)
	nu, nv := f.n.Dot(g.refU), f.n.Dot(g.refV)

	// Exact range of n·x − c over the trimmed face.
	lo, hi := 0.0, 2*math.Pi
	if !g.sweep.full {
		lo, hi = g.sweep.lo, g.sweep.hi
	}
	mn, mx := trigRange(nu, nv, lo, hi)
	tanA := math.Tan(g.half)
	rangeLo, rangeHi := math.Inf(1), math.Inf(-1)
	for _, z := range []float64{g.zWin.lo, g.zWin.hi} {
		for _, m := range []float64{mn, mx} {
			v := apexH + z*(s+tanA*m)
			rangeLo = math.Min(rangeLo, v)
			rangeHi = math.Max(rangeHi, v)
		}
	}
	crossingExcluded := rangeLo > k.tol || rangeHi < -k.tol || clrBoxDist(f.box, g.box) > k.tol

	if math.Abs(math.Abs(s)-sinA) <= clrAngTol {
		if math.Abs(apexH) <= k.tol {
			sink.unsure = true
			return
		}
		// The parallel ruling: the azimuth where n·d(θ) reaches ∓|s|·cos α.
		sign := -1.0
		if s < 0 {
			sign = 1
		}
		th := math.Atan2(sign*nv, sign*nu)
		angAdmit := g.sweep.classify(th, clrAngTol*10)
		if angAdmit != -1 {
			radial := g.refU.Scale(math.Cos(th)).Add(g.refV.Scale(math.Sin(th)))
			p0 := g.anchor.Add(g.axis.Scale(g.zWin.lo)).Add(radial.Scale(g.zWin.lo * tanA))
			p1 := g.anchor.Add(g.axis.Scale(g.zWin.hi)).Add(radial.Scale(g.zWin.hi * tanA))
			x0, y0 := f.planeCoords(p0)
			x1, y1 := f.planeCoords(p1)
			hit, w := f.region.segmentHits(x0, y0, x1, y1)
			if hit != -1 {
				pa := f.o.Add(f.u.Scale(w[0])).Add(f.v.Scale(w[1]))
				sink.candidate(k, admitState(angAdmit, hit), math.Abs(apexH), math.Abs(apexH), true, pa, pa.Add(f.n.Scale(apexH)))
			}
		}
	}
	if !crossingExcluded {
		sink.unsure = true
	}
}

// planeSphere is the plane-row sphere cell: the center's plane distance
// minus the radius, tangency routed to §6, a crossing excluded through the
// trims (the exact crossing circle) or undecided.
func (k *pairKernel) planeSphere(f, g *cFace, sink *cellSink) {
	h := g.anchor.Sub(f.o).Dot(f.n)
	d := math.Abs(h)
	side := 1.0
	if h < 0 {
		side = -1
	}
	switch {
	case d-g.radius > k.tol:
		for _, dirSign := range []float64{1, -1} {
			pb := g.anchor.Sub(f.n.Scale(side * dirSign * g.radius))
			foot := pb.Sub(f.n.Scale(f.n.Dot(pb.Sub(f.o))))
			x, y := f.planeCoords(foot)
			admit := admitState(f.region.classify(x, y, k.tol), g.admitPoint(pb, k.tol))
			sink.candidate(k, admit, d-dirSign*g.radius, d-dirSign*g.radius, true, foot, pb)
		}
	case g.radius-d > k.tol:
		if clrBoxDist(f.box, g.box) > k.tol {
			return
		}
		rc := math.Sqrt(g.radius*g.radius - h*h)
		foot := g.anchor.Sub(f.n.Scale(h))
		cx, cy := f.planeCoords(foot)
		if circleRegionHits(f.region, cx, cy, rc) == -1 {
			return
		}
		sink.unsure = true
	default:
		pb := g.anchor.Sub(f.n.Scale(side * g.radius))
		foot := pb.Sub(f.n.Scale(f.n.Dot(pb.Sub(f.o))))
		x, y := f.planeCoords(foot)
		if admitState(f.region.classify(x, y, k.tol), g.admitPoint(pb, k.tol)) == -1 {
			return
		}
		sink.unsure = true
	}
}

// planeTorus is the plane-row torus cell: the spine circle's plane-distance
// amplitude extreme minus the minor radius — valid for the spindle branch
// too, since the extreme's meridian direction always lies in the extreme
// spine point's own meridian plane.
func (k *pairKernel) planeTorus(f, g *cFace, sink *cellSink) {
	base := g.anchor.Sub(f.o).Dot(f.n)
	au := g.major * f.n.Dot(g.refU)
	av := g.major * f.n.Dot(g.refV)
	lo, hi := 0.0, 2*math.Pi
	if !g.sweep.full {
		lo, hi = g.sweep.lo, g.sweep.hi
	}
	mn, mx := trigRange(au, av, lo, hi)
	hLo, hHi := base+mn, base+mx
	if hLo-g.radius > k.tol || -hHi-g.radius > k.tol {
		side := 1.0
		if hHi < 0 {
			side = -1
		}
		// The spine extreme nearest the plane must be an interior critical
		// of the sweep window (else the boundary tiers hold the minimum).
		star := math.Atan2(av, au)
		if side > 0 {
			star += math.Pi
		}
		if g.sweep.classify(star, clrAngTol*10) != 1 && !g.sweep.full {
			sink.loOnly(math.Min(math.Abs(hLo), math.Abs(hHi)) - g.radius)
			return
		}
		sp := g.anchor.Add(g.refU.Scale(g.major * math.Cos(star))).Add(g.refV.Scale(g.major * math.Sin(star)))
		pb := sp.Sub(f.n.Scale(side * g.radius))
		foot := pb.Sub(f.n.Scale(f.n.Dot(pb.Sub(f.o))))
		x, y := f.planeCoords(foot)
		v := math.Min(math.Abs(hLo), math.Abs(hHi)) - g.radius
		admit := admitState(f.region.classify(x, y, k.tol), g.admitPoint(pb, k.tol))
		sink.candidate(k, admit, v, v, true, foot, pb)
		return
	}
	if clrBoxDist(f.box, g.box) > k.tol {
		return
	}
	if hLo > g.radius-k.tol || hHi < -(g.radius-k.tol) {
		// Tangency zone: certified only in PR 3.
		sink.unsure = true
		return
	}
	sink.unsure = true
}

// coneSphere is the closed-form meridian point/cone cell of §4's sphere
// column: the sphere center against the cone's generating ray in its own
// meridian half-plane, offset by the sphere radius on the strict branches.
func (k *pairKernel) coneSphere(f, g *cFace, sink *cellSink) {
	cone, sph := f, g
	if cone.kind != ckCone {
		cone, sph = g, f
	}
	rel := sph.anchor.Sub(cone.anchor)
	z := rel.Dot(cone.axis)
	perp := rel.Sub(cone.axis.Scale(z))
	rho := perp.Len()
	var radial r3.Vec
	if rho > k.tol {
		radial, _ = perp.Normalize()
	} else {
		// A center on the axis makes every azimuth equivalent; represent at
		// the sweep-window midpoint.
		mid := 0.0
		if !cone.sweep.full {
			mid = (cone.sweep.lo + cone.sweep.hi) / 2
		}
		radial = cone.refU.Scale(math.Cos(mid)).Add(cone.refV.Scale(math.Sin(mid)))
	}
	sinA, cosA := math.Sincos(cone.half)
	t := z*cosA + rho*sinA // slant projection onto the ruling
	if t <= k.tol {
		// The nearest carrier point is the apex — a singular point owned by
		// the synthesized vertex tier; the interior cell has no critical.
		if clrBoxDist(f.box, g.box) <= k.tol && z*z+rho*rho <= (sph.radius+k.tol)*(sph.radius+k.tol) {
			sink.unsure = true
		}
		return
	}
	dCarrier := math.Abs(rho*cosA - z*sinA)
	v := dCarrier - sph.radius
	pf := cone.anchor.Add(cone.axis.Scale(t * cosA)).Add(radial.Scale(t * sinA))
	sep := pf.Sub(sph.anchor)
	switch {
	case v > k.tol:
		dir, ok := sep.Normalize()
		if !ok {
			sink.unsure = true
			return
		}
		pg := sph.anchor.Add(dir.Scale(sph.radius))
		admit := admitState(cone.admitPoint(pf, k.tol), sph.admitPoint(pg, k.tol))
		sink.candidate(k, admit, v, v, true, pf, pg)
	case v < -k.tol:
		if clrBoxDist(f.box, g.box) > k.tol {
			return
		}
		sink.unsure = true
	default:
		if admitState(cone.admitPoint(pf, k.tol), sph.admitPoint(sph.anchor.Add(sep), k.tol)) == -1 {
			return
		}
		sink.unsure = true
	}
}
