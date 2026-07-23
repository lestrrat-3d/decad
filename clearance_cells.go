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
// pairs and spindle-torus pairs are outside the certified-cell set and
// contribute a coarse conservative enclosure instead (§8 — proven disjoint with a wide
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
	// overlap is set only by a trim-admitted transversal boundary crossing.
	// Such a crossing proves a shared open material neighborhood.
	overlap bool
	// unsure is set when a cell meets a question it cannot decide: an
	// admitted-or-ambiguous carrier crossing, an uncertified contact, an
	// equality where a branch demands strictness (§4: equality routes to §6,
	// where only the coplanar plane pair is certified).
	unsure bool
}

// crossing records a carrier crossing after trim admission: admitted proves
// overlap, rejected proves absence, and a boundary-straddling admission stays
// undecided.
func (s *cellSink) crossing(admit int) {
	switch admit {
	case 1:
		s.overlap = true
	case 0:
		s.unsure = true
	}
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

// enumerate runs every tier over the pair. One shared budget bounds
// cancellation latency across both the outer candidate walk and the nested
// work performed by a vertex tier.
func (k *pairKernel) enumerate() (*cellSink, error) {
	sink := &cellSink{}
	budget := newWorkBudget(k.ctx)
	check := func() error {
		if k.err != nil {
			return k.err
		}
		return budget.step()
	}
	for _, fa := range k.a.faces {
		for _, fb := range k.b.faces {
			if err := check(); err != nil {
				return nil, err
			}
			k.ffCell(fa, fb, sink)
		}
	}
	for _, fa := range k.a.faces {
		for _, eb := range k.b.edges {
			if err := check(); err != nil {
				return nil, err
			}
			k.feCell(fa, eb, sink)
		}
	}
	for _, fb := range k.b.faces {
		for _, ea := range k.a.edges {
			if err := check(); err != nil {
				return nil, err
			}
			k.feCell(fb, ea, sink)
		}
	}
	for _, ea := range k.a.edges {
		for _, eb := range k.b.edges {
			if err := check(); err != nil {
				return nil, err
			}
			k.eeCell(ea, eb, sink)
		}
	}
	for _, va := range k.a.verts {
		if err := check(); err != nil {
			return nil, err
		}
		if err := k.vertexTier(budget, va, k.b, sink); err != nil {
			return nil, err
		}
	}
	for _, vb := range k.b.verts {
		if err := check(); err != nil {
			return nil, err
		}
		if err := k.vertexTier(budget, vb, k.a, sink); err != nil {
			return nil, err
		}
	}
	for _, va := range k.a.verts {
		for _, vb := range k.b.verts {
			if err := check(); err != nil {
				return nil, err
			}
			d := va.Sub(vb).Len()
			sink.candidate(k, 1, d, d, true, va, vb)
		}
	}
	if k.err != nil {
		return nil, k.err
	}
	if err := budget.err(); err != nil {
		return nil, err
	}
	return sink, nil
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
		// Cone × sphere stays closed form (§4); everything else
		// cone-involved takes the coarse enclosure — the face-box distance
		// below, the closest witness pair above.
		if f.kind == ckSphere || g.kind == ckSphere {
			k.coneSphere(f, g, sink)
			return
		}
		sink.coarse(f.box, g.box, f.wit, g.wit)
	case (f.kind == ckTorus && f.spindle) || (g.kind == ckTorus && g.spindle):
		// A Minor ≥ Major torus leaves the polynomial path (§4): the pair
		// takes the coarse enclosure — the face-box distance below, the
		// closest witness pair above.
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
	minLo := math.Inf(1)
	for _, c := range crits {
		minLo = math.Min(minLo, c.lo)
		k.emitOffsetCombos(sink, f, g, c)
	}
	rf, rg := spineOffset(f), spineOffset(g)
	if minLo > rf+rg+k.tol {
		return // strict exterior: the carriers never meet (§4)
	}
	if k.certifiedContainment(f, g) {
		return
	}
	if clrBoxDist(f.box, g.box) > k.tol {
		return // the trimmed faces provably cannot touch
	}
	sink.unsure = true
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
		return k.pointCircleCrits(f.anchor, g.anchor, g.axis, g.refU, g.refV, g.major, g.sweep)
	case sf == 1 && sg == 1:
		return k.lineLineCrits(f, g)
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
// circle — closed form. A point PROVENLY on the circle's axis has the same
// distance at every azimuth, so one representative inside the trim's own
// window carries the whole family; a point provenly off the axis has the two
// criticals along its radial direction. An offset the oracle cannot decide
// leaves the radial direction unresolvable — and the azimuth it would produce
// decides a trim admission — so the cell reports no criticals rather than
// guess (ok=false).
func (k *pairKernel) pointCircleCrits(p, c, axis, refU, refV r3.Vec, rad float64, win angWindow) ([]spineCrit, bool) {
	switch k.onAxis(p, c, axis) {
	case degYes:
		th := 0.0
		if !win.full {
			th = (win.lo + win.hi) / 2
		}
		s, cs := math.Sincos(th)
		q := c.Add(refU.Scale(rad * cs)).Add(refV.Scale(rad * s))
		return []spineCrit{exactCrit(p, q)}, true
	case degNo:
		rel := p.Sub(c)
		perp := rel.Sub(axis.Scale(rel.Dot(axis)))
		dir, ok := perp.Normalize()
		if !ok {
			return nil, false
		}
		near := c.Add(dir.Scale(rad))
		far := c.Sub(dir.Scale(rad))
		return []spineCrit{exactCrit(p, near), exactCrit(p, far)}, true
	default:
		return nil, false
	}
}

// lineLineCrits: EXACTLY parallel axes carry one constant-distance family
// (represented at the axial-overlap midpoint — the family really is constant,
// so the representative is a proof); provenly non-parallel axes carry the
// single common-perpendicular critical. An axis pair in the oracle's undecided
// band has neither — a plateau there would be an Exact reading the true
// minimum undercuts, and the common perpendicular is not resolvable — so the
// cell falls to the coarse enclosure (ok=false).
func (k *pairKernel) lineLineCrits(f, g *cFace) ([]spineCrit, bool) {
	switch k.parallel(f.axis, g.axis) {
	case degYes:
		// The representative sits at the overlap midpoint of the two axial
		// windows, projected onto each axis.
		rel := g.anchor.Sub(f.anchor)
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
		return []spineCrit{exactCrit(fa, linePoint(g.anchor, g.axis, fa))}, true
	case degNo:
		c, ok := k.lineLinePerp(f.anchor, f.axis, g.anchor, g.axis)
		if !ok {
			return nil, false
		}
		return []spineCrit{c}, true
	default:
		return nil, false
	}
}

// lineCircleBracketCrits runs the P4 machinery for an explicit circle.
func (k *pairKernel) lineCircleBracketCrits(cp circleParam, center, refU, refV, la, ld r3.Vec) ([]spineCrit, bool) {
	brs, ok, err := lineCircleBracketsContext(k.ctx, cp, [3]float64{la.X, la.Y, la.Z}, [3]float64{ld.X, ld.Y, ld.Z}, k.slack)
	if err != nil {
		k.err = err
		return nil, false
	}
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

// circleCircleCrits is the P8 spine cell: EXACTLY coaxial spines are the
// certified constant-distance closed form; provenly non-coaxial spines take
// the Sturm brackets, guarded against the ρ = 0 kink (a spine meeting the
// other's axis, where the distance is not differentiable — off the shipped
// path, handled coarse). A pair the oracle cannot decide coaxial gets neither:
// the constant closed form would be an Exact reading a tilt undercuts, and the
// brackets' own foot map is not resolvable there.
func (k *pairKernel) circleCircleCrits(f, g *cFace) ([]spineCrit, bool) {
	rel := f.anchor.Sub(g.anchor)
	switch k.coaxial(g, f) {
	case degYes:
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
	case degUnknown:
		return nil, false
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
	brs, ok, err := circleCircleBracketsContext(k.ctx, c1, c2, [3]float64{g.axis.X, g.axis.Y, g.axis.Z}, k.slack)
	if err != nil {
		k.err = err
		return nil, false
	}
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
		// whole ring. Only a PROVEN ring family carries it (concentric
		// shells, coaxial cylinders — the peg-in-hole reading); a
		// near-coincidence the oracle cannot decide is a contact question
		// this cell has no right to answer.
		if k.ringFamily(f, g) == degYes {
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

// emitRingCombos handles the concentric family of a coincident-spine critical
// over point/line spines: the annular gap |rf − rg| and the far-side rf + rg
// pairings. The VALUE is the same all the way around the family — that is what
// the ring family means — so only the trim admission depends on the direction,
// and the directions below are a deterministic SAMPLE of it. A sample proves a
// candidate present, never absent: where no sampled direction of the near
// pairing is admitted, the family is not discarded (that would OVERSTATE the
// gap) but held as the proven lower bound it is — the annular distance bounds
// the whole carrier pair from below.
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
	dirs := make([]r3.Vec, 0, 24)
	for _, th := range ringAngles(f, g, u, v) {
		dirs = append(dirs, u.Scale(math.Cos(th)).Add(v.Scale(math.Sin(th))))
	}
	if spineOf(f) == 0 && spineOf(g) == 0 {
		dirs = append(dirs, axis, axis.Scale(-1))
	}
	margin := k.tol + (c.hi - c.lo)
	nearAdmitted := false
	for _, dir := range dirs {
		pf := c.fa.Add(dir.Scale(rf))
		near := c.fb.Add(dir.Scale(rg))
		far := c.fb.Sub(dir.Scale(rg))
		if a := admitState(f.admitPoint(pf, margin), g.admitPoint(near, margin)); a != -1 {
			nearAdmitted = true
			sink.candidate(k, a, math.Abs(rf-rg), math.Abs(rf-rg), c.exact, pf, near)
		}
		if a := admitState(f.admitPoint(pf, margin), g.admitPoint(far, margin)); a != -1 {
			sink.candidate(k, a, rf+rg, rf+rg, c.exact, pf, far)
		}
	}
	if !nearAdmitted {
		// The near pairing is the small one: a far pairing can never hold the
		// minimum the near pairing does not. Unproven absence keeps its bound.
		sink.loOnly(math.Abs(rf - rg))
	}
}

// ringAngles proposes representative azimuths of the ring family: a
// deterministic uniform set plus each face's own window endpoints and mid
// mapped into the ring frame — the places a partial trim's admission can begin,
// end or sit comfortably inside.
func ringAngles(f, g *cFace, u, v r3.Vec) []float64 {
	const uniform = 16
	out := make([]float64, 0, uniform+6)
	for i := range uniform {
		out = append(out, 2*math.Pi*float64(i)/uniform)
	}
	for _, fc := range []*cFace{f, g} {
		if fc.sweep.full {
			continue
		}
		for _, th := range []float64{fc.sweep.lo, (fc.sweep.lo + fc.sweep.hi) / 2, fc.sweep.hi} {
			dir := fc.refU.Scale(math.Cos(th)).Add(fc.refV.Scale(math.Sin(th)))
			out = append(out, math.Atan2(dir.Dot(v), dir.Dot(u)))
		}
	}
	return out
}

// planePlane is the plane-row cell for two planes: a coplanar pair reaching
// here is uncertified (the coplanar contact certificate runs at the
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
		rel, wit, err := k.coplanarRelation(newWorkBudget(k.ctx), f, g)
		if err != nil {
			k.err = err
			return
		}
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
	// Exact trim intervals certify positive-length transversal crossing. If
	// either trim cannot classify the line, conservative supersets still prove
	// a clean miss; every remaining case stays undecided.
	ivF, okF := f.region.lineIntervals(fx, fy, dir.Dot(f.u), dir.Dot(f.v))
	ivG, okG := g.region.lineIntervals(gx, gy, dir.Dot(g.u), dir.Dot(g.v))
	if okF && okG {
		sink.crossing(intervalsMeet(ivF, ivG, k.tol))
		return
	}
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
func (k *pairKernel) coplanarRelation(budget *workBudget, f, g *cFace) (int, [2]float64, error) {
	if err := budget.err(); err != nil {
		return 0, [2]float64{}, err
	}
	ge := make([]surveyElem, 0, len(g.region.elems))
	for _, e := range g.region.elems {
		if err := budget.step(); err != nil {
			return 0, [2]float64{}, err
		}
		ge = append(ge, transformElem(e, g, f))
	}
	greg, err := newRegion2Budget(budget, ge)
	if err != nil {
		return 0, [2]float64{}, err
	}
	tol := math.Max(f.region.tol(), greg.tol())

	probeInto := func(src, dst region2) (int, [2]float64, error) {
		p, ok, err := regionInteriorPointBudget(budget, src)
		if err != nil {
			return 0, [2]float64{}, err
		}
		if ok {
			class, err := regionClassifyBudget(budget, dst, p[0], p[1], tol)
			if err != nil {
				return 0, [2]float64{}, err
			}
			if class == 1 {
				return 1, p, nil
			}
		}
		samples, err := regionSamplesBudget(budget, src)
		if err != nil {
			return 0, [2]float64{}, err
		}
		for _, s := range samples {
			if err := budget.step(); err != nil {
				return 0, [2]float64{}, err
			}
			class, err := regionClassifyBudget(budget, dst, s[0], s[1], tol)
			if err != nil {
				return 0, [2]float64{}, err
			}
			if class == 1 {
				return 1, s, nil
			}
		}
		return 0, [2]float64{}, nil
	}
	r, w, err := probeInto(greg, f.region)
	if err != nil {
		return 0, [2]float64{}, err
	}
	if r == 1 {
		return 1, w, nil
	}
	r, w, err = probeInto(f.region, greg)
	if err != nil {
		return 0, [2]float64{}, err
	}
	if r == 1 {
		return 1, w, nil
	}
	// Exclusion: boundaries clear each other and neither contains the other.
	clearing, err := coplanarBoundaryClearanceBudget(budget, f.region, greg)
	if err != nil {
		return 0, [2]float64{}, err
	}
	if clearing <= tol {
		return 0, [2]float64{}, budget.err()
	}
	outsideFG, err := regionSampleOutsideBudget(budget, f.region, greg, tol)
	if err != nil {
		return 0, [2]float64{}, err
	}
	if !outsideFG {
		return 0, [2]float64{}, budget.err()
	}
	outsideGF, err := regionSampleOutsideBudget(budget, greg, f.region, tol)
	if err != nil {
		return 0, [2]float64{}, err
	}
	if outsideGF {
		return -1, [2]float64{}, nil
	}
	return 0, [2]float64{}, budget.err()
}

func coplanarBoundaryClearanceBudget(budget *workBudget, a, b region2) (float64, error) {
	clearing := math.Inf(1)
	for _, ea := range a.elems {
		for _, eb := range b.elems {
			if err := budget.step(); err != nil {
				return 0, err
			}
			if d := elemElemDistLB(ea, eb); d < clearing {
				clearing = d
			}
		}
	}
	return clearing, nil
}

// regionSampleOutside reports whether a sample of src is cleanly outside dst
// — with boundaries provably apart, one clean sample settles containment.
func regionSampleOutsideBudget(budget *workBudget, src, dst region2, tol float64) (bool, error) {
	samples, err := regionSamplesBudget(budget, src)
	if err != nil {
		return false, err
	}
	for _, s := range samples {
		if err := budget.step(); err != nil {
			return false, err
		}
		class, err := regionClassifyBudget(budget, dst, s[0], s[1], tol)
		if err != nil {
			return false, err
		}
		switch class {
		case -1:
			return true, nil
		case 1:
			return false, nil
		}
	}
	return false, nil
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

// trimmedCircleCrossingRelation classifies a transversal carrier-crossing
// circle through both face trims. A full-circle miss on the plane proves
// exclusion. Overlap requires one deterministic point cleanly admitted by the
// plane region and the revolved face's own sweep/meridian/axial trims; missing
// such a point is undecided, never a false admission from the full carrier.
func trimmedCircleCrossingRelation(f, g *cFace, center r3.Vec, rad, tol float64) int {
	cx, cy := f.planeCoords(center)
	if circleRegionHits(f.region, cx, cy, rad) == -1 {
		return -1
	}
	const samples = 64
	for i := range samples {
		th := 2 * math.Pi * float64(i) / samples
		q := center.Add(f.u.Scale(rad * math.Cos(th))).Add(f.v.Scale(rad * math.Sin(th)))
		if admitState(f.admitPoint(q, tol), g.admitPoint(q, tol)) == 1 {
			return 1
		}
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
		ambiguous := false
		for _, dth := range []float64{half, -half} {
			th := base + dth
			radial := g.refU.Scale(math.Cos(th)).Add(g.refV.Scale(math.Sin(th)))
			rel := k.rulingRelation(f, g, radial)
			if rel == 1 {
				sink.overlap = true
				return
			}
			ambiguous = ambiguous || rel == 0
		}
		if ambiguous {
			sink.unsure = true
		}
	default:
		// Tangency at distance zero: not certified here (§6); excluded
		// through the trims or undecided.
		if k.rulingRelation(f, g, toward) != -1 {
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

// rulingRelation classifies one carrier-crossing ruling through both trims.
func (k *pairKernel) rulingRelation(f, g *cFace, radial r3.Vec) int {
	th := math.Atan2(radial.Dot(g.refV), radial.Dot(g.refU))
	ang := g.sweep.classify(th, k.tol/math.Max(g.radius, 1e-30))
	if ang == -1 {
		return -1
	}
	p0 := g.anchor.Add(radial.Scale(g.radius)).Add(g.axis.Scale(g.zWin.lo))
	p1 := g.anchor.Add(radial.Scale(g.radius)).Add(g.axis.Scale(g.zWin.hi))
	x0, y0 := f.planeCoords(p0)
	x1, y1 := f.planeCoords(p1)
	hit, _ := f.region.segmentHits(x0, y0, x1, y1)
	return admitState(ang, hit)
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
	if k.parallel(f.n, g.axis) == degYes {
		// Axis EXACTLY perpendicular to the plane: the crossing is a circle at
		// the plane's own axial station, exact against the region. A tilted
		// axis crosses in an ELLIPSE of semi-major radius R/|s| — reading it
		// as the circle would grant an exclusion the ellipse does not earn, so
		// a tilt the oracle cannot rule out leaves the crossing undecided.
		center := g.anchor.Add(g.axis.Scale((c - base) / s))
		rel := trimmedCircleCrossingRelation(f, g, center, g.radius, k.tol)
		if rel == -1 {
			return
		}
		sink.crossing(rel)
		return
	}
	sink.unsure = true
}

// planeCone is the plane-row cone cell, and it owes only the crossing
// exclusion — the exact range of n·x over the trimmed face.
//
// The face-interior plateau §4 grants this cell (the plane parallel to a
// ruling, |n·axis| = sin α, at the apex's own plane distance) is NOT emitted,
// and the cell loses nothing by it. Two reasons, and both must hold:
//
//   - It cannot be certified. |n·axis| is exact arithmetic on the payload's
//     floats, but sin α is a transcendental of the stored half-angle: no exact
//     test on those floats decides the identity, so the plateau could only ever
//     be minted off a tolerance — an Exact reading a tilt undercuts by up to
//     clrAngTol × the slant length, exactly the lie this kernel does not tell.
//   - It is redundant. The plane distance is AFFINE along every ruling, so its
//     minimum over the trimmed face always migrates to a trim boundary in the
//     axial direction: the latitude edges (Circle3 × Plane, closed form) and
//     the synthesized apex vertex (point × plane, closed form) hold it exactly,
//     including in the parallel-ruling case where the plateau merely ties them.
//     A zero crossing inside the trim is not a minimum but a contact, and that
//     is what the crossing exclusion below is for.
func (k *pairKernel) planeCone(f, g *cFace, sink *cellSink) {
	s := f.n.Dot(g.axis)
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
	if rangeLo > k.tol || rangeHi < -k.tol || clrBoxDist(f.box, g.box) > k.tol {
		return
	}
	sink.unsure = true
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
		rel := trimmedCircleCrossingRelation(f, g, foot, rc, k.tol)
		if rel == -1 {
			return
		}
		sink.crossing(rel)
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
		// Tangency zone: not certified here.
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
	sinA, cosA := math.Sincos(cone.half)
	var radial r3.Vec
	switch k.onAxis(sph.anchor, cone.anchor, cone.axis) {
	case degYes:
		// A center PROVENLY on the axis makes every azimuth equivalent — the
		// meridian reading is the same all the way around — so the sweep
		// window's own midpoint represents the family.
		mid := 0.0
		if !cone.sweep.full {
			mid = (cone.sweep.lo + cone.sweep.hi) / 2
		}
		radial = cone.refU.Scale(math.Cos(mid)).Add(cone.refV.Scale(math.Sin(mid)))
	case degNo:
		radial, _ = perp.Normalize()
	default:
		// The value is azimuth-free, but the FOOT that decides trim admission
		// is not: an offset in the undecided band normalizes to a garbage
		// direction. The carrier distance still bounds the trimmed pair from
		// below, so it stands as the honest lower bound it is.
		sink.loOnly(math.Abs(rho*cosA-z*sinA) - sph.radius)
		return
	}
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
