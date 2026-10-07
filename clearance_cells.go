package decad

import (
	"cmp"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/clearance/spine"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

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

// cellSink accumulates contributions and the undecidable findings.
type cellSink struct {
	contribs []clearance.GapContrib
	// overlap is set only by a trim-admitted transversal boundary crossing.
	// Such a crossing proves a shared open material neighborhood.
	overlap bool
	// unsure is set when a cell meets a question it cannot decide: an
	// admitted-or-ambiguous carrier crossing, an uncertified contact, an
	// equality where a branch demands strictness (§4: equality routes to §6,
	// where only the coplanar plane pair is certified).
	unsure bool

	// prune enables §5's cell pruning (pruned, below). A zero-value sink
	// never prunes, so a cell run on its own reports everything it finds.
	prune bool
	// margin is the length charged against a box distance before it may
	// prune. The kernel passes its slack, 1e-9 × the pair's coordinate
	// scale, which covers the few-ulp rounding of the float boxes, of the
	// box distance and of the subtraction many times over.
	margin float64
	// best is the smallest finite contribution hi among contribs[:seen].
	best float64
	seen int
	// skipped counts the cells pruned so far.
	skipped int
}

// newPruningSink returns a sink that prunes against its own best upper
// bound, charging margin against every box distance.
func newPruningSink(margin float64) *cellSink {
	return &cellSink{prune: true, margin: margin, best: math.Inf(1)}
}

// pruned reports whether a cell whose two features' boxes lie lb apart
// cannot hold the pair's minimum, and counts it when so (§5). Every
// contribution's hi bounds the true gap from above, and every point of the
// cell's features lies at least lb − margin from the other's, so a cell with
// lb − margin STRICTLY above the best hi in hand lies wholly beyond the
// minimum. Equality never prunes: a feature pair at exactly the best upper
// bound may hold the minimum itself. A non-finite lb never prunes.
func (s *cellSink) pruned(lb float64) bool {
	if !s.prune || proofbound.IsNonFinite(lb) {
		return false
	}
	for _, c := range s.contribs[s.seen:] {
		if c.Hi < s.best {
			s.best = c.Hi
		}
	}
	s.seen = len(s.contribs)
	if lb-s.margin <= s.best {
		return false
	}
	s.skipped++
	return true
}

// interval folds the contributions into the held-candidate gap interval
// [lo, hi] (§1): hi is the least upper bound, lo the least lower bound below
// it, and exact holds only for a closed-form winner at hi with every rival's
// lo at or above it. ok is false when no contribution carries a finite hi.
func (s *cellSink) interval() (float64, float64, bool, bool) {
	hi := math.Inf(1)
	for _, c := range s.contribs {
		if c.Hi < hi {
			hi = c.Hi
		}
	}
	if math.IsInf(hi, 1) {
		return 0, 0, false, false
	}
	lo := hi
	for _, c := range s.contribs {
		if c.Lo < lo {
			lo = c.Lo
		}
	}
	exact := false
	for _, c := range s.contribs {
		if c.Exact && c.Lo == hi && c.Hi == hi {
			exact = true
		}
	}
	return lo, hi, exact && lo == hi, true
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
//
//nolint:unparam // Keep witness arguments while preserving every caller's admission and evaluation path.
func (s *cellSink) candidate(k *pairKernel, admit int, lo, hi float64, exact bool, pa, pb r3.Vec) {
	if admit == -1 {
		return
	}
	if lo <= k.tol {
		s.unsure = true
		return
	}
	if admit == 0 {
		s.contribs = append(s.contribs, clearance.GapContrib{Lo: lo, Hi: math.Inf(1)})
		return
	}
	s.contribs = append(s.contribs, clearance.GapContrib{Lo: lo, Hi: hi, Exact: exact})
}

// loOnly contributes a bare proven lower bound.
func (s *cellSink) loOnly(lo float64) {
	s.contribs = append(s.contribs, clearance.GapContrib{Lo: math.Max(0, lo), Hi: math.Inf(1)})
}

// coarse contributes a conservative enclosure for a pair no shipped cell can
// solve: the boxes' distance below, the closest admitted witness pair above
// (§5 — enclosure distance never exceeds true distance, a witness is always
// an upper bound).
func (s *cellSink) coarse(boxA, boxB [2]r3.Vec, witA, witB []r3.Vec) {
	lo := clearance.ClrBoxDist(boxA, boxB)
	hi := math.Inf(1)
	for _, wa := range witA {
		for _, wb := range witB {
			if d := wa.Sub(wb).Len(); d < hi {
				hi = d
			}
		}
	}
	s.contribs = append(s.contribs, clearance.GapContrib{Lo: math.Max(0, lo), Hi: hi})
}

// The feature-pair cell kinds enumerate sorts by box distance.
const (
	cellFF uint8 = iota // a face × b face
	cellFE              // a face × b edge
	cellEF              // b face × a edge
	cellEE              // a edge × b edge
)

// featureCell is one face/edge cell queued for the sorted walk: its kind,
// the two feature indices, and the distance between the features' boxes.
type featureCell struct {
	lb   float64
	kind uint8
	i, j int
}

// enumerate runs every tier over the pair, pruning per §5. One shared budget
// bounds cancellation latency across the cell queue build, the outer
// candidate walk, and the nested work performed by a vertex tier.
//
// The order is chosen so good upper bounds arrive early, and it is fixed, so
// a replay prunes identically: the vertex × vertex distances first (cheap and
// exact), then the vertex tiers (closed form), then every face/edge cell in
// ascending box distance, ties in the fixed face × face, face × edge,
// edge × face, edge × edge order. Within that last walk a cell is pruned
// exactly when its box distance, less the margin, exceeds the final best
// upper bound: every cell nearer than it has already run, the one holding
// the best hi among them.
func (k *pairKernel) enumerate() (*cellSink, error) {
	return k.enumerateInto(newPruningSink(k.slack))
}

// enumerateInto runs enumerate's walk into the given sink. A zero-value sink
// never prunes, so it runs every cell.
func (k *pairKernel) enumerateInto(sink *cellSink) (*cellSink, error) {
	budget := proofbound.NewWorkBudget(k.ctx)
	check := func() error {
		if k.err != nil {
			return k.err
		}
		return budget.Step()
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
	cells, err := k.featureCells(budget)
	if err != nil {
		return nil, err
	}
	for _, c := range cells {
		if err := check(); err != nil {
			return nil, err
		}
		if sink.pruned(c.lb) {
			continue
		}
		switch c.kind {
		case cellFF:
			k.ffCell(k.a.faces[c.i], k.b.faces[c.j], sink)
		case cellFE:
			k.feCell(k.a.faces[c.i], k.b.edges[c.j], sink)
		case cellEF:
			k.feCell(k.b.faces[c.i], k.a.edges[c.j], sink)
		default:
			k.eeCell(k.a.edges[c.i], k.b.edges[c.j], sink)
		}
	}
	if k.err != nil {
		return nil, k.err
	}
	if err := budget.Err(); err != nil {
		return nil, err
	}
	if k.clearanceRefused {
		sink.unsure = true
	}
	return sink, nil
}

// featureCells queues every face × face, face × edge, edge × face and
// edge × edge cell with its box distance, sorted ascending. The sort is
// stable, so ties keep the queue's own fixed order.
func (k *pairKernel) featureCells(budget *proofbound.WorkBudget) ([]featureCell, error) {
	a, b := k.a, k.b
	n := len(a.faces)*(len(b.faces)+len(b.edges)) + len(b.faces)*len(a.edges) + len(a.edges)*len(b.edges)
	cells := make([]featureCell, 0, n)
	push := func(kind uint8, i, j int, boxA, boxB [2]r3.Vec) error {
		if err := budget.Step(); err != nil {
			return err
		}
		cells = append(cells, featureCell{lb: clearance.ClrBoxDist(boxA, boxB), kind: kind, i: i, j: j})
		return nil
	}
	for i, fa := range a.faces {
		for j, fb := range b.faces {
			if err := push(cellFF, i, j, fa.Box, fb.Box); err != nil {
				return nil, err
			}
		}
	}
	for i, fa := range a.faces {
		for j, eb := range b.edges {
			if err := push(cellFE, i, j, fa.Box, eb.Box); err != nil {
				return nil, err
			}
		}
	}
	for i, fb := range b.faces {
		for j, ea := range a.edges {
			if err := push(cellEF, i, j, fb.Box, ea.Box); err != nil {
				return nil, err
			}
		}
	}
	for i, ea := range a.edges {
		for j, eb := range b.edges {
			if err := push(cellEE, i, j, ea.Box, eb.Box); err != nil {
				return nil, err
			}
		}
	}
	slices.SortStableFunc(cells, func(x, y featureCell) int { return cmp.Compare(x.lb, y.lb) })
	return cells, nil
}

// ffCell dispatches one face pair through the §4 table.
func (k *pairKernel) ffCell(f, g *clearance.CFace, sink *cellSink) {
	if g.Kind < f.Kind {
		f, g = g, f
	}
	switch {
	case f.Kind == clearance.CkPlane && g.Kind == clearance.CkPlane:
		k.planePlane(f, g, sink)
	case f.Kind == clearance.CkPlane && g.Kind == clearance.CkCylinder:
		k.planeCylinder(f, g, sink)
	case f.Kind == clearance.CkPlane && g.Kind == clearance.CkCone:
		k.planeCone(f, g, sink)
	case f.Kind == clearance.CkPlane && g.Kind == clearance.CkSphere:
		k.planeSphere(f, g, sink)
	case f.Kind == clearance.CkPlane && g.Kind == clearance.CkTorus:
		k.planeTorus(f, g, sink)
	case f.Kind == clearance.CkCone || g.Kind == clearance.CkCone:
		// Cone × sphere stays closed form (§4); everything else
		// cone-involved takes the coarse enclosure — the face-box distance
		// below, the closest witness pair above.
		if f.Kind == clearance.CkSphere || g.Kind == clearance.CkSphere {
			k.coneSphere(f, g, sink)
			return
		}
		sink.coarse(f.Box, g.Box, f.Wit, g.Wit)
	case (f.Kind == clearance.CkTorus && f.Spindle) || (g.Kind == clearance.CkTorus && g.Spindle):
		// A Minor ≥ Major torus leaves the polynomial path (§4): the pair
		// takes the coarse enclosure — the face-box distance below, the
		// closest witness pair above.
		sink.coarse(f.Box, g.Box, f.Wit, g.Wit)
	default:
		k.offsetPair(f, g, sink)
	}
}

// offsetPair is the spine-offset reduction of §4 for cylinder/sphere/torus
// pairs: enumerate the spine-pair criticals, emit every offset combination
// per critical, then decide whether a carrier crossing is excluded — by the
// strict exterior branch, by a certified containment (§4's d_sup list: an
// inner point spine, parallel cylinder axes, coaxial spines), or by face-box
// separation; otherwise the pair is undecided.
func (k *pairKernel) offsetPair(f, g *clearance.CFace, sink *cellSink) {
	crits, ok := k.spineCriticals(f, g)
	if !ok {
		sink.coarse(f.Box, g.Box, f.Wit, g.Wit)
		return
	}
	minLo := math.Inf(1)
	for _, c := range crits {
		minLo = math.Min(minLo, c.Lo)
		k.emitOffsetCombos(sink, f, g, c)
	}
	rf, rg := clearance.SpineOffset(f), clearance.SpineOffset(g)
	if minLo > rf+rg+k.tol {
		return // strict exterior: the carriers never meet (§4)
	}
	if k.oracle().CertifiedContainment(f, g) {
		return
	}
	if clearance.ClrBoxDist(f.Box, g.Box) > k.tol {
		return // the trimmed faces provably cannot touch
	}
	sink.unsure = true
}

func (k *pairKernel) spineEngine() *spine.Engine {
	return &spine.Engine{Context: k.ctx, Tolerance: k.tol, Slack: k.slack}
}

func (k *pairKernel) captureSpine(e *spine.Engine) {
	if e.Err != nil {
		k.err = e.Err
	}
	k.clearanceRefused = k.clearanceRefused || e.Refused
}

func (k *pairKernel) spineCriticals(f, g *clearance.CFace) ([]clearance.SpineCrit, bool) {
	e := k.spineEngine()
	out, ok := e.SpineCriticals(f, g)
	k.captureSpine(e)
	return out, ok
}

func (k *pairKernel) pointCircleCrits(p, c, axis, refU, refV r3.Vec, rad float64, win clearance.AngWindow) ([]clearance.SpineCrit, bool) {
	e := k.spineEngine()
	out, ok := e.PointCircleCrits(p, c, axis, refU, refV, rad, win)
	k.captureSpine(e)
	return out, ok
}

func (k *pairKernel) lineCircleBracketCrits(cp freeform.CircleParam, center, refU, refV, la, ld r3.Vec) ([]clearance.SpineCrit, bool) {
	e := k.spineEngine()
	out, ok := e.LineCircleBracketCrits(cp, center, refU, refV, la, ld)
	k.captureSpine(e)
	return out, ok
}

func (k *pairKernel) circleCircleCrits(f, g *clearance.CFace) ([]clearance.SpineCrit, bool) {
	e := k.spineEngine()
	out, ok := e.CircleCircleCrits(f, g)
	k.captureSpine(e)
	return out, ok
}

// emitOffsetCombos emits the four offset combinations of one spine-pair
// critical: the joining line meets each offset surface twice, and every
// carrier-pair stationary point lies among the combinations, so admission
// (§3) decides each in isolation.
func (k *pairKernel) emitOffsetCombos(sink *cellSink, f, g *clearance.CFace, c clearance.SpineCrit) {
	sep := c.Fb.Sub(c.Fa)
	d := sep.Len()
	if d <= k.tol {
		// Coincident spine feet: the joining direction degenerates into a
		// whole ring. Only a PROVEN ring family carries it (concentric
		// shells, coaxial cylinders — the peg-in-hole reading); a
		// near-coincidence the oracle cannot decide is a contact question
		// this cell has no right to answer.
		if k.oracle().RingFamily(f, g) == clearance.DegYes {
			k.emitRingCombos(sink, f, g, c)
			return
		}
		sink.unsure = true
		return
	}
	dir := sep.Scale(1 / d)
	rf, rg := clearance.SpineOffset(f), clearance.SpineOffset(g)
	margin := k.tol + (c.Hi - c.Lo)
	for _, sf := range []float64{1, -1} {
		for _, sg := range []float64{1, -1} {
			pf := c.Fa.Add(dir.Scale(sf * rf))
			pg := c.Fb.Sub(dir.Scale(sg * rg))
			rawLo := c.Lo - sf*rf - sg*rg
			rawHi := c.Hi - sf*rf - sg*rg
			admit := clearance.AdmitState(f.AdmitPoint(pf, margin), g.AdmitPoint(pg, margin))
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
			sink.candidate(k, admit, lo, hi, c.Exact, pf, pg)
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
func (k *pairKernel) emitRingCombos(sink *cellSink, f, g *clearance.CFace, c clearance.SpineCrit) {
	axis := f.Axis
	if clearance.SpineOf(f) == 0 && clearance.SpineOf(g) == 1 {
		axis = g.Axis
	}
	if clearance.SpineOf(f) == 0 && clearance.SpineOf(g) == 0 {
		axis = clearance.PerpTo(c.Fa.Sub(c.Fb).Add(r3.NewVec(0, 0, 1)))
	}
	u := clearance.PerpTo(axis)
	v := axis.Cross(u)
	rf, rg := clearance.SpineOffset(f), clearance.SpineOffset(g)
	dirs := make([]r3.Vec, 0, 24)
	for _, th := range clearance.RingAngles(f, g, u, v) {
		dirs = append(dirs, u.Scale(math.Cos(th)).Add(v.Scale(math.Sin(th))))
	}
	if clearance.SpineOf(f) == 0 && clearance.SpineOf(g) == 0 {
		dirs = append(dirs, axis, axis.Scale(-1))
	}
	margin := k.tol + (c.Hi - c.Lo)
	nearAdmitted := false
	for _, dir := range dirs {
		pf := c.Fa.Add(dir.Scale(rf))
		near := c.Fb.Add(dir.Scale(rg))
		far := c.Fb.Sub(dir.Scale(rg))
		if a := clearance.AdmitState(f.AdmitPoint(pf, margin), g.AdmitPoint(near, margin)); a != -1 {
			nearAdmitted = true
			sink.candidate(k, a, math.Abs(rf-rg), math.Abs(rf-rg), c.Exact, pf, near)
		}
		if a := clearance.AdmitState(f.AdmitPoint(pf, margin), g.AdmitPoint(far, margin)); a != -1 {
			sink.candidate(k, a, rf+rg, rf+rg, c.Exact, pf, far)
		}
	}
	if !nearAdmitted {
		// The near pairing is the small one: a far pairing can never hold the
		// minimum the near pairing does not. Unproven absence keeps its bound.
		sink.loOnly(math.Abs(rf - rg))
	}
}

// planePlane is the plane-row cell for two planes: a coplanar pair reaching
// here is uncertified (the coplanar contact certificate runs at the
// pair level first); a parallel distinct pair carries its plateau exactly
// where the trims overlap in projection (§3's projection rule); crossing
// carriers are excluded through the trims or read as the §6/§7-routed
// contact, never quietly as a boundary answer.
func (k *pairKernel) planePlane(f, g *clearance.CFace, sink *cellSink) {
	// The plateau exists only for EXACT parallelism: a tolerance here
	// blesses a tilted pair's mid-face height as an Exact minimum the true
	// gap undercuts (or exceeds). A tilt too small for the crossing-line
	// path to certify falls to unsure there — never a wrong Exact.
	if f.N.Cross(g.N).Len() == 0 {
		h := g.O.Sub(f.O).Dot(f.N)
		rel, wit, err := k.coplanarRelation(proofbound.NewWorkBudget(k.ctx), f, g)
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
			pa := f.O.Add(f.U.Scale(wit[0])).Add(f.V.Scale(wit[1]))
			pb := pa.Add(f.N.Scale(h))
			sink.candidate(k, 1, math.Abs(h), math.Abs(h), true, pa, pb)
		case 0:
			sink.loOnly(math.Abs(h))
		}
		return
	}
	if clearance.ClrBoxDist(f.Box, g.Box) > k.tol {
		return
	}
	// The intersection line, clipped by both trims on a shared parameter.
	dir, ok := f.N.Cross(g.N).Normalize()
	if !ok {
		sink.unsure = true
		return
	}
	p0, ok := clearance.PlanesIntersect(f, g)
	if !ok {
		sink.unsure = true
		return
	}
	fx, fy := f.PlaneCoords(p0)
	gx, gy := g.PlaneCoords(p0)
	// Exact trim intervals certify positive-length transversal crossing. If
	// either trim cannot classify the line, conservative supersets still prove
	// a clean miss; every remaining case stays undecided.
	ivF, okF := f.Region.LineIntervals(fx, fy, dir.Dot(f.U), dir.Dot(f.V))
	ivG, okG := g.Region.LineIntervals(gx, gy, dir.Dot(g.U), dir.Dot(g.V))
	if okF && okG {
		sink.crossing(clearance.IntervalsMeet(ivF, ivG, k.tol))
		return
	}
	supF := f.Region.LineIntervalsSuperset(fx, fy, dir.Dot(f.U), dir.Dot(f.V))
	supG := g.Region.LineIntervalsSuperset(gx, gy, dir.Dot(g.U), dir.Dot(g.V))
	if clearance.IntervalsMeet(supF, supG, k.tol) != -1 {
		sink.unsure = true
	}
}

// coplanarRelation classifies two parallel-plane trims in projection along
// the normal: +1 proven positive-area overlap (with a witness point in f's
// frame), −1 provenly apart, 0 ambiguous. Sample-based in the sufficient
// direction, boundary-clearance-based in the exclusion direction — never a
// blessed ambiguity.
func (k *pairKernel) coplanarRelation(budget *proofbound.WorkBudget, f, g *clearance.CFace) (int, [2]float64, error) {
	if err := budget.Err(); err != nil {
		return 0, [2]float64{}, err
	}
	ge := make([]survey2d.SurveyElem, 0, len(g.Region.Elems))
	for _, e := range g.Region.Elems {
		if err := budget.Step(); err != nil {
			return 0, [2]float64{}, err
		}
		ge = append(ge, clearance.TransformElem(e, g, f))
	}
	greg, err := clearance.NewRegion2Budget(budget, ge)
	if err != nil {
		return 0, [2]float64{}, err
	}
	tol := math.Max(f.Region.Tol(), greg.Tol())

	probeInto := func(src, dst clearance.Region2) (int, [2]float64, error) {
		p, ok, err := clearance.RegionInteriorPointBudget(budget, src)
		if err != nil {
			return 0, [2]float64{}, err
		}
		if ok {
			class, err := clearance.RegionClassifyBudget(budget, dst, p[0], p[1], tol)
			if err != nil {
				return 0, [2]float64{}, err
			}
			if class == 1 {
				return 1, p, nil
			}
		}
		samples, err := clearance.RegionSamplesBudget(budget, src)
		if err != nil {
			return 0, [2]float64{}, err
		}
		for _, s := range samples {
			if err := budget.Step(); err != nil {
				return 0, [2]float64{}, err
			}
			class, err := clearance.RegionClassifyBudget(budget, dst, s[0], s[1], tol)
			if err != nil {
				return 0, [2]float64{}, err
			}
			if class == 1 {
				return 1, s, nil
			}
		}
		return 0, [2]float64{}, nil
	}
	r, w, err := probeInto(greg, f.Region)
	if err != nil {
		return 0, [2]float64{}, err
	}
	if r == 1 {
		return 1, w, nil
	}
	r, w, err = probeInto(f.Region, greg)
	if err != nil {
		return 0, [2]float64{}, err
	}
	if r == 1 {
		return 1, w, nil
	}
	// Exclusion: boundaries clear each other and neither contains the other.
	clearing, err := clearance.CoplanarBoundaryClearanceBudget(budget, f.Region, greg)
	if err != nil {
		return 0, [2]float64{}, err
	}
	if clearing <= tol {
		return 0, [2]float64{}, budget.Err()
	}
	outsideFG, err := clearance.RegionSampleOutsideBudget(budget, f.Region, greg, tol)
	if err != nil {
		return 0, [2]float64{}, err
	}
	if !outsideFG {
		return 0, [2]float64{}, budget.Err()
	}
	outsideGF, err := clearance.RegionSampleOutsideBudget(budget, greg, f.Region, tol)
	if err != nil {
		return 0, [2]float64{}, err
	}
	if outsideGF {
		return -1, [2]float64{}, nil
	}
	return 0, [2]float64{}, budget.Err()
}

// planeCylinder is the plane-row cell for a cylinder: an axis-parallel pair
// carries the constant ruling plateau; a crossing carrier is excluded
// through the trims (the two crossing rulings when the axis parallels the
// plane, the exact axial crossing range otherwise) or routed to §6/§7.
func (k *pairKernel) planeCylinder(f, g *clearance.CFace, sink *cellSink) {
	s := f.N.Dot(g.Axis)
	if s != 0 {
		// The constant-ruling plateau exists only for EXACT axis
		// parallelism: a tolerance here would bless a tilted cylinder's
		// plateau as an Exact reading the true minimum undercuts. A tilt
		// too small for the crossing machinery to certify falls through
		// to unsure there — never a wrong Exact.
		k.planeCrossesRevolved(f, g, sink)
		return
	}
	hAxis := g.Anchor.Sub(f.O).Dot(f.N)
	d := math.Abs(hAxis)
	side := 1.0
	if hAxis < 0 {
		side = -1
	}
	toward := f.N.Scale(-side) // radial direction from the axis toward the plane
	switch {
	case d-g.Radius > k.tol:
		for _, dirSign := range []float64{1, -1} {
			radial := toward.Scale(dirSign)
			v := d - dirSign*g.Radius
			k.rulingCandidate(f, g, radial, v, sink)
		}
	case g.Radius-d > k.tol:
		// Crossing along two rulings at ±acos(d/R) around the toward
		// direction.
		half := math.Acos(math.Max(-1, math.Min(1, d/g.Radius)))
		base := math.Atan2(toward.Dot(g.RefV), toward.Dot(g.RefU))
		ambiguous := false
		for _, dth := range []float64{half, -half} {
			th := base + dth
			radial := g.RefU.Scale(math.Cos(th)).Add(g.RefV.Scale(math.Sin(th)))
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
func (k *pairKernel) rulingCandidate(f, g *clearance.CFace, radial r3.Vec, v float64, sink *cellSink) {
	th := math.Atan2(radial.Dot(g.RefV), radial.Dot(g.RefU))
	angAdmit := g.Sweep.Classify(th, k.tol/math.Max(g.Radius, 1e-30))
	if angAdmit == -1 {
		return
	}
	p0 := g.Anchor.Add(radial.Scale(g.Radius)).Add(g.Axis.Scale(g.ZWin.Lo))
	p1 := g.Anchor.Add(radial.Scale(g.Radius)).Add(g.Axis.Scale(g.ZWin.Hi))
	x0, y0 := f.PlaneCoords(p0)
	x1, y1 := f.PlaneCoords(p1)
	hit, w := f.Region.SegmentHits(x0, y0, x1, y1)
	if hit == -1 {
		return
	}
	admit := clearance.AdmitState(angAdmit, hit)
	pa := f.O.Add(f.U.Scale(w[0])).Add(f.V.Scale(w[1]))
	h := f.N.Dot(p0.Sub(f.O))
	pb := pa.Add(f.N.Scale(h))
	sink.candidate(k, admit, v, v, true, pa, pb)
}

// rulingRelation classifies one carrier-crossing ruling through both trims.
func (k *pairKernel) rulingRelation(f, g *clearance.CFace, radial r3.Vec) int {
	th := math.Atan2(radial.Dot(g.RefV), radial.Dot(g.RefU))
	ang := g.Sweep.Classify(th, k.tol/math.Max(g.Radius, 1e-30))
	if ang == -1 {
		return -1
	}
	p0 := g.Anchor.Add(radial.Scale(g.Radius)).Add(g.Axis.Scale(g.ZWin.Lo))
	p1 := g.Anchor.Add(radial.Scale(g.Radius)).Add(g.Axis.Scale(g.ZWin.Hi))
	x0, y0 := f.PlaneCoords(p0)
	x1, y1 := f.PlaneCoords(p1)
	hit, _ := f.Region.SegmentHits(x0, y0, x1, y1)
	return clearance.AdmitState(ang, hit)
}

// planeCrossesRevolved excludes (or reports) the crossing of a plane with a
// tilted revolution carrier: face boxes first, then the exact axial range of
// the carrier crossing against the axial window, then — for a perpendicular
// axis — the exact crossing circle against the plane trim.
func (k *pairKernel) planeCrossesRevolved(f, g *clearance.CFace, sink *cellSink) {
	if clearance.ClrBoxDist(f.Box, g.Box) > k.tol {
		return
	}
	s := f.N.Dot(g.Axis)
	c := f.PlaneOffset()
	base := f.N.Dot(g.Anchor)
	lo, hi := 0.0, 2*math.Pi
	if !g.Sweep.Full {
		lo, hi = g.Sweep.Lo, g.Sweep.Hi
	}
	mn, mx := trigRange(f.N.Dot(g.RefU)*g.Radius, f.N.Dot(g.RefV)*g.Radius, lo, hi)
	z0 := (c - base - mx) / s
	z1 := (c - base - mn) / s
	zw := clearance.NewLinWindow(z0, z1)
	if zw.Hi < g.ZWin.Lo-k.tol || zw.Lo > g.ZWin.Hi+k.tol {
		return
	}
	if k.oracle().Parallel(f.N, g.Axis) == clearance.DegYes {
		// Axis EXACTLY perpendicular to the plane: the crossing is a circle at
		// the plane's own axial station, exact against the region. A tilted
		// axis crosses in an ELLIPSE of semi-major radius R/|s| — reading it
		// as the circle would grant an exclusion the ellipse does not earn, so
		// a tilt the oracle cannot rule out leaves the crossing undecided.
		center := g.Anchor.Add(g.Axis.Scale((c - base) / s))
		rel := clearance.TrimmedCircleCrossingRelation(f, g, center, g.Radius, k.tol)
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
//     clearance.ClrAngTol × the slant length, exactly the lie this kernel does not tell.
//   - It is redundant. The plane distance is AFFINE along every ruling, so its
//     minimum over the trimmed face always migrates to a trim boundary in the
//     axial direction: the latitude edges (Circle3 × Plane, closed form) and
//     the synthesized apex vertex (point × plane, closed form) hold it exactly,
//     including in the parallel-ruling case where the plateau merely ties them.
//     A zero crossing inside the trim is not a minimum but a contact, and that
//     is what the crossing exclusion below is for.
func (k *pairKernel) planeCone(f, g *clearance.CFace, sink *cellSink) {
	s := f.N.Dot(g.Axis)
	apexH := g.Anchor.Sub(f.O).Dot(f.N)
	nu, nv := f.N.Dot(g.RefU), f.N.Dot(g.RefV)

	// Exact range of n·x − c over the trimmed face.
	lo, hi := 0.0, 2*math.Pi
	if !g.Sweep.Full {
		lo, hi = g.Sweep.Lo, g.Sweep.Hi
	}
	mn, mx := trigRange(nu, nv, lo, hi)
	tanA := math.Tan(g.Half)
	rangeLo, rangeHi := math.Inf(1), math.Inf(-1)
	for _, z := range []float64{g.ZWin.Lo, g.ZWin.Hi} {
		for _, m := range []float64{mn, mx} {
			v := apexH + z*(s+tanA*m)
			rangeLo = math.Min(rangeLo, v)
			rangeHi = math.Max(rangeHi, v)
		}
	}
	if rangeLo > k.tol || rangeHi < -k.tol || clearance.ClrBoxDist(f.Box, g.Box) > k.tol {
		return
	}
	sink.unsure = true
}

// planeSphere is the plane-row sphere cell: the center's plane distance
// minus the radius, tangency routed to §6, a crossing excluded through the
// trims (the exact crossing circle) or undecided.
func (k *pairKernel) planeSphere(f, g *clearance.CFace, sink *cellSink) {
	h := g.Anchor.Sub(f.O).Dot(f.N)
	d := math.Abs(h)
	side := 1.0
	if h < 0 {
		side = -1
	}
	switch {
	case d-g.Radius > k.tol:
		for _, dirSign := range []float64{1, -1} {
			pb := g.Anchor.Sub(f.N.Scale(side * dirSign * g.Radius))
			foot := pb.Sub(f.N.Scale(f.N.Dot(pb.Sub(f.O))))
			x, y := f.PlaneCoords(foot)
			admit := clearance.AdmitState(f.Region.Classify(x, y, k.tol), g.AdmitPoint(pb, k.tol))
			sink.candidate(k, admit, d-dirSign*g.Radius, d-dirSign*g.Radius, true, foot, pb)
		}
	case g.Radius-d > k.tol:
		if clearance.ClrBoxDist(f.Box, g.Box) > k.tol {
			return
		}
		rc := math.Sqrt(g.Radius*g.Radius - h*h)
		foot := g.Anchor.Sub(f.N.Scale(h))
		rel := clearance.TrimmedCircleCrossingRelation(f, g, foot, rc, k.tol)
		if rel == -1 {
			return
		}
		sink.crossing(rel)
	default:
		pb := g.Anchor.Sub(f.N.Scale(side * g.Radius))
		foot := pb.Sub(f.N.Scale(f.N.Dot(pb.Sub(f.O))))
		x, y := f.PlaneCoords(foot)
		if clearance.AdmitState(f.Region.Classify(x, y, k.tol), g.AdmitPoint(pb, k.tol)) == -1 {
			return
		}
		sink.unsure = true
	}
}

// planeTorus is the plane-row torus cell: the spine circle's plane-distance
// amplitude extreme minus the minor radius — valid for the spindle branch
// too, since the extreme's meridian direction always lies in the extreme
// spine point's own meridian plane.
func (k *pairKernel) planeTorus(f, g *clearance.CFace, sink *cellSink) {
	base := g.Anchor.Sub(f.O).Dot(f.N)
	au := g.Major * f.N.Dot(g.RefU)
	av := g.Major * f.N.Dot(g.RefV)
	lo, hi := 0.0, 2*math.Pi
	if !g.Sweep.Full {
		lo, hi = g.Sweep.Lo, g.Sweep.Hi
	}
	mn, mx := trigRange(au, av, lo, hi)
	hLo, hHi := base+mn, base+mx
	if hLo-g.Radius > k.tol || -hHi-g.Radius > k.tol {
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
		if g.Sweep.Classify(star, clearance.ClrAngTol*10) != 1 && !g.Sweep.Full {
			sink.loOnly(math.Min(math.Abs(hLo), math.Abs(hHi)) - g.Radius)
			return
		}
		sp := g.Anchor.Add(g.RefU.Scale(g.Major * math.Cos(star))).Add(g.RefV.Scale(g.Major * math.Sin(star)))
		pb := sp.Sub(f.N.Scale(side * g.Radius))
		foot := pb.Sub(f.N.Scale(f.N.Dot(pb.Sub(f.O))))
		x, y := f.PlaneCoords(foot)
		v := math.Min(math.Abs(hLo), math.Abs(hHi)) - g.Radius
		admit := clearance.AdmitState(f.Region.Classify(x, y, k.tol), g.AdmitPoint(pb, k.tol))
		sink.candidate(k, admit, v, v, true, foot, pb)
		return
	}
	if clearance.ClrBoxDist(f.Box, g.Box) > k.tol {
		return
	}
	if hLo > g.Radius-k.tol || hHi < -(g.Radius-k.tol) {
		// Tangency zone: not certified here.
		sink.unsure = true
		return
	}
	sink.unsure = true
}

// coneSphere is the closed-form meridian point/cone cell of §4's sphere
// column: the sphere center against the cone's generating ray in its own
// meridian half-plane, offset by the sphere radius on the strict branches.
func (k *pairKernel) coneSphere(f, g *clearance.CFace, sink *cellSink) {
	cone, sph := f, g
	if cone.Kind != clearance.CkCone {
		cone, sph = g, f
	}
	rel := sph.Anchor.Sub(cone.Anchor)
	z := rel.Dot(cone.Axis)
	perp := rel.Sub(cone.Axis.Scale(z))
	rho := perp.Len()
	sinA, cosA := math.Sincos(cone.Half)
	var radial r3.Vec
	switch k.oracle().OnAxis(sph.Anchor, cone.Anchor, cone.Axis) {
	case clearance.DegYes:
		// A center PROVENLY on the axis makes every azimuth equivalent — the
		// meridian reading is the same all the way around — so the sweep
		// window's own midpoint represents the family.
		mid := 0.0
		if !cone.Sweep.Full {
			mid = (cone.Sweep.Lo + cone.Sweep.Hi) / 2
		}
		radial = cone.RefU.Scale(math.Cos(mid)).Add(cone.RefV.Scale(math.Sin(mid)))
	case clearance.DegNo:
		radial, _ = perp.Normalize()
	default:
		// The value is azimuth-free, but the FOOT that decides trim admission
		// is not: an offset in the undecided band normalizes to a garbage
		// direction. The carrier distance still bounds the trimmed pair from
		// below, so it stands as the honest lower bound it is.
		sink.loOnly(math.Abs(rho*cosA-z*sinA) - sph.Radius)
		return
	}
	t := z*cosA + rho*sinA // slant projection onto the ruling
	if t <= k.tol {
		// The nearest carrier point is the apex — a singular point owned by
		// the synthesized vertex tier; the interior cell has no critical.
		if clearance.ClrBoxDist(f.Box, g.Box) <= k.tol && z*z+rho*rho <= (sph.Radius+k.tol)*(sph.Radius+k.tol) {
			sink.unsure = true
		}
		return
	}
	dCarrier := math.Abs(rho*cosA - z*sinA)
	v := dCarrier - sph.Radius
	pf := cone.Anchor.Add(cone.Axis.Scale(t * cosA)).Add(radial.Scale(t * sinA))
	sep := pf.Sub(sph.Anchor)
	switch {
	case v > k.tol:
		dir, ok := sep.Normalize()
		if !ok {
			sink.unsure = true
			return
		}
		pg := sph.Anchor.Add(dir.Scale(sph.Radius))
		admit := clearance.AdmitState(cone.AdmitPoint(pf, k.tol), sph.AdmitPoint(pg, k.tol))
		sink.candidate(k, admit, v, v, true, pf, pg)
	case v < -k.tol:
		if clearance.ClrBoxDist(f.Box, g.Box) > k.tol {
			return
		}
		sink.unsure = true
	default:
		if clearance.AdmitState(cone.AdmitPoint(pf, k.tol), sph.AdmitPoint(sph.Anchor.Add(sep), k.tol)) == -1 {
			return
		}
		sink.unsure = true
	}
}
