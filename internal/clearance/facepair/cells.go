// Package facepair evaluates the face-interior cells of the clearance kernel.
package facepair

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/clearance/spine"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Kernel holds the state shared by the cells of one face pair.
type Kernel struct {
	ctx        context.Context //nolint:containedctx // A face-pair kernel is per-call state.
	tol, slack float64
	err        error
	refused    bool
}

func New(ctx context.Context, tol, slack float64) *Kernel {
	return &Kernel{ctx: ctx, tol: tol, slack: slack}
}

func (k *Kernel) Err() error               { return k.err }
func (k *Kernel) Refused() bool            { return k.refused }
func (k *Kernel) oracle() clearance.Oracle { return clearance.Oracle{Tol: k.tol} }

func (k *Kernel) PlaneCrossesRevolved(f, g *clearance.CFace, sink *clearance.CellSink) {
	k.planeCrossesRevolved(f, g, sink)
}

func (k *Kernel) PlaneSphere(f, g *clearance.CFace, sink *clearance.CellSink) {
	k.planeSphere(f, g, sink)
}

// FaceCell dispatches one face pair through the §4 table.
func (k *Kernel) FaceCell(f, g *clearance.CFace, sink *clearance.CellSink) {
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
		sink.Coarse(f.Box, g.Box, f.Wit, g.Wit)
	case (f.Kind == clearance.CkTorus && f.Spindle) || (g.Kind == clearance.CkTorus && g.Spindle):
		// A Minor ≥ Major torus leaves the polynomial path (§4): the pair
		// takes the coarse enclosure — the face-box distance below, the
		// closest witness pair above.
		sink.Coarse(f.Box, g.Box, f.Wit, g.Wit)
	default:
		k.offsetPair(f, g, sink)
	}
}

// offsetPair is the spine-offset reduction of §4 for cylinder/sphere/torus
// pairs: enumerate the spine-pair criticals, emit every offset combination
// per critical, then decide whether a carrier crossing is excluded — by the
// strict exterior branch, by a certified containment (§4's d_sup list: an
// inner point spine, parallel cylinder axes, coaxial spines), by the windowed
// nested cell, or by face-box separation; otherwise the pair is undecided.
func (k *Kernel) offsetPair(f, g *clearance.CFace, sink *clearance.CellSink) {
	crits, ok := k.spineCriticals(f, g)
	if !ok {
		// A line pair the parallel oracle cannot decide has no critical, but
		// the windowed cell needs none. Its bound tightens the coarse
		// enclosure rather than replacing it: a window axially outside the
		// partner's trim admits no witness, and the coarse box distance can
		// prove more than the windowed one.
		if f.Kind == clearance.CkCylinder && g.Kind == clearance.CkCylinder {
			if lo, hi, ok := k.windowedNested(f, g); ok {
				sink.CoarseWith(lo, hi, f.Box, g.Box, f.Wit, g.Wit)
				return
			}
		}
		sink.Coarse(f.Box, g.Box, f.Wit, g.Wit)
		return
	}
	minLo := math.Inf(1)
	nested := false
	for _, c := range crits {
		minLo = math.Min(minLo, c.Lo)
		nested = k.emitOffsetCombos(sink, f, g, c) || nested
	}
	if nested {
		return // the windowed cell proved f's face strictly inside g's carrier
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
	sink.Unsure = true
}

func (k *Kernel) spineEngine() *spine.Engine {
	return &spine.Engine{Context: k.ctx, Tolerance: k.tol, Slack: k.slack}
}

func (k *Kernel) captureSpine(e *spine.Engine) {
	if e.Err != nil {
		k.err = e.Err
	}
	k.refused = k.refused || e.Refused
}

func (k *Kernel) spineCriticals(f, g *clearance.CFace) ([]clearance.SpineCrit, bool) {
	e := k.spineEngine()
	out, ok := e.SpineCriticals(f, g)
	k.captureSpine(e)
	return out, ok
}

// emitOffsetCombos emits the four offset combinations of one spine-pair
// critical: the joining line meets each offset surface twice, and every
// carrier-pair stationary point lies among the combinations, so admission
// (§3) decides each in isolation. It reports true when the windowed nested
// cell answered a coincident critical, which also excludes a crossing of the
// two faces.
func (k *Kernel) emitOffsetCombos(sink *clearance.CellSink, f, g *clearance.CFace, c clearance.SpineCrit) bool {
	sep := c.Fb.Sub(c.Fa)
	d := sep.Len()
	if d <= k.tol {
		// Coincident spine feet: the joining direction degenerates into a
		// whole ring. Only a PROVEN ring family carries it (concentric
		// shells, coaxial cylinders — the peg-in-hole reading). A
		// near-coincidence the oracle cannot prove — a pin a rounding error
		// off its bore's axis — is answered by the windowed nested cell when
		// one face's trimmed spine lies strictly inside the other's carrier,
		// and is otherwise a contact question this cell has no right to
		// answer.
		if k.oracle().RingFamily(f, g) == clearance.DegYes {
			k.emitRingCombos(sink, f, g, c)
			return false
		}
		if lo, hi, ok := k.windowedNested(f, g); ok {
			sink.Contribs = append(sink.Contribs, clearance.GapContrib{Lo: lo, Hi: hi})
			return true
		}
		sink.Unsure = true
		return false
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
					sink.Unsure = true
				}
				continue
			}
			lo, hi := math.Abs(rawLo), math.Abs(rawHi)
			if lo > hi {
				lo, hi = hi, lo
			}
			sink.Candidate(k.tol, admit, lo, hi, c.Exact, pf, pg)
		}
	}
	return false
}

// windowedNested is §4's windowed nested cell: a sphere or cylinder face whose
// trimmed spine lies strictly inside a sphere or cylinder carrier, bounded
// from the inner face's spine window alone, with no oracle answer. f and g
// are reordered so f has the smaller radius. d_sup is the greatest distance
// from f's spine window (its centre, or its axial window's two ends) to g's
// spine, plus the charge below; when r_g − r_f − d_sup clears the tolerance,
// every point of f's face lies strictly inside g's carrier, so the two faces
// cannot meet and every point pair is at least that far apart. It returns
// that lower bound and the nearest admitted witness pair's distance as the
// upper one, +Inf when no witness is admitted, and ok false when the radii,
// the spines or the margin do not admit the certificate.
//
// The charge is AnalyticRoundBound over an envelope of every coordinate and
// radius the cell reads: each distance and each witness takes fewer than 128
// additions, multiplications, divisions and correctly rounded square roots
// at that magnitude. A witness is never Exact: its feet are float points a
// few roundings off the carriers, and a window end is read 2·tol inside the
// window so the trim admits it with the kernel's own margin.
func (k *Kernel) windowedNested(f, g *clearance.CFace) (float64, float64, bool) {
	if clearance.SpineOffset(g) < clearance.SpineOffset(f) {
		f, g = g, f
	}
	if clearance.SpineOf(f) > 1 || clearance.SpineOf(g) > 1 {
		return 0, 0, false // a circle spine has no endpoint maximum
	}
	rf, rg := clearance.SpineOffset(f), clearance.SpineOffset(g)
	window := []float64{0}
	reach := 0.0
	if f.Kind == clearance.CkCylinder {
		window = []float64{f.ZWin.Lo, f.ZWin.Hi}
		reach = math.Max(math.Abs(f.ZWin.Lo), math.Abs(f.ZWin.Hi))
	}
	charge := proofbound.AnalyticRoundBound(proofbound.AbsSumUpper(
		proofbound.VecMaxAbs(f.Anchor), proofbound.VecMaxAbs(g.Anchor), reach, rf, rg))
	dSup := 0.0
	for _, z := range window {
		dSup = math.Max(dSup, clearance.PointSpineDist(f.Anchor.Add(f.Axis.Scale(z)), g))
	}
	dSup = proofbound.AbsSumUpper(dSup, charge)
	lo := freeform.DownRound(freeform.DownRound(rg-rf) - dSup)
	if proofbound.IsNonFinite(lo) || !(lo > k.tol) {
		return 0, 0, false
	}

	axis := f.Axis
	if g.Kind == clearance.CkCylinder {
		axis = g.Axis
	}
	u := clearance.PerpTo(axis)
	v := axis.Cross(u)
	dirs := make([]r3.Vec, 0, 24)
	for _, th := range clearance.RingAngles(f, g, u, v) {
		dirs = append(dirs, u.Scale(math.Cos(th)).Add(v.Scale(math.Sin(th))))
	}
	stations := []float64{0}
	if f.Kind == clearance.CkCylinder {
		stations = []float64{f.ZWin.Lo + 2*k.tol, (f.ZWin.Lo + f.ZWin.Hi) / 2, f.ZWin.Hi - 2*k.tol}
	} else {
		dirs = append(dirs, axis, axis.Scale(-1))
	}
	best := math.Inf(1)
	for _, z := range stations {
		a := f.Anchor.Add(f.Axis.Scale(z))
		for _, dir := range dirs {
			if f.Kind == clearance.CkCylinder {
				dir = dir.Sub(f.Axis.Scale(dir.Dot(f.Axis)))
			}
			dir, ok := dir.Normalize()
			if !ok {
				continue
			}
			p := a.Add(dir.Scale(rf))
			c := g.Anchor
			if g.Kind == clearance.CkCylinder {
				c = clearance.LinePoint(g.Anchor, g.Axis, p)
			}
			out, ok := p.Sub(c).Normalize()
			if !ok {
				continue
			}
			q := c.Add(out.Scale(rg))
			if clearance.AdmitState(f.AdmitPoint(p, k.tol), g.AdmitPoint(q, k.tol)) == 1 {
				best = math.Min(best, p.Sub(q).Len())
			}
		}
	}
	if math.IsInf(best, 1) {
		// A sample proves a witness present, never absent: the lower bound
		// stands alone and the pair reads undecided for want of an upper one.
		return lo, best, true
	}
	return lo, proofbound.AbsSumUpper(best, charge), true
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
func (k *Kernel) emitRingCombos(sink *clearance.CellSink, f, g *clearance.CFace, c clearance.SpineCrit) {
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
			sink.Candidate(k.tol, a, math.Abs(rf-rg), math.Abs(rf-rg), c.Exact, pf, near)
		}
		if a := clearance.AdmitState(f.AdmitPoint(pf, margin), g.AdmitPoint(far, margin)); a != -1 {
			sink.Candidate(k.tol, a, rf+rg, rf+rg, c.Exact, pf, far)
		}
	}
	if !nearAdmitted {
		// The near pairing is the small one: a far pairing can never hold the
		// minimum the near pairing does not. Unproven absence keeps its bound.
		sink.LoOnly(math.Abs(rf - rg))
	}
}

// planePlane is the plane-row cell for two planes: a coplanar pair reaching
// here is uncertified (the coplanar contact certificate runs at the
// pair level first); a parallel distinct pair carries its plateau exactly
// where the trims overlap in projection (§3's projection rule); crossing
// carriers are excluded through the trims or read as the §6/§7-routed
// contact, never quietly as a boundary answer.
func (k *Kernel) planePlane(f, g *clearance.CFace, sink *clearance.CellSink) {
	// The plateau exists only for EXACT parallelism: a tolerance here
	// blesses a tilted pair's mid-face height as an Exact minimum the true
	// gap undercuts (or exceeds). A tilt too small for the crossing-line
	// path to certify falls to unsure there — never a wrong Exact.
	if f.N.Cross(g.N).Len() == 0 {
		h := g.O.Sub(f.O).Dot(f.N)
		rel, wit, err := clearance.CoplanarRelation(proofbound.NewWorkBudget(k.ctx), f, g)
		if err != nil {
			k.err = err
			return
		}
		if math.Abs(h) <= k.tol {
			if rel != -1 {
				sink.Unsure = true
			}
			return
		}
		switch rel {
		case 1:
			pa := f.O.Add(f.U.Scale(wit[0])).Add(f.V.Scale(wit[1]))
			pb := pa.Add(f.N.Scale(h))
			sink.Candidate(k.tol, 1, math.Abs(h), math.Abs(h), true, pa, pb)
		case 0:
			sink.LoOnly(math.Abs(h))
		}
		return
	}
	if clearance.ClrBoxDist(f.Box, g.Box) > k.tol {
		return
	}
	// The intersection line, clipped by both trims on a shared parameter.
	dir, ok := f.N.Cross(g.N).Normalize()
	if !ok {
		sink.Unsure = true
		return
	}
	p0, ok := clearance.PlanesIntersect(f, g)
	if !ok {
		sink.Unsure = true
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
		sink.Crossing(clearance.IntervalsMeet(ivF, ivG, k.tol))
		return
	}
	supF := f.Region.LineIntervalsSuperset(fx, fy, dir.Dot(f.U), dir.Dot(f.V))
	supG := g.Region.LineIntervalsSuperset(gx, gy, dir.Dot(g.U), dir.Dot(g.V))
	if clearance.IntervalsMeet(supF, supG, k.tol) != -1 {
		sink.Unsure = true
	}
}

// planeCylinder is the plane-row cell for a cylinder: an axis-parallel pair
// carries the constant ruling plateau; a crossing carrier is excluded
// through the trims (the two crossing rulings when the axis parallels the
// plane, the exact axial crossing range otherwise) or routed to §6/§7.
func (k *Kernel) planeCylinder(f, g *clearance.CFace, sink *clearance.CellSink) {
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
				sink.Overlap = true
				return
			}
			ambiguous = ambiguous || rel == 0
		}
		if ambiguous {
			sink.Unsure = true
		}
	default:
		// Tangency at distance zero: not certified here (§6); excluded
		// through the trims or undecided.
		if k.rulingRelation(f, g, toward) != -1 {
			sink.Unsure = true
		}
	}
}

// rulingCandidate emits the plateau candidate carried by one cylinder
// ruling at radial direction `radial` and plane distance v.
func (k *Kernel) rulingCandidate(f, g *clearance.CFace, radial r3.Vec, v float64, sink *clearance.CellSink) {
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
	sink.Candidate(k.tol, admit, v, v, true, pa, pb)
}

// rulingRelation classifies one carrier-crossing ruling through both trims.
func (k *Kernel) rulingRelation(f, g *clearance.CFace, radial r3.Vec) int {
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
func (k *Kernel) planeCrossesRevolved(f, g *clearance.CFace, sink *clearance.CellSink) {
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
	mn, mx := clearance.TrigRange(f.N.Dot(g.RefU)*g.Radius, f.N.Dot(g.RefV)*g.Radius, lo, hi)
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
		sink.Crossing(rel)
		return
	}
	sink.Unsure = true
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
func (k *Kernel) planeCone(f, g *clearance.CFace, sink *clearance.CellSink) {
	s := f.N.Dot(g.Axis)
	apexH := g.Anchor.Sub(f.O).Dot(f.N)
	nu, nv := f.N.Dot(g.RefU), f.N.Dot(g.RefV)

	// Exact range of n·x − c over the trimmed face.
	lo, hi := 0.0, 2*math.Pi
	if !g.Sweep.Full {
		lo, hi = g.Sweep.Lo, g.Sweep.Hi
	}
	mn, mx := clearance.TrigRange(nu, nv, lo, hi)
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
	sink.Unsure = true
}

// planeSphere is the plane-row sphere cell: the center's plane distance
// minus the radius, tangency routed to §6, a crossing excluded through the
// trims (the exact crossing circle) or undecided.
func (k *Kernel) planeSphere(f, g *clearance.CFace, sink *clearance.CellSink) {
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
			sink.Candidate(k.tol, admit, d-dirSign*g.Radius, d-dirSign*g.Radius, true, foot, pb)
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
		sink.Crossing(rel)
	default:
		pb := g.Anchor.Sub(f.N.Scale(side * g.Radius))
		foot := pb.Sub(f.N.Scale(f.N.Dot(pb.Sub(f.O))))
		x, y := f.PlaneCoords(foot)
		if clearance.AdmitState(f.Region.Classify(x, y, k.tol), g.AdmitPoint(pb, k.tol)) == -1 {
			return
		}
		sink.Unsure = true
	}
}

// planeTorus is the plane-row torus cell: the spine circle's plane-distance
// amplitude extreme minus the minor radius — valid for the spindle branch
// too, since the extreme's meridian direction always lies in the extreme
// spine point's own meridian plane.
func (k *Kernel) planeTorus(f, g *clearance.CFace, sink *clearance.CellSink) {
	base := g.Anchor.Sub(f.O).Dot(f.N)
	au := g.Major * f.N.Dot(g.RefU)
	av := g.Major * f.N.Dot(g.RefV)
	lo, hi := 0.0, 2*math.Pi
	if !g.Sweep.Full {
		lo, hi = g.Sweep.Lo, g.Sweep.Hi
	}
	mn, mx := clearance.TrigRange(au, av, lo, hi)
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
			sink.LoOnly(math.Min(math.Abs(hLo), math.Abs(hHi)) - g.Radius)
			return
		}
		sp := g.Anchor.Add(g.RefU.Scale(g.Major * math.Cos(star))).Add(g.RefV.Scale(g.Major * math.Sin(star)))
		pb := sp.Sub(f.N.Scale(side * g.Radius))
		foot := pb.Sub(f.N.Scale(f.N.Dot(pb.Sub(f.O))))
		x, y := f.PlaneCoords(foot)
		v := math.Min(math.Abs(hLo), math.Abs(hHi)) - g.Radius
		admit := clearance.AdmitState(f.Region.Classify(x, y, k.tol), g.AdmitPoint(pb, k.tol))
		sink.Candidate(k.tol, admit, v, v, true, foot, pb)
		return
	}
	if clearance.ClrBoxDist(f.Box, g.Box) > k.tol {
		return
	}
	if hLo > g.Radius-k.tol || hHi < -(g.Radius-k.tol) {
		// Tangency zone: not certified here.
		sink.Unsure = true
		return
	}
	sink.Unsure = true
}

// coneSphere is the closed-form meridian point/cone cell of §4's sphere
// column: the sphere center against the cone's generating ray in its own
// meridian half-plane, offset by the sphere radius on the strict branches.
func (k *Kernel) coneSphere(f, g *clearance.CFace, sink *clearance.CellSink) {
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
		sink.LoOnly(math.Abs(rho*cosA-z*sinA) - sph.Radius)
		return
	}
	t := z*cosA + rho*sinA // slant projection onto the ruling
	if t <= k.tol {
		// The nearest carrier point is the apex — a singular point owned by
		// the synthesized vertex tier; the interior cell has no critical.
		if clearance.ClrBoxDist(f.Box, g.Box) <= k.tol && z*z+rho*rho <= (sph.Radius+k.tol)*(sph.Radius+k.tol) {
			sink.Unsure = true
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
			sink.Unsure = true
			return
		}
		pg := sph.Anchor.Add(dir.Scale(sph.Radius))
		admit := clearance.AdmitState(cone.AdmitPoint(pf, k.tol), sph.AdmitPoint(pg, k.tol))
		sink.Candidate(k.tol, admit, v, v, true, pf, pg)
	case v < -k.tol:
		if clearance.ClrBoxDist(f.Box, g.Box) > k.tol {
			return
		}
		sink.Unsure = true
	default:
		if clearance.AdmitState(cone.AdmitPoint(pf, k.tol), sph.AdmitPoint(sph.Anchor.Add(sep), k.tol)) == -1 {
			return
		}
		sink.Unsure = true
	}
}
