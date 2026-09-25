package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/r3"
)

// This file is the pair kernel of docs/clearance-design.md §1/§2/§5/§6/§7,
// and, since PR increment 2, docs/surface-design.md §9.3's sheet-against-solid
// decision procedure (sheetSolidPair, below). One non-mutating pass proves one
// of four outcomes: pairDisjoint, pairTouching, pairOverlapping, or
// pairUndecided. Admitted non-coplanar transversal crossings and strict
// containment produce pairOverlapping; unsupported contact stays
// pairUndecided. When asked, the kernel measures the gap as a proven interval
// [lo, hi]. Exactness belongs to the interval, not the winning candidate:
// Exact requires a closed-form winner and every bracketed rival proven to sit
// at or above it. Cone-involved pairs may read a coarse enclosure interval
// rather than a fabricated verdict.

// pairVerdict is the kernel's partition answer for one pair.
type pairVerdict int

const (
	// pairUndecided: the kernel can prove neither shared interior nor
	// disjoint interiors. It joins neither report list and reads Suspect.
	pairUndecided pairVerdict = iota
	// pairDisjoint: boundary clearance proven positive and nesting excluded.
	pairDisjoint
	// pairTouching: every zero-distance contact certified (the coplanar
	// plane-pair certificate) — the gap is a measured Exact zero (§6).
	pairTouching
	// pairOverlapping: shared interior is proven by nesting or another
	// admitted certificate. A row still requires a complete bounded overlap
	// volume from containment or the read-only boolean evaluator.
	pairOverlapping
)

// pairResult is one pair's kernel answer: the verdict, the proven gap
// interval (when disjoint or touching), and the pair-diameter reading the
// §7 gate needs.
type pairResult struct {
	verdict pairVerdict
	lo, hi  float64
	exact   bool
	diam    float64
	// contained is non-nil only when strict positive boundary clearance, one
	// witness per shell of this body proven inside the other, and one witness
	// per shell of the other — voids included — proven outside this one prove
	// this whole body lies in the other.
	contained *Body
}

// pairKernel is one pair's working state.
type pairKernel struct {
	a, b  *bodyGeom
	scale float64
	tol   float64
	slack float64
	ctx   context.Context //nolint:containedctx // pairKernel is per-call state and never outlives clearancePair.
	err   error
	// clearanceRefused records a finite-input arithmetic range failure. The
	// cell walk may continue finding overlap, but it cannot certify a gap.
	clearanceRefused bool
}

// bodyGeomCache reuses completed carrier models within one Verify call.
// A failed or canceled build is never cached.
type bodyGeomCache struct {
	entries map[*Body]bodyGeomCacheEntry
}

type bodyGeomCacheEntry struct {
	geom *bodyGeom
	ok   bool
}

func (cache *bodyGeomCache) get(budget *workBudget, body *Body) (*bodyGeom, bool, error) {
	if cache == nil {
		return newBodyGeomBudget(budget, body)
	}
	if err := budget.err(); err != nil {
		return nil, false, err
	}
	if entry, found := cache.entries[body]; found {
		return entry.geom, entry.ok, nil
	}
	geom, ok, err := newBodyGeomBudget(budget, body)
	if err != nil {
		return nil, false, err
	}
	if cache.entries == nil {
		cache.entries = make(map[*Body]bodyGeomCacheEntry)
	}
	cache.entries[body] = bodyGeomCacheEntry{geom: geom, ok: ok}
	return geom, ok, nil
}

// clearanceDeltaWiden widens a held-candidate interval [lo, hi] by the two
// bodies' bodyGeom.delta (payload-verification-design.md §2.3's formula,
// applied here to the analytic arms rather than the not-yet-landed faceted
// one): distance between closed sets is 1-Lipschitz in each operand, so the
// TRUE interval lies in [max(0, down(lo−widen)), up(hi+widen)]. exact
// collapses to false whenever either delta is nonzero — a widened interval
// can never carry an Exact label (docs/api-design.md §5.3: Exact is a claim
// the number IS the truth). A zero widen (both deltas zero, the common
// feature-built, unplaced, axis-aligned case) returns its inputs unchanged,
// so it costs no extra rounding there.
func clearanceDeltaWiden(lo, hi float64, exact bool, deltaA, deltaB float64) (float64, float64, bool) {
	widen := absSumUpper(deltaA, deltaB)
	if widen == 0 {
		return lo, hi, exact
	}
	lo = math.Max(0, downRound(lo-widen))
	hi = absSumUpper(hi, widen)
	return lo, hi, false
}

// clearancePair runs the kernel over one pair of proven solids.
// nestingExcluded is true when box separation has already excluded nesting
// (a box-proven pair needs the kernel only for its gap — §7).
//
// Check order is load-bearing: the coplanar Plane×Plane contact certificate
// runs first and short-circuits every later check when it certifies a touch,
// because a separating plane already certifies the whole contact set — but
// ONLY when both bodies' bodyGeom.delta are exactly zero, since the
// certificate is an exact material-side claim (payload-verification §7.2's
// rule for the faceted case, applied here to every analytic arm) that a
// carrier built from rounded coordinates cannot honestly make. Past that, an
// admitted transversal crossing (sink.overlap) is read before sink.unsure, so
// a pair that is both overlapping AND touches an unsupported contact
// elsewhere still reports the overlap it can prove. The held-candidate
// interval is then widened ONCE by the two bodies' deltas (clearanceDeltaWiden,
// above) before the proven lower bound lo must clear the tolerance floor
// k.tol — widening first, so the §2 nesting cast never runs on a lo the
// widening would have brought back down to the floor.
func clearancePair(ctx context.Context, a, b *Body, nestingExcluded bool) (pairResult, error) {
	return clearancePairCached(ctx, a, b, nestingExcluded, nil)
}

func clearancePairCached(ctx context.Context, a, b *Body, nestingExcluded bool, cache *bodyGeomCache) (pairResult, error) {
	if err := ctx.Err(); err != nil {
		return pairResult{}, err
	}
	if result, ok := clearanceAxisBoxes(a, b); ok {
		if err := ctx.Err(); err != nil {
			return pairResult{}, err
		}
		return result, nil
	}
	budget := newWorkBudget(ctx)
	ga, oka, err := cache.get(budget, a)
	if err != nil {
		return pairResult{}, err
	}
	gb, okb, err := cache.get(budget, b)
	if err != nil {
		return pairResult{}, err
	}
	if !oka || !okb {
		return pairResult{}, nil
	}
	scale := 1.0
	for _, bb := range [][2]r3.Vec{{a.bounds.Min, a.bounds.Max}, {b.bounds.Min, b.bounds.Max}} {
		for _, p := range bb {
			for _, c := range []float64{p.X, p.Y, p.Z} {
				if v := math.Abs(c); v > scale {
					scale = v
				}
			}
		}
	}
	k := &pairKernel{a: ga, b: gb, scale: scale, tol: 1e-9 * scale, slack: 1e-9 * scale, ctx: ctx}
	diam, err := k.pairDiameter()
	if err != nil {
		return pairResult{}, err
	}

	// The coplanar Plane × Plane contact certificate (§6): coplanar caps
	// with strictly opposing outward normals and a positive-area trim
	// overlap, with each body's material wholly on its own side of the
	// shared plane — the separating plane clears the interiors globally and
	// certifies the whole contact set, so the gap is a measured Exact zero.
	// It runs only when both bodies' bodyGeom.delta are exactly zero: it is
	// an exact material-side claim, and a nonzero delta means at least one
	// carrier plane may itself be displaced from the boundary it is meant to
	// certify (clearancePair's own doc comment).
	certified := false
	if ga.delta == 0 && gb.delta == 0 {
		certified, err = k.coplanarContactCertified(ctx)
		if err != nil {
			return pairResult{}, err
		}
	}
	if certified {
		return pairResult{verdict: pairTouching, exact: true, diam: diam}, nil
	}

	sink, err := k.enumerate()
	if err != nil {
		return pairResult{}, err
	}
	if sink.overlap {
		return pairResult{verdict: pairOverlapping, diam: diam}, nil
	}
	if sink.unsure {
		return pairResult{diam: diam}, nil
	}
	hi := math.Inf(1)
	for _, c := range sink.contribs {
		if c.hi < hi {
			hi = c.hi
		}
	}
	if math.IsInf(hi, 1) {
		return pairResult{diam: diam}, nil
	}
	lo := hi
	exact := false
	for _, c := range sink.contribs {
		if c.lo >= hi {
			continue // §5 pruning: a bound at or above the best upper bound cannot hold the minimum
		}
		if c.lo < lo {
			lo = c.lo
		}
	}
	for _, c := range sink.contribs {
		if c.exact && c.lo == hi && c.hi == hi {
			exact = true
		}
	}
	exact = exact && lo == hi
	lo, hi, exact = clearanceDeltaWiden(lo, hi, exact, ga.delta, gb.delta)
	if lo <= k.tol {
		return pairResult{diam: diam}, nil
	}
	if !nestingExcluded {
		verdict, contained, err := k.nestingRelation()
		if err != nil {
			return pairResult{}, err
		}
		if verdict != pairDisjoint {
			return pairResult{verdict: verdict, diam: diam, contained: contained}, nil
		}
	}
	return pairResult{verdict: pairDisjoint, lo: lo, hi: hi, exact: exact, diam: diam}, nil
}

// coplanarContactCertified scans the plane-face pairs for the §6 coplanar
// certificate.
func (k *pairKernel) coplanarContactCertified(ctx context.Context) (bool, error) {
	if err := ctx.Err(); err != nil {
		return false, err
	}
	budget := newWorkBudget(ctx)
	for _, fa := range k.a.faces {
		if err := budget.step(); err != nil {
			return false, err
		}
		if fa.kind != ckPlane {
			continue
		}
		for _, fb := range k.b.faces {
			if err := budget.step(); err != nil {
				return false, err
			}
			if fb.kind != ckPlane {
				continue
			}
			// The certificate is EXACT and reject-only: a zero row may
			// only come from a proven contact, so any nonzero plane
			// separation, tilt, or side penetration leaves the pair
			// undecided — a tolerance here would bless a real sub-tol
			// overlap as a Sound Exact-zero clearance.
			if fa.n.Dot(fb.n) != -1 {
				continue // not exactly opposing
			}
			if fb.o.Sub(fa.o).Dot(fa.n) != 0 {
				continue // not exactly coplanar
			}
			rel, _, err := k.coplanarRelation(budget, fa, fb)
			if err != nil {
				return false, err
			}
			if rel != 1 {
				continue // no proven positive-area overlap
			}
			c := fa.planeOffset()
			aLo, aHi, okA := payloadExtent(ctx, k.a.body, fa.n)
			bLo, bHi, okB := payloadExtent(ctx, k.b.body, fa.n)
			if !okA || !okB {
				continue
			}
			_ = aLo
			_ = bHi
			if aHi <= c && bLo >= c {
				return true, nil
			}
			if bHi <= c && aLo >= c {
				return true, nil
			}
		}
	}
	return false, ctx.Err()
}

// payloadExtent is the body's exact extent interval along a direction, read
// off its own payload.
func payloadExtent(ctx context.Context, b *Body, g r3.Vec) (float64, float64, bool) {
	switch pl := b.payload.(type) {
	case prismPayload:
		lo, hi, err := pl.extentAlongContext(ctx, g)
		if err != nil {
			return 0, 0, false
		}
		return lo, hi, true
	case revolvePayload:
		lo, hi, err := pl.extentAlongContext(ctx, g)
		if err != nil {
			return 0, 0, false
		}
		return lo, hi, true
	default:
		return 0, 0, false
	}
}

// nestingRelation classifies every deterministic shell witness of each shipped
// single-lump analytic body in both directions. Positive boundary clearance
// makes membership constant on each connected component of one body minus the
// other body's shells — never across the lump as a whole, since the other
// body's shells can cut through it. So containment needs both directions: the
// inner body's every witness inside the outer, AND the outer body's every
// witness — void shells included — outside the inner. Only then does the outer
// body's boundary miss the inner body entirely, leaving the inner body in one
// component its witnesses speak for. Multi-lump/faceted bodies never reach this
// method; they use read-only intersection instead.
func (k *pairKernel) nestingRelation() (pairVerdict, *Body, error) {
	type membership struct {
		inside, outside, unknown int
	}
	budget := newWorkBudget(k.ctx)
	classify := func(inner, outer *bodyGeom) (membership, error) {
		var got membership
		for _, w := range inner.shellWit {
			if err := budget.step(); err != nil {
				return membership{}, err
			}
			inside, ok, err := outer.pointInBody(k.ctx, w, k.tol)
			if err != nil {
				return membership{}, err
			}
			switch {
			case !ok:
				got.unknown++
			case inside:
				got.inside++
			default:
				got.outside++
			}
		}
		return got, nil
	}
	ab, err := classify(k.a, k.b)
	if err != nil {
		return pairUndecided, nil, err
	}
	ba, err := classify(k.b, k.a)
	if err != nil {
		return pairUndecided, nil, err
	}
	contains := func(inner, outer membership, innerWit, outerWit []r3.Vec) bool {
		return len(innerWit) > 0 && len(outerWit) > 0 &&
			inner.inside == len(innerWit) && outer.outside == len(outerWit)
	}
	if contains(ab, ba, k.a.shellWit, k.b.shellWit) {
		return pairOverlapping, k.a.body, nil
	}
	if contains(ba, ab, k.b.shellWit, k.a.shellWit) {
		return pairOverlapping, k.b.body, nil
	}
	if ab.inside > 0 || ba.inside > 0 {
		return pairOverlapping, nil, nil
	}
	if ab.unknown > 0 || ba.unknown > 0 {
		return pairUndecided, nil, nil
	}
	return pairDisjoint, nil, nil
}

// pairDiameter reads the pair's diameter D — the greatest distance between
// two points drawn from either body — from exact vertex positions and
// per-face analytic support points (§7). The reading may understate the true
// diameter, which only lowers the noise floor: the safe direction.
func (k *pairKernel) pairDiameter() (float64, error) {
	pts := append(append([]r3.Vec{}, k.a.supports...), k.b.supports...)
	best := 0.0
	budget := newWorkBudget(k.ctx)
	for i := range pts {
		for j := i + 1; j < len(pts); j++ {
			if err := budget.step(); err != nil {
				return 0, err
			}
			if d := pts[i].Sub(pts[j]).Len(); d > best {
				best = d
			}
		}
	}
	return best, k.ctx.Err()
}

// sheetSolidVerdict is docs/surface-design.md §9.3's pair decision procedure's
// answer for a pair holding exactly one sheet operand against a proven solid.
// A sheet encloses no region, so §1's four interior relations do not apply to
// this pair shape at all; these four answers replace them.
type sheetSolidVerdict int

const (
	// sheetSolidUndecided — a body model was missing, the candidate
	// enumeration could not decide the boundary distance, or the witness
	// cast failed. Neither Interfering, Sound, nor a Clearance row follows.
	sheetSolidUndecided sheetSolidVerdict = iota
	// sheetSolidCrossing — an admitted transversal crossing between a sheet
	// face and a solid face proves the sheet crosses the solid's boundary.
	sheetSolidCrossing
	// sheetSolidContained — the boundary distance is proven positive, and the
	// witness vertex is proven inside the solid.
	sheetSolidContained
	// sheetSolidOutside — the same proof, with the witness proven outside.
	sheetSolidOutside
)

// sheetSolidResult is one sheet-against-solid pair's decision. lo, hi, exact
// and diam are populated only for sheetSolidContained and sheetSolidOutside,
// and they carry the SAME boundary-distance interval regardless of which side
// the witness landed on (docs/api-design.md §6.2): the interval is proven
// before the witness cast ever runs, so containment and separation share one
// Clearance row that states the distance to the solid's boundary without
// asserting which side the sheet is on.
type sheetSolidResult struct {
	verdict sheetSolidVerdict
	lo, hi  float64
	exact   bool
	diam    float64
}

// sheetSolidPair decides docs/surface-design.md §9.3's pair procedure for a
// sheet operand against a proven solid: run when their bounds-inflated boxes
// meet, and also when they are separated but a gap was requested — the same
// two occasions clearancePair itself runs for a solid pair. boxDisjoint is
// true in that second case: the caller already proved the boxes separated,
// so the answer here can only be sheetSolidOutside or sheetSolidUndecided,
// never crossing or contained, and the witness cast below is skipped as
// redundant, exactly as clearancePair's own nestingExcluded parameter skips
// its two-directional cast for the same reason. It reuses clearancePair's own
// carrier model and candidate enumeration (§3) unchanged, with two departures
// a sheet's missing material forces:
//
//   - the §6 coplanar contact certificate is skipped outright. That
//     certificate proves each body's material lies wholly on its own side of
//     a shared plane, which is a claim about material a sheet does not
//     have — it could never fire honestly here, so it is dropped rather than
//     run for nothing.
//   - a proven positive lower bound on the boundary distance is settled by
//     ONE deterministic witness cast, rather than the two-directional nesting
//     relation clearancePair runs for a solid pair — skipped outright when
//     boxDisjoint already answers it.
//
// WHY ONE WITNESS DECIDES THE WHOLE SHEET: once the boundary distance is
// proven positive, the sheet's boundary misses the solid's boundary entirely.
// The sheet is one connected shell — asserted below, not merely assumed, from
// docs/surface-design.md §2.2's rule that a sheet lump holds exactly one
// shell, together with newBodyGeomBudget's existing single-lump gate — so it
// lies wholly within one connected component of space minus the solid's
// boundary, and one point's membership answers for the whole shell. The
// two-directional cast a solid pair needs exists because an INNER body's
// several shells can be cut apart by the OUTER body's void shells, so the
// outer's own witnesses must be checked too, in the other direction; a sheet
// with a single shell has no void shells of its own for anything to cut
// apart, so the reverse cast is not merely skipped as an optimization — it
// has nothing left to prove. Box separation proves the same conclusion an
// easier way: a box that does not even meet the solid's own box cannot admit
// a crossing or a containment either.
func sheetSolidPair(ctx context.Context, sheet, solid *Body, boxDisjoint bool, cache *bodyGeomCache) (sheetSolidResult, error) {
	if err := ctx.Err(); err != nil {
		return sheetSolidResult{}, err
	}
	budget := newWorkBudget(ctx)
	gs, oks, err := cache.get(budget, sheet)
	if err != nil {
		return sheetSolidResult{}, err
	}
	gb, okb, err := cache.get(budget, solid)
	if err != nil {
		return sheetSolidResult{}, err
	}
	if !oks || !okb {
		// Either operand's carrier model is missing — a revolve sheet is
		// refused a model outright, and a multi-lump or faceted operand
		// bypasses this analytic kernel entirely (docs/clearance-design.md
		// §2). The pair stays undecided rather than guessing a side.
		return sheetSolidResult{verdict: sheetSolidUndecided}, nil
	}
	scale := 1.0
	for _, bb := range [][2]r3.Vec{{sheet.bounds.Min, sheet.bounds.Max}, {solid.bounds.Min, solid.bounds.Max}} {
		for _, p := range bb {
			for _, c := range []float64{p.X, p.Y, p.Z} {
				if v := math.Abs(c); v > scale {
					scale = v
				}
			}
		}
	}
	k := &pairKernel{a: gs, b: gb, scale: scale, tol: 1e-9 * scale, slack: 1e-9 * scale, ctx: ctx}
	diam, err := k.pairDiameter()
	if err != nil {
		return sheetSolidResult{}, err
	}

	// No coplanarContactCertified call here — see the doc comment above.
	sink, err := k.enumerate()
	if err != nil {
		return sheetSolidResult{}, err
	}
	if sink.overlap {
		return sheetSolidResult{verdict: sheetSolidCrossing, diam: diam}, nil
	}
	if sink.unsure {
		return sheetSolidResult{verdict: sheetSolidUndecided, diam: diam}, nil
	}
	hi := math.Inf(1)
	for _, c := range sink.contribs {
		if c.hi < hi {
			hi = c.hi
		}
	}
	if math.IsInf(hi, 1) {
		return sheetSolidResult{verdict: sheetSolidUndecided, diam: diam}, nil
	}
	lo := hi
	exact := false
	for _, c := range sink.contribs {
		if c.lo >= hi {
			continue // §5 pruning: a bound at or above the best upper bound cannot hold the minimum
		}
		if c.lo < lo {
			lo = c.lo
		}
	}
	for _, c := range sink.contribs {
		if c.exact && c.lo == hi && c.hi == hi {
			exact = true
		}
	}
	exact = exact && lo == hi
	lo, hi, exact = clearanceDeltaWiden(lo, hi, exact, gs.delta, gb.delta)
	if lo <= k.tol {
		return sheetSolidResult{verdict: sheetSolidUndecided, diam: diam}, nil
	}

	if boxDisjoint {
		// Box separation already proves the sheet lies wholly outside the
		// solid (docs/surface-design.md §9.3): the witness cast below would
		// only confirm what a non-overlapping bounding box already decided,
		// the same shortcut clearancePair takes via nestingExcluded.
		return sheetSolidResult{verdict: sheetSolidOutside, lo: lo, hi: hi, exact: exact, diam: diam}, nil
	}

	// The one-shell premise the doc comment above states, asserted rather
	// than assumed: newBodyGeomBudget already refuses any body with more
	// than one lump, and a sheet lump holds exactly one shell
	// (docs/surface-design.md §2.2), so gs.shellWit carries exactly one
	// witness here. A future sheet construction that broke that premise
	// would land here undecided rather than pick a witness arbitrarily.
	if len(gs.shellWit) != 1 {
		return sheetSolidResult{verdict: sheetSolidUndecided, diam: diam}, nil
	}
	inside, ok, err := gb.pointInBody(ctx, gs.shellWit[0], k.tol)
	if err != nil {
		return sheetSolidResult{}, err
	}
	if !ok {
		return sheetSolidResult{verdict: sheetSolidUndecided, diam: diam}, nil
	}
	if inside {
		return sheetSolidResult{verdict: sheetSolidContained, lo: lo, hi: hi, exact: exact, diam: diam}, nil
	}
	return sheetSolidResult{verdict: sheetSolidOutside, lo: lo, hi: hi, exact: exact, diam: diam}, nil
}
