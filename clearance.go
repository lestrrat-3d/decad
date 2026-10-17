package decad

import (
	"context"
	"math"
	"sync"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/reportvocab"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// This file is the pair kernel of docs/clearance-design.md §1/§2/§5/§6/§7,
// and, since PR increment 2, docs/surface-design.md §9.3's sheet-against-solid
// decision procedure (sheetSolidPair, below). One non-mutating pass proves one
// of four outcomes: reportvocab.PairDisjoint, reportvocab.PairTouching, reportvocab.PairOverlapping, or
// reportvocab.PairUndecided. Admitted non-coplanar transversal crossings and strict
// containment produce reportvocab.PairOverlapping; unsupported contact stays
// reportvocab.PairUndecided. When asked, the kernel measures the gap as a proven interval
// [lo, hi]. Exactness belongs to the interval, not the winning candidate:
// Exact requires a closed-form winner and every bracketed rival proven to sit
// at or above it. Cone-involved pairs may read a coarse enclosure interval
// rather than a fabricated verdict.

// pairResult is one pair's kernel answer: the verdict, the proven gap
// interval (when disjoint or touching), and the pair-diameter reading the
// §7 gate needs.
type pairResult struct {
	verdict reportvocab.PairVerdict
	lo, hi  float64
	exact   bool
	diam    float64
	// contained is non-nil only when strict positive boundary clearance, one
	// witness per shell of this body proven inside the other, and one witness
	// per shell of the other — voids included — proven outside this one prove
	// this whole body lies in the other.
	contained *Body
	// ruling is non-nil only on a reportvocab.PairTouching verdict proved by a §6
	// ruling certificate; it keeps the certified faces and feet.
	ruling *clearance.RulingContact
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
	// cache is the call's bodyGeomCache, which pairDiameter reads each
	// body's gate points through. It is nil when the pair has no call.
	cache *bodyGeomCache
}

// bodyGeomCache reuses completed carrier models within one Verify call.
// A failed or canceled build is never cached. It is safe for concurrent use:
// Verify proves several pairs at once. A body's model is built outside the
// lock, so two pairs asking for the same body at once may both build it; the
// build is deterministic, and the first stored model is the one every later
// call reads.
type bodyGeomCache struct {
	mu      sync.Mutex
	entries map[*Body]bodyGeomCacheEntry
	// stations keeps the section stations of the profiles this call reads
	// (diameter.StationMemo). It lives and dies with the cache.
	stations diameter.StationMemo
}

type bodyGeomCacheEntry struct {
	geom *bodyGeom
	ok   bool
	// gate is the body's gate points once read (gatePoints). It sits in the
	// entry so evicting the body's entry drops it with the carrier model.
	gate *gateReading
}

// gateReading is one body's bodyGatePoints answer.
type gateReading struct {
	points gatePoints
	ok     bool
}

// gatePoints is bodyGatePoints read once per body per call: a body's
// gate points depend on its payload alone, which a call never changes.
// A reading is stored only in the body's existing entry, so it lives
// and is evicted with the body's carrier model; a body with no entry
// is read without storing. The budget's context is checked before a stored
// reading is returned and, under the lock, before a reading is stored or a
// concurrent winner returned, so a cancelled call neither stores nor returns a
// reading. The
// returned points are shared between hits and must not be modified.
func (cache *bodyGeomCache) gatePoints(ctx context.Context, budget *proofbound.WorkBudget,
	body *Body) (gatePoints, bool, error) {
	if cache == nil {
		return bodyGatePoints(ctx, budget, body, nil)
	}
	cache.mu.Lock()
	entry, found := cache.entries[body]
	cache.mu.Unlock()
	if found && entry.gate != nil {
		if err := budget.Err(); err != nil {
			return gatePoints{}, false, err
		}
		return entry.gate.points, entry.gate.ok, nil
	}
	points, ok, err := bodyGatePoints(ctx, budget, body, &cache.stations)
	if err != nil {
		return gatePoints{}, false, err
	}
	cache.mu.Lock()
	defer cache.mu.Unlock()
	// A reading is stored and returned only while the budget is live: a
	// short read polls the context too rarely to have noticed a cancel.
	if err := budget.Err(); err != nil {
		return gatePoints{}, false, err
	}
	entry, found = cache.entries[body]
	if !found {
		return points, ok, nil
	}
	if entry.gate != nil {
		return entry.gate.points, entry.gate.ok, nil
	}
	entry.gate = &gateReading{points: points, ok: ok}
	cache.entries[body] = entry
	return points, ok, nil
}

func (cache *bodyGeomCache) get(budget *proofbound.WorkBudget, body *Body) (*bodyGeom, bool, error) {
	if cache == nil {
		return newBodyGeomBudget(budget, body)
	}
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	cache.mu.Lock()
	entry, found := cache.entries[body]
	cache.mu.Unlock()
	if found {
		return entry.geom, entry.ok, nil
	}
	geom, ok, err := newBodyGeomBudget(budget, body)
	if err != nil {
		return nil, false, err
	}
	cache.mu.Lock()
	defer cache.mu.Unlock()
	if entry, found := cache.entries[body]; found {
		return entry.geom, entry.ok, nil
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
	widen := proofbound.AbsSumUpper(deltaA, deltaB)
	if widen == 0 {
		return lo, hi, exact
	}
	lo = math.Max(0, freeform.DownRound(lo-widen))
	hi = proofbound.AbsSumUpper(hi, widen)
	return lo, hi, false
}

// newPairKernel builds one pair's working state over two carrier models. Its
// scale is the largest coordinate magnitude of either body's bounds, at least
// 1, and the tolerance and slack are 1e-9 of it.
func newPairKernel(ctx context.Context, ga, gb *bodyGeom) *pairKernel {
	scale := 1.0
	for _, g := range []*bodyGeom{ga, gb} {
		for _, p := range []r3.Vec{g.body.n.Bounds.Min, g.body.n.Bounds.Max} {
			for _, c := range []float64{p.X, p.Y, p.Z} {
				if v := math.Abs(c); v > scale {
					scale = v
				}
			}
		}
	}
	return &pairKernel{a: ga, b: gb, scale: scale, tol: 1e-9 * scale, slack: 1e-9 * scale, ctx: ctx}
}

// clearancePair runs the kernel over one pair of proven solids.
// nestingExcluded is true when box separation has already excluded nesting
// (a box-proven pair needs the kernel only for its gap — §7).
//
// Check order is load-bearing: the coplanar Plane×Plane contact certificate
// and then the ruling certificates run first and short-circuit every later
// check when one certifies a touch, because a separating plane already
// certifies the whole contact set — but ONLY when both bodies' carriers are
// exact (bodyGeom.delta zero; carrierDelta for the rulings), since the
// certificate is an exact material-side claim (payload-verification §7.2's
// rule for the faceted case, applied here to every analytic arm) that a
// carrier built from rounded coordinates cannot honestly make. Past that, an
// admitted transversal crossing (sink.Overlap) is read before sink.Unsure, so
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
	budget := proofbound.NewWorkBudget(ctx)
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
	k := newPairKernel(ctx, ga, gb)
	k.cache = cache
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
		return pairResult{verdict: reportvocab.PairTouching, exact: true, diam: diam}, nil
	}
	// The ruling certificates (§6) make the same exact separating-plane
	// claim along a tangent ruling. They read only carriers and exact
	// payload extents, so their gate is the carrier part of each delta.
	if ga.carrierDelta == 0 && gb.carrierDelta == 0 {
		ruling, err := k.rulingContactCertified(ctx)
		if err != nil {
			return pairResult{}, err
		}
		if ruling != nil {
			return pairResult{verdict: reportvocab.PairTouching, exact: true, diam: diam, ruling: ruling}, nil
		}
	}

	sink, err := k.enumerate()
	if err != nil {
		return pairResult{}, err
	}
	if sink.Overlap {
		return pairResult{verdict: reportvocab.PairOverlapping, diam: diam}, nil
	}
	if sink.Unsure {
		return pairResult{diam: diam}, nil
	}
	lo, hi, exact, ok := sink.Interval()
	if !ok {
		return pairResult{diam: diam}, nil
	}
	lo, hi, exact = clearanceDeltaWiden(lo, hi, exact, ga.widenDelta(), gb.widenDelta())
	if lo <= k.tol {
		return pairResult{diam: diam}, nil
	}
	if !nestingExcluded {
		verdict, contained, err := k.nestingRelation()
		if err != nil {
			return pairResult{}, err
		}
		if verdict != reportvocab.PairDisjoint {
			return pairResult{verdict: verdict, diam: diam, contained: contained}, nil
		}
	}
	return pairResult{verdict: reportvocab.PairDisjoint, lo: lo, hi: hi, exact: exact, diam: diam}, nil
}

// coplanarContactCertified scans the plane-face pairs for the §6 coplanar
// certificate.
func (k *pairKernel) coplanarContactCertified(ctx context.Context) (bool, error) {
	if err := ctx.Err(); err != nil {
		return false, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, fa := range k.a.faces {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if fa.Kind != clearance.CkPlane {
			continue
		}
		for _, fb := range k.b.faces {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if fb.Kind != clearance.CkPlane {
				continue
			}
			// The certificate is EXACT and reject-only: a zero row may
			// only come from a proven contact, so any nonzero plane
			// separation, tilt, or side penetration leaves the pair
			// undecided — a tolerance here would bless a real sub-tol
			// overlap as a Sound Exact-zero clearance.
			if fa.N.Dot(fb.N) != -1 {
				continue // not exactly opposing
			}
			if fb.O.Sub(fa.O).Dot(fa.N) != 0 {
				continue // not exactly coplanar
			}
			rel, _, err := clearance.CoplanarRelation(budget, fa, fb)
			if err != nil {
				return false, err
			}
			if rel != 1 {
				continue // no proven positive-area overlap
			}
			c := fa.PlaneOffset()
			aLo, aHi, okA := payloadExtent(ctx, k.a.body, fa.N)
			bLo, bHi, okB := payloadExtent(ctx, k.b.body, fa.N)
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
	switch pl := b.n.Payload.(type) {
	case prismPayload:
		lo, hi, err := pl.ExtentAlongContext(ctx, g)
		if err != nil {
			return 0, 0, false
		}
		return lo, hi, true
	case revolvePayload:
		lo, hi, err := pl.ExtentAlongContext(ctx, g)
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
func (k *pairKernel) nestingRelation() (reportvocab.PairVerdict, *Body, error) {
	type membership struct {
		inside, outside, unknown int
	}
	budget := proofbound.NewWorkBudget(k.ctx)
	classify := func(inner, outer *bodyGeom) (membership, error) {
		var got membership
		for _, w := range inner.shellWit {
			if err := budget.Step(); err != nil {
				return membership{}, err
			}
			inside, ok, err := clearance.PointInBody(k.ctx, outer.faces, w, k.tol)
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
		return reportvocab.PairUndecided, nil, err
	}
	ba, err := classify(k.b, k.a)
	if err != nil {
		return reportvocab.PairUndecided, nil, err
	}
	contains := func(inner, outer membership, innerWit, outerWit []r3.Vec) bool {
		return len(innerWit) > 0 && len(outerWit) > 0 &&
			inner.inside == len(innerWit) && outer.outside == len(outerWit)
	}
	if contains(ab, ba, k.a.shellWit, k.b.shellWit) {
		return reportvocab.PairOverlapping, k.a.body, nil
	}
	if contains(ba, ab, k.b.shellWit, k.a.shellWit) {
		return reportvocab.PairOverlapping, k.b.body, nil
	}
	if ab.inside > 0 || ba.inside > 0 {
		return reportvocab.PairOverlapping, nil, nil
	}
	if ab.unknown > 0 || ba.unknown > 0 {
		return reportvocab.PairUndecided, nil, nil
	}
	return reportvocab.PairDisjoint, nil, nil
}

// pairDiameter reads the pair's diameter D — the greatest distance between
// two points drawn from either body — through pairGateDiameter (§7,
// verification §3): points each body proves lie on it, each charged its
// proven gap. It never reads the carriers' witnesses (CFace.Wit), whose gaps
// (clearance.Witness) are proven to the carriers, not to the body. The
// reading may understate the true diameter, which only lowers the noise
// floor: the safe direction.
func (k *pairKernel) pairDiameter() (float64, error) {
	return pairGateDiameterCached(k.ctx, k.a.body, k.b.body, k.cache)
}

// sheetSolidPair decides docs/surface-design.md §9.3's pair procedure for a
// sheet operand against a proven solid: run when their bounds-inflated boxes
// meet, and also when they are separated but a gap was requested — the same
// two occasions clearancePair itself runs for a solid pair. boxDisjoint is
// true in that second case: the caller already proved the boxes separated,
// so the answer here can only be reportvocab.SheetSolidOutside or reportvocab.SheetSolidUndecided,
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
func sheetSolidPair(ctx context.Context, sheet, solid *Body, boxDisjoint bool, cache *bodyGeomCache) (reportvocab.SheetSolidResult, error) {
	if err := ctx.Err(); err != nil {
		return reportvocab.SheetSolidResult{}, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	gs, oks, err := cache.get(budget, sheet)
	if err != nil {
		return reportvocab.SheetSolidResult{}, err
	}
	gb, okb, err := cache.get(budget, solid)
	if err != nil {
		return reportvocab.SheetSolidResult{}, err
	}
	if !oks || !okb {
		// Either operand's carrier model is missing — a revolve sheet is
		// refused a model outright, and a multi-lump or faceted operand
		// bypasses this analytic kernel entirely (docs/clearance-design.md
		// §2). The pair stays undecided rather than guessing a side.
		return reportvocab.SheetSolidResult{Verdict: reportvocab.SheetSolidUndecided}, nil
	}
	k := newPairKernel(ctx, gs, gb)
	k.cache = cache
	diam, err := k.pairDiameter()
	if err != nil {
		return reportvocab.SheetSolidResult{}, err
	}

	// No coplanarContactCertified call here — see the doc comment above.
	sink, err := k.enumerate()
	if err != nil {
		return reportvocab.SheetSolidResult{}, err
	}
	if sink.Overlap {
		return reportvocab.SheetSolidResult{Verdict: reportvocab.SheetSolidCrossing, Diam: diam}, nil
	}
	if sink.Unsure {
		return reportvocab.SheetSolidResult{Verdict: reportvocab.SheetSolidUndecided, Diam: diam}, nil
	}
	lo, hi, exact, ok := sink.Interval()
	if !ok {
		return reportvocab.SheetSolidResult{Verdict: reportvocab.SheetSolidUndecided, Diam: diam}, nil
	}
	lo, hi, exact = clearanceDeltaWiden(lo, hi, exact, gs.widenDelta(), gb.widenDelta())
	if lo <= k.tol {
		return reportvocab.SheetSolidResult{Verdict: reportvocab.SheetSolidUndecided, Diam: diam}, nil
	}

	if boxDisjoint {
		// Box separation already proves the sheet lies wholly outside the
		// solid (docs/surface-design.md §9.3): the witness cast below would
		// only confirm what a non-overlapping bounding box already decided,
		// the same shortcut clearancePair takes via nestingExcluded.
		return reportvocab.SheetSolidResult{Verdict: reportvocab.SheetSolidOutside, Lo: lo, Hi: hi, Exact: exact, Diam: diam}, nil
	}

	// The one-shell premise the doc comment above states, asserted rather
	// than assumed: newBodyGeomBudget already refuses any body with more
	// than one lump, and a sheet lump holds exactly one shell
	// (docs/surface-design.md §2.2), so gs.shellWit carries exactly one
	// witness here. A future sheet construction that broke that premise
	// would land here undecided rather than pick a witness arbitrarily.
	if len(gs.shellWit) != 1 {
		return reportvocab.SheetSolidResult{Verdict: reportvocab.SheetSolidUndecided, Diam: diam}, nil
	}
	inside, ok, err := clearance.PointInBody(ctx, gb.faces, gs.shellWit[0], k.tol)
	if err != nil {
		return reportvocab.SheetSolidResult{}, err
	}
	if !ok {
		return reportvocab.SheetSolidResult{Verdict: reportvocab.SheetSolidUndecided, Diam: diam}, nil
	}
	if inside {
		return reportvocab.SheetSolidResult{Verdict: reportvocab.SheetSolidContained, Lo: lo, Hi: hi, Exact: exact, Diam: diam}, nil
	}
	return reportvocab.SheetSolidResult{Verdict: reportvocab.SheetSolidOutside, Lo: lo, Hi: hi, Exact: exact, Diam: diam}, nil
}
