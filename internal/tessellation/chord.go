package tessellation

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ChordWalkMin is docs/tessellation-design.md §3's default walk minimum: a
// whole closed curve needs at least three chords to bound a polygon, and every
// other walk needs one. A revolve meridian has a third case of its own
// (revolvesampling.MeridianMin), which is why the minimum is chordCount's parameter
// rather than a property it reads off the walk.
func ChordWalkMin(w survey2d.SegmentWalk) int {
	if w.Closed {
		return 3
	}
	return 1
}

// ChordCount picks the number of chords for a circular walk so chordSagitta's
// PROVEN bound on each chord's sagitta stays within tol, and returns that
// bound. Every accept/reject below reads chordSagitta, never the true
// sagitta r·(1 − cos(Δθ/2)) it encloses. The per-chord angle never exceeds π,
// so consecutive samples are always distinct.
//
// nMin is the walk's own minimum count (chordWalkMin, or revolvesampling.MeridianMin for
// a revolve meridian). docs/tessellation-design.md §3 asks for it to be tried
// FIRST: when its own sagitta already fits the budget it is the answer, and the
// inverse below — which is undefined for a budget at or above 2r — is never
// evaluated at all.
func ChordCount(w survey2d.SegmentWalk, tol float64, nMin int) (int, float64, error) {
	sweep := math.Abs(w.Th1 - w.Th0)
	if nMin < 1 {
		nMin = 1
	}
	if nMin > freeform.MaxChordsPerWalk {
		return 0, 0, freeform.ErrTooManyChords
	}
	if s := ChordSagitta(w.Radius, sweep, nMin); s <= tol {
		return nMin, s, nil
	}
	maxD := math.Pi
	if tol < w.Radius {
		// The TRUE sagitta 2r·sin²(Δθ/4) inverts to Δθ_max = 4·asin(√(tol/2r))
		// — the stable inverse: acos(1 − tol/r) rounds to zero for tiny tol
		// and would seed an unbounded walk-up. It seeds the search only; the
		// walk-up below decides on chordSagitta's larger proven figure, so
		// this seed can land one chord short and is corrected there.
		maxD = 4 * math.Asin(math.Sqrt(tol/(2*w.Radius)))
	}
	// A tolerance this fine asks for more chords than any mesh can hold;
	// the intent cannot be built, so it is refused outright (evaluator §2).
	// The cap is enforced again after ceil and inside the walk-up: rounding
	// can push n past a precheck that barely admitted it.
	if maxD == 0 || sweep/maxD > freeform.MaxChordsPerWalk {
		return 0, 0, freeform.ErrTooManyChords
	}
	n := max(int(math.Ceil(sweep/maxD)), nMin)
	if n > freeform.MaxChordsPerWalk {
		return 0, 0, freeform.ErrTooManyChords
	}
	// The returned sagitta is a PROVEN bound, so it may never exceed the
	// asked tolerance: float rounding in the asin/ceil path can land one
	// chord short at a threshold value. Each increment strictly shrinks the
	// sagitta toward zero, so the walk-up terminates.
	s := ChordSagitta(w.Radius, sweep, n)
	for s > tol && sweep > 0 {
		if n == freeform.MaxChordsPerWalk {
			return 0, 0, freeform.ErrTooManyChords
		}
		n++
		s = ChordSagitta(w.Radius, sweep, n)
	}
	return n, s, nil
}

// ChordSagitta is a PROVEN upper bound on the sagitta 2r·sin²(Δθ/4) a chord
// subtends when a circular walk of the given sweep is split into n equal
// chords — docs/tessellation-design.md §3's published sagitta, and the
// single owner of chordCount's walk-up step, so a later caller measuring the
// SAME quantity for a hand-chorded fixture reads the identical closed form
// rather than a second copy of it.
//
// It never calls math.Sin. sin(x) <= x for every x >= 0 — the tangent line
// to sin at the origin lies above the curve on the whole nonnegative axis,
// an elementary inequality, not a derived one — so with x = sweep/(4n),
// 2r·sin²(x) <= 2r·x² = r·sweep²/(8n²). That bound needs no trig call at
// all, so it carries none of Sin's own missing ulp contract (Go publishes
// none) and none of the FMA-contraction difference between amd64 and arm64
// a Sin-based formula would expose: every operation below is an ordinary
// multiply or divide, each individually outward-rounded, the same
// single-operation proof every other bound in this package rests on.
//
// The denominator 8n² is computed with PLAIN arithmetic, never
// outward-rounded: for every n this package ever calls with (n <=
// freeform.MaxChordsPerWalk = 1<<14, so 8n² <= 2^31, far under float64's 2^53
// exact-integer range) the computation commits no rounding at all, and
// outward-rounding a DENOMINATOR would move the bound the WRONG way — a
// larger denominator gives a SMALLER, tighter, and here unproven quotient.
// The numerator r·sweep² is outward-rounded through proofbound.ProductUpper, and the
// one division is outward-rounded by a final proofbound.UpRound, exactly the
// single-operation margin proofbound.ProductUpper already applies to a multiply.
//
// That float chain is a proven upper bound wherever it lands on a positive
// value, and it is the only thing this helper publishes there. What it may
// NOT do is answer ZERO for a positive r·sweep²/(8n²): rounding outward
// cannot rescue an UNDERFLOW, since proofbound.UpRound leaves 0 alone, and both the
// sweep·sweep product and the final quotient can reach 0 from operands that
// are each positive. chordCount hands this figure on as the walk's proven
// sagitta and [Mesh.Bound] publishes it, so a zero there would claim an
// exact chording for a nonzero deviation and would end chordCount's walk-up
// loop on a false reading. A non-positive answer from the float chain
// therefore falls through to exactChordSagitta, which states the same bound
// over the rationals.
//
// The bound sits above the true sagitta by the factor (x/sin x)² — about
// 1.0000126 at a typical fine chording (90 degrees over 64 stations) and
// π²/9 in the coarsest case a closed walk can reach (n=3 on a full circle;
// TestChordSagittaCoarsestClosedWalkStaysProven pins that ratio). Two
// consequences reach a caller, and [Body.Tessellate]'s doc comment owns
// both in the caller's own terms: [Mesh.Bound] reads by that factor above
// the deviation the chords take, and chordCount's walk-up settles on a
// slightly larger n than the true sagitta alone would need — up to and
// including hitting freeform.MaxChordsPerWalk and returning freeform.ErrTooManyChords for a
// tol within (x/sin x)²−1 of the finest chording the cap admits. Both are
// the safe direction: a finer mesh, or a typed refusal, never a coarser
// mesh under a claim this package cannot prove.
// TestChordCountRefusesTheToleranceWindowAtTheMeshCap pins that refusal at
// its exact boundary.
func ChordSagitta(radius, sweep float64, n int) float64 {
	if n <= 0 || sweep < 0 {
		// A non-positive n or a negative sweep would otherwise read as
		// sagitta 0 — proofbound.ProductUpper(sweep, sweep) already zeroes a negative
		// sweep — which UNDERSTATES the true, positive sagitta rather than
		// falsifying the claim (proofbound.CutDisplacementAllow's own rule: an absent
		// bound must never read as a small one).
		return math.Inf(1)
	}
	if radius < 0 {
		// The true sagitta r·(1−cos) is itself non-positive for a negative
		// radius, so 0 is a valid (if unattained) upper bound here — no
		// refusal needed.
		return 0
	}
	if proofbound.IsNonFinite(radius) || proofbound.IsNonFinite(sweep) {
		// A non-finite claim states no bound at all. It refuses with +Inf for
		// the same reason the two arms above do, rather than carrying the NaN
		// the arithmetic below would otherwise publish as a bound.
		return math.Inf(1)
	}
	denom := 8 * float64(n) * float64(n)
	if denom <= 0 || proofbound.IsNonFinite(denom) {
		// 8n² saturated, so the true bound r·sweep²/(8n²) is itself driven to
		// zero and 0 remains an upper bound.
		return 0
	}
	if radius == 0 || sweep == 0 {
		// The true sagitta is exactly zero, and so is its bound.
		return 0
	}
	if s := proofbound.UpRound(proofbound.ProductUpper(radius, proofbound.ProductUpper(sweep, sweep)) / denom); s > 0 {
		return s
	}
	return exactChordSagitta(radius, sweep, n)
}

// exactChordSagitta states chordSagitta's own r·sweep²/(8n²) over the
// RATIONALS, for the one case its float chain cannot: a POSITIVE bound whose
// float evaluation underflows to zero. Two independent sites in that chain can
// underflow — sweep·sweep can fall below the smallest denormal on its own, and
// the final quotient can land on zero from a numerator that survived — and this
// helper covers both, because it is reached from the PUBLISHED value rather
// than from either site.
//
// radius and sweep are float64 and therefore exact rationals, and 8n² is an
// exact integer for every positive n, so the quotient here is the EXACT bound
// with no rounding anywhere in it. proofbound.RatFloatUp then rounds that one value
// outward, which answers the smallest positive float64 — never zero — for a
// bound too small for float64 to represent.
func exactChordSagitta(radius, sweep float64, n int) float64 {
	num := new(big.Rat).Mul(proofarith.FloatRat(radius), new(big.Rat).Mul(proofarith.FloatRat(sweep), proofarith.FloatRat(sweep)))
	nRat := new(big.Rat).SetInt64(int64(n))
	denom := new(big.Rat).Mul(new(big.Rat).SetInt64(8), new(big.Rat).Mul(nRat, nRat))
	return proofbound.RatFloatUp(num.Quo(num, denom))
}

// WalkAreaSlack is the proven chord-versus-arc area slack one circular walk
// contributes ACROSS TWO CAPS — a solid's own composition: the wall loses
// (arc − chord) × height, and each of the two caps gains or loses the
// circular segment between arc and chords — both closed form, both padded a
// hair so float rounding never understates them. It states the two-cap case
// only; chordLoop keeps the wall and one cap's own share apart
// (chordedLoop.wallSlack/capSlack) so a sheet mesh, which triangulates no
// cap, can decline the cap half entirely (docs/surface-design.md §10).
func WalkAreaSlack(w survey2d.SegmentWalk, n int, h float64) float64 {
	return (WalkWallSlack(w, n, h) + 2*WalkSegmentArea(w, n)) * (1 + 1e-9)
}

// WalkWallSlack is the WALL half of walkAreaSlack: the area one circular walk's
// side face loses by standing on n chords instead of its arc, (arc − chord) × h.
func WalkWallSlack(w survey2d.SegmentWalk, n int, h float64) float64 {
	sweep := math.Abs(w.Th1 - w.Th0)
	arc := sweep * w.Radius
	chord := float64(n) * 2 * w.Radius * math.Sin(sweep/(2*float64(n)))
	return math.Max(arc-chord, 0) * h
}

// WalkSegmentArea is the PLANAR half of walkAreaSlack, stated on its own
// because two proofs read it: the caps' own area slack reads it twice (one per
// cap), and docs/tessellation-reach-design.md §3's occupied-volume term reads it
// once per walk against the sweep height.
//
// It is the closed-form sum Σ a_c of the n circular segments between one
// circular walk's arc and its chords, r²/2 · (θ − n·sin(θ/n)) for a walk of
// radius r sweeping θ. The chorded section differs from the section it denotes
// by exactly those segments, so the prism between two levels differs from the
// prism it denotes by their sum times the sweep height — an area a hole gains
// and an outline loses, summed in ABSOLUTE value here because occupied volume
// admits no cancellation (docs/tessellation-design.md §2).
func WalkSegmentArea(w survey2d.SegmentWalk, n int) float64 {
	sweep := math.Abs(w.Th1 - w.Th0)
	return w.Radius * w.Radius / 2 * math.Max(sweep-float64(n)*math.Sin(sweep/float64(n)), 0)
}
