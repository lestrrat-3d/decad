package survey2d

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// ExtremeAggregate reduces a population of candidates, each holding a value
// under its OWN proven bound, to the single interval
// docs/payload-verification-design.md §9.2 specifies for the faceted twin of
// the minimum-radius survey. It is the one owner of that reduction: every
// survey reading that is an extremum over candidates — the concave radius, the
// wall reading's arms, and the 2D kernel's own spanning diameter and inradius —
// takes its interval from here rather than from whichever candidate won a
// comparison of held values.
//
// For a MINIMUM, lo is the least of the candidates' lower endpoints
// (value − bound) and hi the least of their upper endpoints (value + bound).
// Every true value is at least its own candidate's lower endpoint, so the true
// minimum is at least the least of them; and the candidate achieving the least
// upper endpoint has a true value no greater than that endpoint, so the true
// minimum is at most hi. For a MAXIMUM the same argument runs the other way,
// with both ends taken as maxima instead.
//
// Both endpoints are needed because each candidate's bound is its own.
// Reducing winner-only instead — keeping the winning held value together with
// that one candidate's bound — publishes an interval a RIVAL candidate's truth
// can sit outside whenever the rival carries the wider bound, and nothing the
// winner's own arithmetic knows can detect it.
//
// The reduction never widens an exact reading: when the candidates that decide
// both ends have a zero bound, lo and hi coincide, so the midpoint is that
// value and the half-width is zero. An inexact rival that does not reach the
// extremum changes neither end, so it cannot make an exact reading
// approximate.
//
// Every arm feeds one sink, so the refusal on an underivable bound lives here
// rather than on a winner: a candidate the arithmetic could not bound leaves
// the whole reading undecided however its held value ranks.
type ExtremeAggregate struct {
	Lo, Hi    *big.Rat
	Maximum   bool // reduce toward the greatest candidate rather than the least
	Unbounded bool
}

// MinAggregate and MaxAggregate name the two directions the reduction runs in.
func MinAggregate() ExtremeAggregate { return ExtremeAggregate{} }

func MaxAggregate() ExtremeAggregate { return ExtremeAggregate{Maximum: true} }

// take admits one candidate under its own proven bound, read as the magnitude
// it is. A candidate whose value or bound is not finite has no enclosure at
// all, so it refuses the aggregate rather than dropping silently out of the
// comparison.
func (ra *ExtremeAggregate) Take(value, bound float64) {
	v, b := proofarith.FloatRat(value), proofarith.FloatRat(math.Abs(bound))
	if v == nil || b == nil {
		ra.Unbounded = true
		return
	}
	keep := func(cur, cand *big.Rat) *big.Rat {
		if cur == nil {
			return cand
		}
		cmp := cand.Cmp(cur)
		if (ra.Maximum && cmp > 0) || (!ra.Maximum && cmp < 0) {
			return cand
		}
		return cur
	}
	ra.Lo = keep(ra.Lo, new(big.Rat).Sub(v, b))
	ra.Hi = keep(ra.Hi, new(big.Rat).Add(v, b))
}

// empty reports that no candidate was admitted at all — the proven absence its
// caller turns into whichever answer absence means for that reading.
func (ra ExtremeAggregate) Empty() bool { return ra.Lo == nil }

// resolve publishes the aggregate's midpoint under a half-width measured from
// the RATIONAL endpoints, so the float midpoint's own rounding is already
// inside the bound it publishes. It answers ok=false when a candidate could
// not be bounded, or when the interval itself cannot be stated in float64.
func (ra ExtremeAggregate) Resolve() (float64, float64, bool) {
	if ra.Unbounded || ra.Lo == nil {
		return 0, 0, false
	}
	sum := new(big.Rat).Add(ra.Lo, ra.Hi)
	mid, _ := new(big.Rat).Mul(sum, big.NewRat(1, 2)).Float64()
	if proofbound.IsNonFinite(mid) {
		return 0, 0, false
	}
	bound := math.Max(RatAbsDiff(ra.Lo, mid), RatAbsDiff(ra.Hi, mid))
	if proofbound.IsNonFinite(bound) {
		return 0, 0, false
	}
	return mid, bound, true
}
