package clearance

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// CellSink accumulates contributions and the undecidable findings.
type CellSink struct {
	Contribs []GapContrib
	// overlap is set only by a trim-admitted transversal boundary crossing.
	// Such a crossing proves a shared open material neighborhood.
	Overlap bool
	// unsure is set when a cell meets a question it cannot decide: an
	// admitted-or-ambiguous carrier crossing, an uncertified contact, an
	// equality where a branch demands strictness (§4: equality routes to §6,
	// where only the coplanar plane pair is certified).
	Unsure bool

	// prune enables §5's cell pruning (pruned, below). A zero-value sink
	// never prunes, so a cell run on its own reports everything it finds.
	Prune bool
	// margin is the length charged against a box distance before it may
	// prune. The kernel passes its slack, 1e-9 × the pair's coordinate
	// scale, which covers the few-ulp rounding of the float boxes, of the
	// box distance and of the subtraction many times over.
	Margin float64
	// best is the smallest finite contribution hi among contribs[:seen].
	Best float64
	Seen int
	// skipped counts the cells pruned so far.
	Skipped int
}

// NewPruningSink returns a sink that prunes against its own best upper
// bound, charging margin against every box distance.
func NewPruningSink(margin float64) *CellSink {
	return &CellSink{Prune: true, Margin: margin, Best: math.Inf(1)}
}

// Pruned reports whether a cell whose two features' boxes lie lb apart
// cannot hold the pair's minimum, and counts it when so (§5). Every
// contribution's hi bounds the true gap from above, and every point of the
// cell's features lies at least lb − margin from the other's, so a cell with
// lb − margin STRICTLY above the best hi in hand lies wholly beyond the
// minimum. Equality never prunes: a feature pair at exactly the best upper
// bound may hold the minimum itself. A non-finite lb never prunes.
func (s *CellSink) Pruned(lb float64) bool {
	if !s.Prune || proofbound.IsNonFinite(lb) {
		return false
	}
	for _, c := range s.Contribs[s.Seen:] {
		if c.Hi < s.Best {
			s.Best = c.Hi
		}
	}
	s.Seen = len(s.Contribs)
	if lb-s.Margin <= s.Best {
		return false
	}
	s.Skipped++
	return true
}

// Interval folds the contributions into the held-candidate gap interval
// [lo, hi] (§1): hi is the least upper bound, lo the least lower bound below
// it, and exact holds only for a closed-form winner at hi with every rival's
// lo at or above it. ok is false when no contribution carries a finite hi.
func (s *CellSink) Interval() (float64, float64, bool, bool) {
	hi := math.Inf(1)
	for _, c := range s.Contribs {
		if c.Hi < hi {
			hi = c.Hi
		}
	}
	if math.IsInf(hi, 1) {
		return 0, 0, false, false
	}
	lo := hi
	for _, c := range s.Contribs {
		if c.Lo < lo {
			lo = c.Lo
		}
	}
	exact := false
	for _, c := range s.Contribs {
		if c.Exact && c.Lo == hi && c.Hi == hi {
			exact = true
		}
	}
	return lo, hi, exact && lo == hi, true
}

// Crossing records a carrier crossing after trim admission: admitted proves
// overlap, rejected proves absence, and a boundary-straddling admission stays
// undecided.
func (s *CellSink) Crossing(admit int) {
	switch admit {
	case 1:
		s.Overlap = true
	case 0:
		s.Unsure = true
	}
}

// Candidate folds an admission state into a contribution: rejected feet are
// discarded (a lower tier holds the minimum), a straddle keeps only the
// lower bound, and a near-zero value that is not cleanly rejected is a
// possible contact — undecided.
func (s *CellSink) Candidate(tol float64, admit int, lo, hi float64, exact bool, pa, pb r3.Vec) {
	if admit == -1 {
		return
	}
	if lo <= tol {
		s.Unsure = true
		return
	}
	if admit == 0 {
		s.Contribs = append(s.Contribs, GapContrib{Lo: lo, Hi: math.Inf(1)})
		return
	}
	s.Contribs = append(s.Contribs, GapContrib{Lo: lo, Hi: hi, Exact: exact})
}

// LoOnly contributes a bare proven lower bound.
func (s *CellSink) LoOnly(lo float64) {
	s.Contribs = append(s.Contribs, GapContrib{Lo: math.Max(0, lo), Hi: math.Inf(1)})
}

// Coarse contributes a conservative enclosure for a pair no shipped cell can
// solve: the boxes' distance below, the closest admitted witness pair above
// (§5 — enclosure distance never exceeds true distance, a witness is always
// an upper bound).
func (s *CellSink) Coarse(boxA, boxB [2]r3.Vec, witA, witB []r3.Vec) {
	lo := ClrBoxDist(boxA, boxB)
	hi := math.Inf(1)
	for _, wa := range witA {
		for _, wb := range witB {
			if d := wa.Sub(wb).Len(); d < hi {
				hi = d
			}
		}
	}
	s.Contribs = append(s.Contribs, GapContrib{Lo: math.Max(0, lo), Hi: hi})
}
