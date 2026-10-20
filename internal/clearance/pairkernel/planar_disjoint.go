package pairkernel

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
)

// DisjointResult widens the held gap's enclosure by delta, in exact
// rationals, and publishes the pair disjoint only where the widened lower
// end stays positive after rounding down. exact holds only when the held
// gap is a single float and delta is zero, so the interval is a point.
func DisjointResult(gap *pair.ScalarReading, delta *big.Rat, diam float64) (Result, bool) {
	if gap == nil {
		return Result{}, false
	}
	value, bound := proofarith.FloatRat(gap.ValueMM), proofarith.FloatRat(gap.BoundMM)
	loRat := new(big.Rat).Sub(value, bound)
	loRat.Sub(loRat, delta)
	hiRat := new(big.Rat).Add(value, bound)
	hiRat.Add(hiRat, delta)
	lo, hi := proofbound.RatFloatDown(loRat), proofbound.RatFloatUp(hiRat)
	if !(lo > 0) || !measurement.FiniteValues(lo, hi) {
		return Result{}, false
	}
	return Result{Verdict: reportvocab.PairDisjoint, Lo: lo, Hi: hi, Exact: lo == hi, Diam: diam}, true
}
