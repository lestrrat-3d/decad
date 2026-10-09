package motionbound

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// CertificatePair states how one pair enters a motion interval or joint box
// certificate. A skipped pair has no obligation; an excluded pair contributes
// its whole-call lower bound; every other pair needs a positive proof.
type CertificatePair struct {
	Skip      bool
	Excluded  bool
	Lower     float64
	Evaluated bool
}

// IntervalPairLower combines the endpoint travel and projection proofs. The
// endpoint gaps are floats whose exact rational values were already proven.
// A nil or nonpositive result means neither proof certifies the pair.
func IntervalPairLower(loA, loB float64, travel, projection *big.Rat) *big.Rat {
	var lower *big.Rat
	if travel != nil {
		lower = new(big.Rat).Add(proofarith.FloatRat(loA), proofarith.FloatRat(loB))
		lower.Sub(lower, travel)
		lower.Quo(lower, big.NewRat(2, 1))
	}
	if projection != nil && (lower == nil || projection.Cmp(lower) > 0) {
		lower = projection
	}
	if lower == nil || lower.Sign() <= 0 {
		return nil
	}
	return lower
}

// CertifyPairs visits pairs in row order. It returns the smallest proven lower
// bound, or nil when no pair contributes one. When held is nil it stops at the
// first pair without a proof; otherwise it reports every such pair and keeps
// the lower bound from the pairs that did certify.
func CertifyPairs(rows int, rowLen func(int) int, pairAt func(int, int) CertificatePair,
	certify func(int, int) *big.Rat, held func(int, int)) (*big.Rat, bool) {
	var lowest *big.Rat
	ok := true
	for i := range rows {
		for k := range rowLen(i) {
			pair := pairAt(i, k)
			if pair.Skip {
				continue
			}
			if pair.Excluded {
				lowest = smallerRat(lowest, proofarith.FloatRat(pair.Lower))
				continue
			}
			var bound *big.Rat
			if pair.Evaluated {
				bound = certify(i, k)
			}
			if bound != nil {
				lowest = smallerRat(lowest, bound)
				continue
			}
			if held == nil {
				return nil, false
			}
			ok = false
			held(i, k)
		}
	}
	return lowest, ok
}

func smallerRat(running, candidate *big.Rat) *big.Rat {
	if running == nil || candidate.Cmp(running) < 0 {
		return candidate
	}
	return running
}
