package revolveloftfaces

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// SlopedRadialInside checks the exact radial slice of a whole-line section.
// The axis walks toward +X and the selected outer side walks toward -X.
// Exactly those two crossings must enclose the filled interval at x.
func SlopedRadialInside(section facetedrecord.RevolveSectionCertificate,
	spanIndex int, x, radius *big.Rat) bool {
	if x == nil || radius == nil || radius.Sign() <= 0 ||
		spanIndex < 0 || spanIndex >= len(section.Spans) ||
		section.AxisSpan < 0 || section.AxisSpan >= len(section.Spans) ||
		spanIndex == section.AxisSpan {
		return false
	}
	axis := section.Spans[section.AxisSpan]
	if axis.Start.V != 0 || axis.End.V != 0 ||
		proofarith.FloatRat(axis.Start.U).Cmp(x) >= 0 ||
		proofarith.FloatRat(axis.End.U).Cmp(x) <= 0 {
		return false
	}
	found := false
	for i, span := range section.Spans {
		startX, endX := proofarith.FloatRat(span.Start.U), proofarith.FloatRat(span.End.U)
		if startX.Cmp(x) == 0 || endX.Cmp(x) == 0 {
			return false
		}
		if i == section.AxisSpan {
			continue
		}
		if startX.Cmp(x) == endX.Cmp(x) {
			continue
		}
		if i != spanIndex || startX.Cmp(x) <= 0 || endX.Cmp(x) >= 0 ||
			span.Start.V < 0 || span.End.V < 0 {
			return false
		}
		fraction := proofarith.QuoRat(new(big.Rat),
			proofarith.SubRat(new(big.Rat), x, startX),
			proofarith.SubRat(new(big.Rat), endX, startX))
		atX := proofarith.AddRat(new(big.Rat), proofarith.FloatRat(span.Start.V),
			proofarith.MulRat(new(big.Rat), fraction,
				proofarith.SubRat(new(big.Rat), proofarith.FloatRat(span.End.V),
					proofarith.FloatRat(span.Start.V))))
		if atX.Cmp(radius) != 0 {
			return false
		}
		found = true
	}
	return found
}
