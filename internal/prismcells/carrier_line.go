package prismcells

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// SameLineCarrier compares complete supporting lines in exact dyadic arithmetic.
// A shared carrier needs a separate source-span proof before scene construction.
func SameLineCarrier(a, b sectionrecord.LineSeg) bool {
	startU, startV := proofarith.FloatRat(a.Start.U), proofarith.FloatRat(a.Start.V)
	dx := proofarith.SubRat(new(big.Rat), proofarith.FloatRat(a.End.U), startU)
	dy := proofarith.SubRat(new(big.Rat), proofarith.FloatRat(a.End.V), startV)
	for _, endpoint := range []sectionrecord.Point2{b.Start, b.End} {
		ex := proofarith.SubRat(new(big.Rat), proofarith.FloatRat(endpoint.U), startU)
		ey := proofarith.SubRat(new(big.Rat), proofarith.FloatRat(endpoint.V), startV)
		cross := proofarith.SubRat(new(big.Rat),
			proofarith.MulRat(new(big.Rat), dx, ey),
			proofarith.MulRat(new(big.Rat), dy, ex))
		if cross.Sign() != 0 {
			return false
		}
	}
	return true
}
