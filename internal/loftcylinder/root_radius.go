package loftcylinder

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// RootRadiusGap bounds how far both sections' root arc starts sit from radius
// under the placement's XY scale; ok is false at 0.01 or more.
func RootRadiusGap(xform r3.Transform, profiles [2]momentinput.Profile, radius float64) (float64, bool) {
	gap := 0.0
	basis := xform.Basis()
	c, s := proofarith.FloatRat(basis.EX.X), proofarith.FloatRat(basis.EX.Y)
	scale2 := new(big.Rat).Add(new(big.Rat).Mul(c, c), new(big.Rat).Mul(s, s))
	for _, profile := range profiles[:] {
		arc, ok := profile.Outer.Segments[5].(sectionrecord.ArcSeg)
		if !ok || proofbound.IsNonFinite(arc.Start.U) || proofbound.IsNonFinite(arc.Start.V) {
			return 0, false
		}
		x, y := proofarith.FloatRat(arc.Start.U), proofarith.FloatRat(arc.Start.V)
		q := new(big.Rat).Mul(scale2,
			new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y)))
		lo, hi := proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q)
		if lo <= 0 || proofbound.IsNonFinite(hi) {
			return 0, false
		}
		r := proofarith.FloatRat(radius)
		d0 := new(big.Rat).Abs(new(big.Rat).Sub(proofarith.FloatRat(lo), r))
		d1 := new(big.Rat).Abs(new(big.Rat).Sub(proofarith.FloatRat(hi), r))
		if d0.Cmp(d1) < 0 {
			d0 = d1
		}
		gap = max(gap, proofbound.RatFloatUp(d0))
	}
	return gap, gap < 0.01
}
