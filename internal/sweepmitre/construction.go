package sweepmitre

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/r3"
)

// SpanLengthLower returns λ_j of §16.3: the largest float whose square does
// not exceed the exact squared length. DySqrtDown fixes the precision, so
// the join plane does not depend on platform sqrt or FMA contraction.
func SpanLengthLower(start, end r3.Vec) (*big.Rat, error) {
	d := proofarith.DvSub(proofarith.DyVec(end), proofarith.DyVec(start))
	lambda := proofarith.DySqrtDown(proofarith.DvDot(d, d))
	if !(lambda > 0) || math.IsInf(lambda, 0) {
		return nil, fmt.Errorf(`%w: the span's length has no positive float lower bound`, decaderr.ErrUnsupported)
	}
	return proofarith.FloatRat(lambda), nil
}

// Factor reads f_k with f_0 equal to one.
func Factor(factors []float64, k int) *big.Rat {
	if k == 0 {
		return big.NewRat(1, 1)
	}
	return proofarith.FloatRat(factors[k-1])
}

// Lift maps one plane-local point through the recorded plane exactly.
func Lift(plane sectionrecord.PlaneRecord, p sectionrecord.Point2) sweeparc.RatVec {
	u, v := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
	return sweeparc.Add(sweeparc.VecOf(plane.Origin), sweeparc.Scale(sweeparc.VecOf(plane.U), u),
		sweeparc.Scale(sweeparc.VecOf(plane.V), v))
}

// RequireWalls enforces SM7: every wall quad p, q, q', p' has a nonzero
// exact area vector (q' − p) × (p' − q), and end vertices do not coincide.
func RequireWalls(k int, loopIdx [][]int, from, to []sweeparc.RatVec) error {
	for i, idx := range loopIdx {
		m := len(idx)
		for j := range m {
			p, q := from[idx[j]], from[idx[(j+1)%m]]
			pn, qn := to[idx[j]], to[idx[(j+1)%m]]
			if sweeparc.IsZero(sweeparc.Cross(sweeparc.Sub(qn, p), sweeparc.Sub(pn, q))) {
				return fmt.Errorf(`%w: mitred sweep span %d, loop %d, %s`, decaderr.ErrDegenerate, k, i,
					fmt.Sprintf("segment %d: its wall quad has zero area", j))
			}
		}
	}
	seen := make(map[string]int, len(to))
	for v, p := range to {
		key := p[0].RatString() + "," + p[1].RatString() + "," + p[2].RatString()
		if _, dup := seen[key]; dup {
			return fmt.Errorf(`%w: mitred sweep span %d's end section has two coincident vertices`, decaderr.ErrDegenerate, k)
		}
		seen[key] = v
	}
	return nil
}

// Place applies the accumulated placement to a rational point exactly.
// A Transform's basis and translation entries are floats, hence rationals.
func Place(xform r3.Transform, p sweeparc.RatVec) sweeparc.RatVec {
	if xform == r3.Identity() {
		return p
	}
	basis := xform.Basis()
	return sweeparc.Add(
		sweeparc.Scale(sweeparc.VecOf(basis.EX), p[0]),
		sweeparc.Scale(sweeparc.VecOf(basis.EY), p[1]),
		sweeparc.Scale(sweeparc.VecOf(basis.EZ), p[2]),
		sweeparc.VecOf(xform.Translation()),
	)
}

// Round returns a rational coordinate's nearest float and exact gap.
func Round(r *big.Rat) (float64, *big.Rat, bool) {
	f, _ := r.Float64()
	if math.IsInf(f, 0) || math.IsNaN(f) {
		return 0, nil, false
	}
	gap := new(big.Rat).Sub(r, proofarith.FloatRat(f))
	return f, gap.Abs(gap), true
}
