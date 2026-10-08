package massmoment

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Moments is one solid's V = ∫dV, P_i = ∫q_i dV and Q_ij = ∫q_i·q_j dV
// as rational intervals, q the coordinates about the anchor the producer
// names (docs/dynamic-mass-design.md §2). Second is filled symmetrically.
type Moments struct {
	Volume proofbound.RatInterval
	First  [3]proofbound.RatInterval
	Second [3][3]proofbound.RatInterval
}

// Shift re-anchors m from anchor a to anchor a − s: with
// q' = q + s, P' = P + V·s and Q'_ij = Q_ij + s_i·P_j + P_i·s_j + V·s_i·s_j.
// s is exact and the map is a polynomial in V, P and Q, so its interval
// evaluation encloses the re-anchored moments of every solid the input
// encloses. It is an exact change of anchor, not a parallel-axis estimate.
func Shift(m Moments, s [3]*big.Rat) Moments {
	out := Moments{Volume: m.Volume}
	for i := range out.First {
		out.First[i] = proofbound.IntervalAdd(m.First[i], proofbound.IntervalScale(m.Volume, s[i]))
	}
	for i := range out.Second {
		for j := range out.Second[i] {
			term := proofbound.IntervalAdd(m.Second[i][j], proofbound.IntervalScale(m.First[j], s[i]))
			term = proofbound.IntervalAdd(term, proofbound.IntervalScale(m.First[i], s[j]))
			out.Second[i][j] = proofbound.IntervalAdd(term, proofbound.IntervalScale(m.Volume, new(big.Rat).Mul(s[i], s[j])))
		}
	}
	return out
}

// Transform carries m through the exact linear map f, f's column k the
// image of local axis k: the image solid's moments are V′ = |det f|·V,
// P′ = |det f|·f·P and Q′ = |det f|·f·Q·fᵀ, since f scales every volume
// element by |det f|. It is exact for the image whatever f's departure from
// orthonormal, the map every analytic payload's readings denote through
// (docs/evaluator-design.md §5.1).
func Transform(m Moments, f [3][3]*big.Rat) Moments {
	det := Determinant(f)
	det.Abs(det)
	out := Moments{Volume: proofbound.IntervalScale(m.Volume, det)}
	for i := range out.First {
		sum := proofbound.PointInterval(new(big.Rat))
		for k := range m.First {
			sum = proofbound.IntervalAdd(sum, proofbound.IntervalScale(m.First[k], f[i][k]))
		}
		out.First[i] = proofbound.IntervalScale(sum, det)
	}
	second := RotateTensor(f, m.Second)
	for i := range second {
		for j := range second[i] {
			out.Second[i][j] = proofbound.IntervalScale(second[i][j], det)
		}
	}
	return out
}

// Add and Sub combine two solids' moments about one
// anchor. Each operand keeps its own outward interval, so an uncertainty
// never cancels against a neighbor's (docs/dynamic-mass-design.md §2).
func Add(a, b Moments) Moments {
	return combine(a, b, proofbound.IntervalAdd)
}

func Sub(a, b Moments) Moments {
	return combine(a, b, proofbound.IntervalSub)
}

func combine(a, b Moments, op func(proofbound.RatInterval, proofbound.RatInterval) proofbound.RatInterval) Moments {
	out := Moments{Volume: op(a.Volume, b.Volume)}
	for i := range out.First {
		out.First[i] = op(a.First[i], b.First[i])
		for j := range out.Second[i] {
			out.Second[i][j] = op(a.Second[i][j], b.Second[i][j])
		}
	}
	return out
}
