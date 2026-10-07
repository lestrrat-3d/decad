package massmoment

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
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

// Rotate carries m through the rigid rotation Q nearest the
// exact rational matrix f (its polar factor), f's column k the image of
// local axis k. With d = OrthonormalityDefect(f) ≥ ‖f − Q‖_F:
//   - each (Q·P)_i lies within d·‖P‖₁ of (f·P)_i, since
//     |((Q − f)P)_i| ≤ ‖Q − f‖₂‖P‖₂ ≤ d·‖P‖₁;
//   - each (Q·Q_m·Qᵀ)_ij lies within 3·d·(2+d)·m of (f·Q_m·fᵀ)_ij, m the
//     largest entry magnitude of Q_m. Here ‖Q_m‖₂ ≤ 3m and the rotation
//     defect bounds ‖f − Q‖₂, so this holds for any 3×3 matrix.
func Rotate(m Moments, f [3][3]*big.Rat) Moments {
	defect := OrthonormalityDefect(f)
	firstNorm := new(big.Rat)
	for _, component := range m.First {
		firstNorm.Add(firstNorm, proofbound.IntervalAbsUpper(component))
	}
	firstWiden := new(big.Rat).Mul(defect, firstNorm)
	out := Moments{Volume: m.Volume}
	for i := range out.First {
		sum := proofbound.PointInterval(new(big.Rat))
		for k := range m.First {
			sum = proofbound.IntervalAdd(sum, proofbound.IntervalScale(m.First[k], f[i][k]))
		}
		out.First[i] = survey2d.IntervalWiden(sum, firstWiden)
	}
	secondWiden := new(big.Rat).Mul(big.NewRat(3, 1), defect)
	secondWiden.Mul(secondWiden, new(big.Rat).Add(big.NewRat(2, 1), defect))
	secondWiden.Mul(secondWiden, TensorMagnitude(m.Second))
	out.Second = RotateTensor(f, m.Second)
	for i := range out.Second {
		for j := range out.Second[i] {
			out.Second[i][j] = survey2d.IntervalWiden(out.Second[i][j], secondWiden)
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
