package sweepmitre

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/r3"
)

// Span holds the two recorded ends used by the mitred section construction.
type Span struct {
	Start, End r3.Vec
}

// Construction holds the exact section polygons and the first path point.
type Construction struct {
	Sections [][]sweeparc.RatVec
	Anchor   sweeparc.RatVec
}

// spanError names the refused span, loop and vertex (or segment), so a
// caller can identify the path or section entry that needs repair.
func spanError(sentinel error, span, loop, index int, what string) error {
	return fmt.Errorf(`%w: mitred sweep span %d, loop %d, %s`, sentinel, span, loop, fmt.Sprintf(what, index))
}

// Construct lifts the recorded profile, states each join plane, maps every
// section through its span's wall lines, and applies SM5–SM7 in path order.
// Each wall line passes through its span's apex, or is parallel to the span
// when its ratio is one, so every wall quad is exactly planar. SM6's linear
// inequalities hold on the section hull when they hold at every vertex;
// the resulting projective map preserves the profile and cap triangulation.
func Construct(ctx context.Context, plane sectionrecord.PlaneRecord, pts2 []sectionrecord.Point2,
	loopIdx [][]int, spans []Span, factors []float64) (Construction, error) {
	n := len(spans)
	if len(factors) != n {
		return Construction{}, fmt.Errorf(`%w: the mitred sweep records %d factors for %d spans`,
			decaderr.ErrDegenerate, len(factors), n)
	}
	points := make([]sweeparc.RatVec, n+1)
	dirs := make([]sweeparc.RatVec, n)
	lambdas := make([]*big.Rat, n)
	points[0] = sweeparc.VecOf(spans[0].Start)
	for k, span := range spans {
		points[k+1] = sweeparc.VecOf(span.End)
		dirs[k] = sweeparc.Sub(points[k+1], points[k])
		lambda, err := SpanLengthLower(span.Start, span.End)
		if err != nil {
			return Construction{}, fmt.Errorf(`mitred sweep span %d: %w`, k, err)
		}
		lambdas[k] = lambda
	}

	// Π_0 uses the profile plane's normal, each join plane uses the
	// length-weighted directions, and Π_N uses the last span's direction.
	normals := make([]sweeparc.RatVec, n+1)
	normals[0] = sweeparc.FromDyadic(proofarith.DvCross(proofarith.DyVec(plane.U), proofarith.DyVec(plane.V)))
	normals[n] = dirs[n-1]

	section := make([]sweeparc.RatVec, len(pts2))
	for v, p := range pts2 {
		section[v] = Lift(plane, p)
	}
	sections := [][]sweeparc.RatVec{section}

	one := big.NewRat(1, 1)
	for k := range n {
		if err := ctx.Err(); err != nil {
			return Construction{}, err
		}
		if k+1 < n {
			// SM5: exactly reversed consecutive spans have no join plane.
			if sweeparc.IsZero(sweeparc.Cross(dirs[k], dirs[k+1])) &&
				sweeparc.Dot(dirs[k], dirs[k+1]).Sign() < 0 {
				return Construction{}, fmt.Errorf(`%w: mitred sweep spans %d and %d run exactly back along each other and have no join plane`,
					decaderr.ErrDegenerate, k, k+1)
			}
			normals[k+1] = sweeparc.Add(sweeparc.Scale(dirs[k], lambdas[k+1]), sweeparc.Scale(dirs[k+1], lambdas[k]))
		}
		// SM6 first checks that the span leaves and reaches its section
		// planes forward. These signs constrain every wall line and apex.
		if sweeparc.Dot(normals[k], dirs[k]).Sign() <= 0 {
			return Construction{}, fmt.Errorf(`%w: mitred sweep span %d does not leave its start section plane forward`,
				decaderr.ErrDegenerate, k)
		}
		if sweeparc.Dot(normals[k+1], dirs[k]).Sign() <= 0 {
			return Construction{}, fmt.Errorf(`%w: mitred sweep span %d does not reach its end section plane forward`,
				decaderr.ErrDegenerate, k)
		}

		ratio := new(big.Rat).Quo(Factor(factors, k+1), Factor(factors, k))
		ratioLess := ratio.Cmp(one) < 0
		shrink := new(big.Rat).Sub(one, ratio)
		grow := new(big.Rat).Neg(shrink)
		end := normals[k+1]
		next := make([]sweeparc.RatVec, len(section))
		for i, idx := range loopIdx {
			for j, v := range idx {
				p := section[v]
				w := sweeparc.Add(dirs[k], sweeparc.Scale(sweeparc.Sub(p, points[k]), grow))
				nw := sweeparc.Dot(end, w)
				if nw.Sign() == 0 {
					return Construction{}, spanError(decaderr.ErrDegenerate, k, i, j,
						"vertex %d: its wall line is parallel to the end section plane")
				}
				s := sweeparc.Dot(end, sweeparc.Sub(points[k+1], p))
				s.Quo(s, nw)
				if s.Sign() <= 0 {
					return Construction{}, spanError(decaderr.ErrDegenerate, k, i, j,
						"vertex %d: its wall line meets the end section plane at or behind its start")
				}
				if ratioLess && new(big.Rat).Mul(s, shrink).Cmp(one) >= 0 {
					return Construction{}, spanError(decaderr.ErrDegenerate, k, i, j,
						"vertex %d: its wall line meets the end section plane at or past the span's apex; lengthen the span, shrink the section or weaken the taper")
				}
				next[v] = sweeparc.Add(p, sweeparc.Scale(w, s))
			}
		}
		if err := RequireWalls(k, loopIdx, section, next); err != nil {
			return Construction{}, err
		}
		section = next
		sections = append(sections, section)
	}
	return Construction{Sections: sections, Anchor: points[0]}, nil
}

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
