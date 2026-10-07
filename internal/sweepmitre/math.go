// Package sweepmitre computes exact mitred sweep sections and measurements.
package sweepmitre

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
)

// ScaleVertices lifts rational vertices relative to an anchor onto the
// least common multiple of every coordinate denominator. It returns that
// denominator and each vertex's integer (p − anchor) multiple.
func ScaleVertices(exact []sweeparc.RatVec, anchor sweeparc.RatVec) (*big.Int, [][3]*big.Int) {
	den := big.NewInt(1)
	var rem, g big.Int
	widen := func(r *big.Rat) {
		d := r.Denom()
		if rem.Rem(den, d).Sign() == 0 {
			return
		}
		g.GCD(nil, nil, den, d)
		den.Mul(den, rem.Quo(d, &g))
	}
	for _, p := range exact {
		for axis := range 3 {
			widen(p[axis])
		}
	}
	for axis := range 3 {
		widen(anchor[axis])
	}
	lift := func(r *big.Rat) *big.Int {
		n := new(big.Int).Quo(den, r.Denom())
		return n.Mul(n, r.Num())
	}
	origin := [3]*big.Int{lift(anchor[0]), lift(anchor[1]), lift(anchor[2])}
	rel := make([][3]*big.Int, len(exact))
	for v, p := range exact {
		for axis := range 3 {
			n := lift(p[axis])
			rel[v][axis] = n.Sub(n, origin[axis])
		}
	}
	return den, rel
}

// VolumeMoments returns six times the signed volume and the signed first
// moments times twenty-four. Every vertex is lifted onto the common integer
// denominator den. Each signed tetrahedron contributes over den cubed to
// volume and over den to the fourth to each moment. Integer sums are divided
// only once, after all triangles. SetFrac's single reduction yields the same
// canonical rational as summing rational terms one by one.
func VolumeMoments(exact []sweeparc.RatVec, tris [][3]int, anchor sweeparc.RatVec) (*big.Rat, [3]*big.Rat) {
	den, rel := ScaleVertices(exact, anchor)
	var vol6N, term, sum, tmp big.Int
	var momN, cross [3]big.Int
	for _, t := range tris {
		a, b, c := rel[t[0]], rel[t[1]], rel[t[2]]
		for i := range 3 {
			j, k := (i+1)%3, (i+2)%3
			cross[i].Mul(b[j], c[k])
			cross[i].Sub(&cross[i], tmp.Mul(b[k], c[j]))
		}
		term.Mul(a[0], &cross[0])
		term.Add(&term, tmp.Mul(a[1], &cross[1]))
		term.Add(&term, tmp.Mul(a[2], &cross[2]))
		vol6N.Add(&vol6N, &term)
		for axis := range 3 {
			sum.Add(a[axis], b[axis])
			sum.Add(&sum, c[axis])
			momN[axis].Add(&momN[axis], tmp.Mul(&sum, &term))
		}
	}
	den3 := new(big.Int).Mul(den, den)
	den3.Mul(den3, den)
	den4 := new(big.Int).Mul(den3, den)
	vol6 := new(big.Rat).SetFrac(&vol6N, den3)
	var moments [3]*big.Rat
	for axis := range 3 {
		moments[axis] = new(big.Rat).SetFrac(&momN[axis], den4)
	}
	return vol6, moments
}

// TriangleAreas encloses each exact triangle area with rational endpoints.
func TriangleAreas(ctx context.Context, exact []sweeparc.RatVec, tris [][3]int) ([][2]*big.Rat, error) {
	quarter := big.NewRat(1, 4)
	out := make([][2]*big.Rat, len(tris))
	for k, t := range tris {
		if k%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		n := sweeparc.Cross(sweeparc.Sub(exact[t[1]], exact[t[0]]), sweeparc.Sub(exact[t[2]], exact[t[0]]))
		q := sweeparc.Dot(n, n)
		q.Mul(q, quarter)
		lo, hi := proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q)
		if math.IsInf(hi, 0) {
			return nil, fmt.Errorf(`%w: a mitred sweep triangle's area runs past the representable float64 range`, decaderr.ErrUnsupported)
		}
		out[k] = [2]*big.Rat{proofarith.FloatRat(lo), proofarith.FloatRat(hi)}
	}
	return out, nil
}

// Enclosure publishes the midpoint of [lo, hi] with its larger exact gap.
func Enclosure(lo, hi *big.Rat) (float64, float64) {
	mid := new(big.Rat).Add(lo, hi)
	mid.Quo(mid, big.NewRat(2, 1))
	value, _ := mid.Float64()
	return value, math.Max(proofarith.RationalFloatError(lo, value), proofarith.RationalFloatError(hi, value))
}

// OrientSign reads det[b−a, c−a, d−a] over exact rationals.
func OrientSign(a, b, c, d sweeparc.RatVec) int {
	return sweeparc.Dot(sweeparc.Cross(sweeparc.Sub(b, a), sweeparc.Sub(c, a)), sweeparc.Sub(d, a)).Sign()
}

// Centroid rounds the exact centroid coordinates once and bounds their 3D gap.
func Centroid(anchor sweeparc.RatVec, vol6 *big.Rat, moments [3]*big.Rat) ([3]float64, float64, error) {
	if vol6.Sign() == 0 {
		return [3]float64{}, 0, fmt.Errorf(`%w: a mitred sweep with zero volume has no centroid`, decaderr.ErrDegenerate)
	}
	denom := new(big.Rat).Mul(big.NewRat(4, 1), vol6)
	var value [3]float64
	worst := 0.0
	for axis := range 3 {
		c := new(big.Rat).Quo(moments[axis], denom)
		c.Add(c, anchor[axis])
		value[axis], _ = c.Float64()
		worst = math.Max(worst, proofarith.RationalFloatError(c, value[axis]))
	}
	bound := proofbound.Radius3D(worst)
	return value, bound, nil
}

// Bounds rounds every rational coordinate extreme outward.
func Bounds(exact []sweeparc.RatVec) ([3]float64, [3]float64, float64) {
	var lo, hi [3]*big.Rat
	for _, p := range exact {
		for axis := range 3 {
			if lo[axis] == nil || p[axis].Cmp(lo[axis]) < 0 {
				lo[axis] = p[axis]
			}
			if hi[axis] == nil || p[axis].Cmp(hi[axis]) > 0 {
				hi[axis] = p[axis]
			}
		}
	}
	var minV, maxV [3]float64
	worst := 0.0
	for axis := range 3 {
		minV[axis] = proofbound.RatFloatDown(lo[axis])
		maxV[axis] = proofbound.RatFloatUp(hi[axis])
		worst = math.Max(worst, proofarith.RationalFloatError(lo[axis], minV[axis]))
		worst = math.Max(worst, proofarith.RationalFloatError(hi[axis], maxV[axis]))
	}
	return minV, maxV, worst
}
