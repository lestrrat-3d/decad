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
	"github.com/lestrrat-3d/r3"
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

// PlacedVolumeMoments applies a transform's exact float entries to a
// tetrahedron sum about its anchor. Each determinant term scales by det(M),
// and each relative first moment also maps through M. Fresh results keep the
// source sums unchanged when orientation is corrected after placement.
func PlacedVolumeMoments(vol6 *big.Rat, moments [3]*big.Rat, xform r3.Transform) (*big.Rat, [3]*big.Rat) {
	if xform == r3.Identity() {
		return new(big.Rat).Set(vol6), [3]*big.Rat{
			new(big.Rat).Set(moments[0]),
			new(big.Rat).Set(moments[1]),
			new(big.Rat).Set(moments[2]),
		}
	}
	basis := xform.Basis()
	ex, ey, ez := sweeparc.VecOf(basis.EX), sweeparc.VecOf(basis.EY), sweeparc.VecOf(basis.EZ)
	vectors := [3]sweeparc.RatVec{ex, ey, ez}
	// Every basis entry comes from a float, so its reduced denominator is a
	// power of two. The largest one is a common denominator for the matrix.
	basisDen := big.NewInt(1)
	for _, vector := range vectors {
		for _, value := range vector {
			if value.Denom().Cmp(basisDen) > 0 {
				basisDen.Set(value.Denom())
			}
		}
	}
	var matrix [3][3]*big.Int // columns of the basis over basisDen
	for col, vector := range vectors {
		for row, value := range vector {
			matrix[col][row] = proofarith.ScaledNum(value, basisDen)
		}
	}
	var cross [3]big.Int
	var tmp, detN big.Int
	for row := range 3 {
		j, k := (row+1)%3, (row+2)%3
		cross[row].Mul(matrix[1][j], matrix[2][k])
		cross[row].Sub(&cross[row], tmp.Mul(matrix[1][k], matrix[2][j]))
		detN.Add(&detN, tmp.Mul(matrix[0][row], &cross[row]))
	}
	basisDen3 := new(big.Int).Mul(basisDen, basisDen)
	basisDen3.Mul(basisDen3, basisDen)
	volume := new(big.Rat).SetFrac(
		new(big.Int).Mul(vol6.Num(), &detN),
		new(big.Int).Mul(vol6.Denom(), basisDen3),
	)
	momentDen := proofarith.CommonDenom(moments[:]...)
	var momentN [3]*big.Int
	for axis, moment := range moments {
		momentN[axis] = proofarith.ScaledNum(moment, momentDen)
	}
	placedDen := new(big.Int).Mul(basisDen3, basisDen)
	placedDen.Mul(placedDen, momentDen)
	var placed [3]*big.Rat
	for row := range 3 {
		var numerator big.Int
		for col := range 3 {
			numerator.Add(&numerator, tmp.Mul(matrix[col][row], momentN[col]))
		}
		numerator.Mul(&numerator, &detN)
		placed[row] = new(big.Rat).SetFrac(&numerator, placedDen)
	}
	return volume, placed
}

// TriangleAreas encloses each exact triangle area with rational endpoints.
func TriangleAreas(ctx context.Context, exact []sweeparc.RatVec, tris [][3]int) ([][2]*big.Rat, error) {
	out := make([][2]*big.Rat, len(tris))
	for k, t := range tris {
		if k%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		// Scale only this triangle. A sweep-wide denominator makes the
		// integers grow with unrelated vertices and slows their products.
		den := big.NewInt(1)
		var rem, gcd big.Int
		for _, vertex := range t {
			for axis := range 3 {
				d := exact[vertex][axis].Denom()
				if rem.Rem(den, d).Sign() == 0 {
					continue
				}
				gcd.GCD(nil, nil, den, d)
				den.Mul(den, rem.Quo(d, &gcd))
			}
		}
		var edge [2][3]big.Int
		for axis := range 3 {
			origin := new(big.Int).Mul(new(big.Int).Quo(den, exact[t[0]][axis].Denom()), exact[t[0]][axis].Num())
			for e := range 2 {
				r := exact[t[e+1]][axis]
				edge[e][axis].Mul(new(big.Int).Quo(den, r.Denom()), r.Num())
				edge[e][axis].Sub(&edge[e][axis], origin)
			}
		}
		var square, component, scratch big.Int
		for axis := range 3 {
			j, l := (axis+1)%3, (axis+2)%3
			component.Mul(&edge[0][j], &edge[1][l])
			component.Sub(&component, scratch.Mul(&edge[0][l], &edge[1][j]))
			square.Add(&square, scratch.Mul(&component, &component))
		}
		den4 := new(big.Int).Mul(den, den)
		den4.Mul(den4, den4)
		den4.Lsh(den4, 2)
		q := new(big.Rat).SetFrac(&square, den4)
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
	fourVol := new(big.Int).Lsh(vol6.Num(), 2)
	var value [3]float64
	worst := 0.0
	for axis := range 3 {
		moment, origin := moments[axis], anchor[axis]
		var num, den, tmp big.Int
		num.Mul(origin.Num(), fourVol)
		num.Mul(&num, moment.Denom())
		tmp.Mul(moment.Num(), vol6.Denom())
		tmp.Mul(&tmp, origin.Denom())
		num.Add(&num, &tmp)
		den.Mul(fourVol, moment.Denom())
		den.Mul(&den, origin.Denom())
		c := new(big.Rat).SetFrac(&num, &den)
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
