package smoothloft

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// twistPatch is the bicubic plane-coordinate part of one wall patch of the
// twisted fitted loft (agent-docs/loft-sections-design.md §13.1). The first
// index is the curve parameter u, the second the height parameter t. Every
// control coordinate is an exact rational.
type twistPatch [4][4]freeform.RatPoint

// twistSpan holds the control points of one rational cubic Bézier span of one
// edge of one section.
type twistSpan [4]freeform.RatPoint

func twistSub(a, b freeform.RatPoint) freeform.RatPoint {
	return freeform.RatPoint{U: sub(a.U, b.U), V: sub(a.V, b.V)}
}

func twistAdd(a, b freeform.RatPoint) freeform.RatPoint {
	return freeform.RatPoint{U: add(a.U, b.U), V: add(a.V, b.V)}
}

func twistScale(a freeform.RatPoint, q *big.Rat) freeform.RatPoint {
	return freeform.RatPoint{U: mul(a.U, q), V: mul(a.V, q)}
}

func twistL1(a freeform.RatPoint) *big.Rat { return add(abs(a.U), abs(a.V)) }

// twistLineSpan is the degree-elevated cubic of a straight record.
func twistLineSpan(a, b freeform.RatPoint) twistSpan {
	third := big.NewRat(1, 3)
	return twistSpan{a,
		twistAdd(a, twistScale(twistSub(b, a), third)),
		twistAdd(a, twistScale(twistSub(b, a), big.NewRat(2, 3))), b}
}

// twistSlopes gives §13.1's tangents of one control point per unit of Z:
// the central difference quotient across neighbouring levels, one-sided at
// the ends. On equal spacing it is the Catmull-Rom tangent.
func twistSlopes(points []freeform.RatPoint, zs []*big.Rat) []freeform.RatPoint {
	n := len(points)
	out := make([]freeform.RatPoint, n)
	quotient := func(a, b int) freeform.RatPoint {
		return twistScale(twistSub(points[b], points[a]), div(rint(1), sub(zs[b], zs[a])))
	}
	out[0], out[n-1] = quotient(0, 1), quotient(n-2, n-1)
	for i := 1; i < n-1; i++ {
		out[i] = quotient(i-1, i+1)
	}
	return out
}

// twistPatchOf blends two neighbouring sections' span controls with their
// per-Z slopes into the cubic Hermite patch of one height span of height h.
func twistPatchOf(c0, c1, m0, m1 twistSpan, h *big.Rat) twistPatch {
	third := div(h, rint(3))
	var p twistPatch
	for j := range p {
		p[j][0] = c0[j]
		p[j][1] = twistAdd(c0[j], twistScale(m0[j], third))
		p[j][2] = twistSub(c1[j], twistScale(m1[j], third))
		p[j][3] = c1[j]
	}
	return p
}

var twistBinomial = [7][]int64{
	{1}, {1, 1}, {1, 2, 1}, {1, 3, 3, 1}, {1, 4, 6, 4, 1}, {1, 5, 10, 10, 5, 1}, {1, 6, 15, 20, 15, 6, 1},
}

// blendT collapses the height direction at fixed Bernstein weights, leaving
// the four cubic control points of the curve at that height.
func (p twistPatch) blendT(bt [4]*big.Rat) [4]freeform.RatPoint {
	var out [4]freeform.RatPoint
	for j := range p {
		out[j] = freeform.RatPoint{U: new(big.Rat), V: new(big.Rat)}
		for l := range p[j] {
			out[j] = twistAdd(out[j], twistScale(p[j][l], bt[l]))
		}
	}
	return out
}

func blendRow(row [4]freeform.RatPoint, bu [4]*big.Rat) freeform.RatPoint {
	out := freeform.RatPoint{U: new(big.Rat), V: new(big.Rat)}
	for j := range row {
		out = twistAdd(out, twistScale(row[j], bu[j]))
	}
	return out
}

// At evaluates the patch at exact rational (u, t).
func (p twistPatch) At(u, t *big.Rat) freeform.RatPoint {
	bu, bt := twistBernstein3(u), twistBernstein3(t)
	out := freeform.RatPoint{U: new(big.Rat), V: new(big.Rat)}
	for j := range p {
		for l := range p[j] {
			out = twistAdd(out, twistScale(p[j][l], mul(bu[j], bt[l])))
		}
	}
	return out
}

func twistBernstein3(x *big.Rat) [4]*big.Rat {
	y := sub(rint(1), x)
	xx, yy := mul(x, x), mul(y, y)
	return [4]*big.Rat{mul(yy, y), mul(rint(3), mul(x, yy)), mul(rint(3), mul(xx, y)), mul(xx, x)}
}

// polarCoefficients gives cross(C, ∂C/∂u) in Bernstein form of degree (5,6).
// Positive coefficients prove a positive polar-angle derivative on the patch.
func (p twistPatch) polarCoefficients() [6][7]*big.Rat {
	// For the four cubic control curves P_i(t), let C_ij = cross(P_i, P_j).
	// Antisymmetry combines cross(C, C_u)'s twelve derivative products into
	// six products. Its six u coefficients are
	// [3C01, (3C01+6C02)/5, (3C03+6C02+9C12)/10,
	//  (3C03+6C13+9C12)/10, (6C13+3C23)/5, 3C23].
	// Each C_ij is a degree-six Bernstein polynomial in t.
	pairs := [6][2]int{{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}}
	var products [6][7]*big.Rat
	for k, pair := range pairs {
		for l := range products[k] {
			products[k][l] = new(big.Rat)
		}
		a, b := p[pair[0]], p[pair[1]]
		for j := range a {
			for n := range b {
				wt := big.NewRat(twistBinomial[3][j]*twistBinomial[3][n], twistBinomial[6][j+n])
				cross := sub(mul(a[j].U, b[n].V), mul(a[j].V, b[n].U))
				products[k][j+n].Add(products[k][j+n], mul(cross, wt))
			}
		}
	}
	three, six, nine := rint(3), rint(6), rint(9)
	fifth, tenth := big.NewRat(1, 5), big.NewRat(1, 10)
	var coeff [6][7]*big.Rat
	for l := range coeff[0] {
		a := mul(three, products[0][l])
		b := mul(six, products[1][l])
		c := add(mul(three, products[2][l]), mul(nine, products[3][l]))
		d := mul(six, products[4][l])
		e := mul(three, products[5][l])
		coeff[0][l] = a
		coeff[1][l] = mul(add(a, b), fifth)
		coeff[2][l] = mul(add(b, c), tenth)
		coeff[3][l] = mul(add(c, d), tenth)
		coeff[4][l] = mul(add(d, e), fifth)
		coeff[5][l] = e
	}
	return coeff
}

func (p twistPatch) split(alongU bool) (twistPatch, twistPatch) {
	var left, right twistPatch
	half := big.NewRat(1, 2)
	for fixed := range 4 {
		us, vs := make([]*big.Rat, 4), make([]*big.Rat, 4)
		for i := range 4 {
			q := p[i][fixed]
			if !alongU {
				q = p[fixed][i]
			}
			us[i], vs[i] = q.U, q.V
		}
		lu, ru := freeform.BernsteinSplit(us, half)
		lv, rv := freeform.BernsteinSplit(vs, half)
		for i := range 4 {
			l := freeform.RatPoint{U: lu[i], V: lv[i]}
			r := freeform.RatPoint{U: ru[i], V: rv[i]}
			if alongU {
				left[i][fixed], right[i][fixed] = l, r
			} else {
				left[fixed][i], right[fixed][i] = l, r
			}
		}
	}
	return left, right
}

// provePolarPositive proves cross(C, C_u) > 0 on the whole patch by exact
// Bernstein subdivision. A cell whose corner value is not positive refuses at
// once, since a corner coefficient is the function's value there.
func (p twistPatch) provePolarPositive(ctx context.Context, budget *proofbound.WorkBudget) error {
	const maxDepth = 12
	var check func(twistPatch, int) error
	check = func(cell twistPatch, depth int) error {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := budget.Step(); err != nil {
			return err
		}
		coeff := cell.polarCoefficients()
		for _, corner := range [...][2]int{{0, 0}, {5, 0}, {0, 6}, {5, 6}} {
			if coeff[corner[0]][corner[1]].Sign() <= 0 {
				return fmt.Errorf("%w: twisted loft section has no increasing polar angle", decaderr.ErrUnsupported)
			}
		}
		positive := true
		for _, row := range coeff {
			for _, c := range row {
				if c.Sign() <= 0 {
					positive = false
				}
			}
		}
		if positive {
			return nil
		}
		if depth == maxDepth {
			return fmt.Errorf("%w: twisted loft has no positive polar-angle proof", decaderr.ErrUnsupported)
		}
		left, right := cell.split(depth%2 == 0)
		if err := check(left, depth+1); err != nil {
			return err
		}
		return check(right, depth+1)
	}
	return check(p, 0)
}

// twistBounds are §13.3's derivative bounds of a patch, in the L1 norm.
type twistBounds struct {
	bu, bt, muu, mtt, mut *big.Rat
	radius                *big.Rat
}

func (p twistPatch) bounds() twistBounds {
	b := twistBounds{bu: new(big.Rat), bt: new(big.Rat), muu: new(big.Rat), mtt: new(big.Rat),
		mut: new(big.Rat), radius: new(big.Rat)}
	for i := range p {
		for j := range p[i] {
			b.radius = maxRat(b.radius, twistL1(p[i][j]))
			if i < 3 {
				b.bu = maxRat(b.bu, mul(rint(3), twistL1(twistSub(p[i+1][j], p[i][j]))))
			}
			if j < 3 {
				b.bt = maxRat(b.bt, mul(rint(3), twistL1(twistSub(p[i][j+1], p[i][j]))))
			}
			if i < 2 {
				second := twistAdd(twistSub(p[i+2][j], twistScale(p[i+1][j], rint(2))), p[i][j])
				b.muu = maxRat(b.muu, mul(rint(6), twistL1(second)))
			}
			if j < 2 {
				second := twistAdd(twistSub(p[i][j+2], twistScale(p[i][j+1], rint(2))), p[i][j])
				b.mtt = maxRat(b.mtt, mul(rint(6), twistL1(second)))
			}
			if i < 3 && j < 3 {
				mixed := twistSub(twistSub(p[i+1][j+1], p[i+1][j]), twistSub(p[i][j+1], p[i][j]))
				b.mut = maxRat(b.mut, mul(rint(9), twistL1(mixed)))
			}
		}
	}
	return b
}
