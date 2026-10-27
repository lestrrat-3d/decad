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

// twistBernsteinScaled is n³·twistBernstein3(a/n) for 0 <= a <= n: the
// integers C(3,j)·a^j·(n−a)^(3−j).
func twistBernsteinScaled(a, n int64) [4]int64 {
	b := n - a
	return [4]int64{b * b * b, 3 * a * b * b, 3 * a * a * b, a * a * a}
}

// twistRow is the four cubic control points of the curve at one height, as
// integers over the patch's common denominator times the height weights'
// scale.
type twistRow [4][2]big.Int

// blendT collapses the height direction at the scaled Bernstein weights wt
// (twistBernsteinScaled), leaving the four control points of the curve at
// that height over c.d·nt³.
func (c *twistCommon) blendT(wt [4]int64) *twistRow {
	var row twistRow
	var t big.Int
	for j := range c.q {
		for l := range c.q[j] {
			w := big.NewInt(wt[l])
			for k := range row[j] {
				row[j][k].Add(&row[j][k], t.Mul(&c.q[j][l][k], w))
			}
		}
	}
	return &row
}

// twistOrigin is the plane origin's dyadic X and Y as x/2^shift, y/2^shift.
type twistOrigin struct {
	x, y  *big.Int
	shift uint
}

// twistOriginOf takes the least shift that makes both of ox, oy integers;
// each is a float and so has a power-of-two denominator.
func twistOriginOf(ox, oy *big.Rat) twistOrigin {
	sx, sy := ox.Denom().TrailingZeroBits(), oy.Denom().TrailingZeroBits()
	shift := max(sx, sy)
	return twistOrigin{x: new(big.Int).Lsh(ox.Num(), shift-sx), y: new(big.Int).Lsh(oy.Num(), shift-sy),
		shift: shift}
}

// twistCurve is one patch's curve at one height, plus the plane origin, over
// the positive denominator den = d·nu³·nt³·2^shift.
type twistCurve struct {
	row             *twistRow
	den, offX, offY big.Int
	shift           uint
}

// curveAt readies the curve at scaled height weights wt for stations of
// count nu: a station's world X is (row point·2^shift + origin X·d·nu³·nt³)/den.
func (c *twistCommon) curveAt(wt [4]int64, nu, nt int, o twistOrigin) *twistCurve {
	cv := &twistCurve{row: c.blendT(wt), shift: o.shift}
	var scale big.Int
	scale.Mul(c.d, big.NewInt(int64(nu*nu*nu)))
	scale.Mul(&scale, big.NewInt(int64(nt*nt*nt)))
	cv.offX.Mul(o.x, &scale)
	cv.offY.Mul(o.y, &scale)
	cv.den.Lsh(&scale, o.shift)
	return cv
}

// point sets x and y to the world X and Y numerators over cv.den of the
// station at scaled Bernstein weights wu (twistBernsteinScaled of a/nu).
func (cv *twistCurve) point(wu [4]int64, x, y *big.Int) {
	var t big.Int
	x.SetInt64(0)
	y.SetInt64(0)
	for j := range cv.row {
		w := big.NewInt(wu[j])
		x.Add(x, t.Mul(&cv.row[j][0], w))
		y.Add(y, t.Mul(&cv.row[j][1], w))
	}
	x.Lsh(x, cv.shift)
	x.Add(x, &cv.offX)
	y.Lsh(y, cv.shift)
	y.Add(y, &cv.offY)
}

// polarCoefficients gives cross(C, ∂C/∂u) in Bernstein form of degree (5,6).
// Positive coefficients prove a positive polar-angle derivative on the patch.
// It is polarNumerators divided by their positive denominators.
func (p twistPatch) polarCoefficients() [6][7]*big.Rat {
	num, d := p.polarNumerators()
	d2 := new(big.Int).Mul(d, d)
	var coeff [6][7]*big.Rat
	for k := range coeff {
		for l := range coeff[k] {
			den := new(big.Int).Mul(d2, big.NewInt(twistBinomial[6][l]*twistPolarScale[k]))
			coeff[k][l] = new(big.Rat).SetFrac(new(big.Int).Mul(num[k][l], big.NewInt(3)), den)
		}
	}
	return coeff
}

// twistPolarScale is the u-row divisor of polarNumerators.
var twistPolarScale = [6]int64{1, 5, 10, 10, 5, 1}

// polarNumerators gives polarCoefficients as integers: coefficient (k,l) is
// 3·num[k][l] / (d²·C(6,l)·twistPolarScale[k]), with d > 0 the denominator of
// overCommon. Every denominator is positive, so a numerator has its
// coefficient's sign. The sums run on integers and no GCD reduces them.
func (p twistPatch) polarNumerators() ([6][7]*big.Int, *big.Int) {
	// For the four cubic control curves P_i(t), let C_ij = cross(P_i, P_j).
	// Antisymmetry combines cross(C, C_u)'s twelve derivative products into
	// six products. Its six u coefficients are
	// [3C01, (3C01+6C02)/5, (3C03+6C02+9C12)/10,
	//  (3C03+6C13+9C12)/10, (6C13+3C23)/5, 3C23].
	// Each C_ij is a degree-six Bernstein polynomial in t whose coefficient l
	// is Σ_{j+n=l} C(3,j)·C(3,n)·cross(P_ij, P_in)/C(6,l). With every
	// coordinate x/d, d²·C(6,l)·C_ij[l] is the integer s[pair][l] below.
	c := p.overCommon()
	q, d := &c.q, c.d
	pairs := [6][2]int{{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}}
	var s [6][7]big.Int
	var cross, t big.Int
	for k, pair := range pairs {
		a, b := &q[pair[0]], &q[pair[1]]
		for j := range a {
			for n := range b {
				cross.Mul(&a[j][0], &b[n][1])
				t.Mul(&a[j][1], &b[n][0])
				cross.Sub(&cross, &t)
				if w := twistBinomial[3][j] * twistBinomial[3][n]; w != 1 {
					cross.Mul(&cross, big.NewInt(w))
				}
				s[k][j+n].Add(&s[k][j+n], &cross)
			}
		}
	}
	// Divided by three, the u coefficients are [C01, C01+2C02,
	// C03+2C02+3C12, C03+2C13+3C12, 2C13+C23, C23] over twistPolarScale.
	var num [6][7]*big.Int
	two, three := big.NewInt(2), big.NewInt(3)
	for l := range num[0] {
		b := new(big.Int).Mul(two, &s[1][l])
		c := new(big.Int).Mul(three, &s[3][l])
		c.Add(c, &s[2][l])
		e := new(big.Int).Mul(two, &s[4][l])
		num[0][l] = new(big.Int).Set(&s[0][l])
		num[1][l] = new(big.Int).Add(&s[0][l], b)
		num[2][l] = b.Add(b, c)
		num[3][l] = c.Add(c, e)
		num[4][l] = e.Add(e, &s[5][l])
		num[5][l] = new(big.Int).Set(&s[5][l])
	}
	return num, d
}

// twistCommon is a patch's coordinates as integers over one positive
// denominator d: q[i][j][0]/d is p[i][j].U and q[i][j][1]/d is p[i][j].V.
// It is only read once built.
type twistCommon struct {
	q [4][4][2]big.Int
	d *big.Int
}

// overCommon builds the patch's twistCommon over the least common multiple
// of its coordinates' denominators. The power of two of d is the largest of
// the denominators', and its odd part is the least common multiple of
// theirs, so only odd parts take a GCD.
func (p twistPatch) overCommon() *twistCommon {
	out := new(twistCommon)
	q := &out.q
	var odds [4][4][2]big.Int
	var shifts [4][4][2]uint
	var top uint
	odd := big.NewInt(1)
	var g, r big.Int
	for i := range p {
		for j := range p[i] {
			for c, x := range [2]*big.Rat{p[i][j].U, p[i][j].V} {
				tz := x.Denom().TrailingZeroBits()
				shifts[i][j][c] = tz
				top = max(top, tz)
				o := odds[i][j][c].Rsh(x.Denom(), tz)
				if o.IsInt64() && o.Int64() == 1 {
					continue
				}
				if r.Rem(odd, o); r.Sign() == 0 {
					continue
				}
				g.GCD(nil, nil, odd, o)
				odd.Mul(odd, r.Quo(o, &g))
			}
		}
	}
	for i := range p {
		for j := range p[i] {
			for c, x := range [2]*big.Rat{p[i][j].U, p[i][j].V} {
				v := q[i][j][c].Lsh(x.Num(), top-shifts[i][j][c])
				switch o := &odds[i][j][c]; {
				case o.Cmp(odd) == 0:
				case o.IsInt64() && o.Int64() == 1:
					v.Mul(v, odd)
				default:
					v.Mul(v, r.Quo(odd, o))
				}
			}
		}
	}
	out.d = odd.Lsh(odd, top)
	return out
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
		// polarNumerators share their coefficients' signs.
		coeff, _ := cell.polarNumerators()
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

func (p twistPatch) bounds() twistBounds { return p.overCommon().bounds() }

// bounds takes every maximum over the integer coordinates: the common
// denominator d is positive, so the largest numerator is the largest value,
// and each bound is that numerator times its factor over d.
func (c *twistCommon) bounds() twistBounds {
	q, d := &c.q, c.d
	var radius, bu, bt, muu, mtt, mut, l1, x, y big.Int
	// keep sets m to the L1 norm of (x, y) when that is larger.
	keep := func(m *big.Int) {
		l1.Abs(&x)
		y.Abs(&y)
		l1.Add(&l1, &y)
		if l1.Cmp(m) > 0 {
			m.Set(&l1)
		}
	}
	// second sets (x, y) to a - 2b + c.
	second := func(a, b, c *[2]big.Int) {
		x.Lsh(&b[0], 1)
		x.Sub(&a[0], &x)
		x.Add(&x, &c[0])
		y.Lsh(&b[1], 1)
		y.Sub(&a[1], &y)
		y.Add(&y, &c[1])
	}
	for i := range q {
		for j := range q[i] {
			x.Set(&q[i][j][0])
			y.Set(&q[i][j][1])
			keep(&radius)
			if i < 3 {
				x.Sub(&q[i+1][j][0], &q[i][j][0])
				y.Sub(&q[i+1][j][1], &q[i][j][1])
				keep(&bu)
			}
			if j < 3 {
				x.Sub(&q[i][j+1][0], &q[i][j][0])
				y.Sub(&q[i][j+1][1], &q[i][j][1])
				keep(&bt)
			}
			if i < 2 {
				second(&q[i][j], &q[i+1][j], &q[i+2][j])
				keep(&muu)
			}
			if j < 2 {
				second(&q[i][j], &q[i][j+1], &q[i][j+2])
				keep(&mtt)
			}
			if i < 3 && j < 3 {
				for c, v := range [2]*big.Int{&x, &y} {
					v.Sub(&q[i+1][j+1][c], &q[i+1][j][c])
					v.Sub(v, &q[i][j+1][c])
					v.Add(v, &q[i][j][c])
				}
				keep(&mut)
			}
		}
	}
	frac := func(m *big.Int, factor int64) *big.Rat {
		return new(big.Rat).SetFrac(m.Mul(m, big.NewInt(factor)), d)
	}
	return twistBounds{bu: frac(&bu, 3), bt: frac(&bt, 3), muu: frac(&muu, 6), mtt: frac(&mtt, 6),
		mut: frac(&mut, 9), radius: frac(&radius, 1)}
}
