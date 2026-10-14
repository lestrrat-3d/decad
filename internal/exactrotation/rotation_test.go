package exactrotation_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type rotCase struct {
	name         string
	center, axis r3.Vec
	theta        float64
}

func rotCases() []rotCase {
	axes := []struct {
		name string
		axis r3.Vec
	}{
		{"x", r3.Vec{X: 1}},
		{"122", r3.Vec{X: 1, Y: 2, Z: 2}},
		{"maxmax0", r3.Vec{X: math.MaxFloat64, Y: math.MaxFloat64}},
		{"tiny", r3.Vec{X: 0x1p-1074}},
		{"maxtiny", r3.Vec{X: math.MaxFloat64, Y: 0x1p-1074}},
	}
	thetas := []float64{math.Pi / 2, math.Pi, -2 * math.Pi / 3, 2 * math.Pi * 23 / 31, 0}
	centers := []r3.Vec{{}, {X: 3, Y: -4, Z: 5.5}}
	var out []rotCase
	for _, a := range axes {
		for _, th := range thetas {
			for _, c := range centers {
				out = append(out, rotCase{name: a.name, center: c, axis: a.axis, theta: th})
			}
		}
	}
	return out
}

func ratOf(f float64) *big.Rat { return proof.FloatRat(f) }

func ratVec(v r3.Vec) [3]*big.Rat { return [3]*big.Rat{ratOf(v.X), ratOf(v.Y), ratOf(v.Z)} }

func mulVec(m [3][3]*big.Rat, v [3]*big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for i := range out {
		out[i] = new(big.Rat)
		for j := range v {
			out[i].Add(out[i], new(big.Rat).Mul(m[i][j], v[j]))
		}
	}
	return out
}

func det(m [3][3]*big.Rat) *big.Rat {
	mul := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
	minor := func(i, j, k, l int) *big.Rat {
		return new(big.Rat).Sub(mul(m[i][k], m[j][l]), mul(m[i][l], m[j][k]))
	}
	d := mul(m[0][0], minor(1, 2, 1, 2))
	d.Sub(d, mul(m[0][1], minor(1, 2, 0, 2)))
	return d.Add(d, mul(m[0][2], minor(1, 2, 0, 1)))
}

func floatOf(q *big.Rat) float64 {
	f, _ := q.Float64()
	return f
}

func build(t *testing.T, c rotCase) (*exactrotation.Rotation, r3.Transform) {
	t.Helper()
	rec, d, err := exactrotation.New(c.center, c.axis, units.Radians(c.theta))
	require.NoError(t, err, "%s theta=%v center=%v", c.name, c.theta, c.center)
	return rec, d
}

func TestNewExactOrthogonality(t *testing.T) {
	for _, c := range rotCases() {
		rec, _ := build(t, c)
		m := rec.Linear()
		// Q^T Q = I exactly.
		for i := range 3 {
			for j := range 3 {
				dot := new(big.Rat)
				for k := range 3 {
					dot.Add(dot, new(big.Rat).Mul(m[k][i], m[k][j]))
				}
				want := new(big.Rat)
				if i == j {
					want.SetInt64(1)
				}
				require.Zero(t, dot.Cmp(want), "%s theta=%v: Q^T Q [%d][%d] = %s", c.name, c.theta, i, j, dot)
			}
		}
		require.Zero(t, det(m).Cmp(big.NewRat(1, 1)), "%s theta=%v: determinant", c.name, c.theta)
	}
}

func TestNewExactAxisFixationAndTranslation(t *testing.T) {
	for _, c := range rotCases() {
		rec, _ := build(t, c)
		m := rec.Linear()
		a := ratVec(c.axis)
		got := mulVec(m, a)
		for i := range a {
			require.Zero(t, got[i].Cmp(a[i]), "%s theta=%v: Q.a component %d", c.name, c.theta, i)
		}
		// t = c - Q c exactly.
		cc := ratVec(c.center)
		qc := mulVec(m, cc)
		tr := rec.Translation()
		for i := range cc {
			want := new(big.Rat).Sub(cc[i], qc[i])
			require.Zero(t, tr[i].Cmp(want), "%s theta=%v: translation %d", c.name, c.theta, i)
		}
		// The center is a fixed point of the affine map.
		img := mulVec(m, cc)
		for i := range img {
			img[i].Add(img[i], tr[i])
			require.Zero(t, img[i].Cmp(cc[i]), "%s theta=%v: center component %d", c.name, c.theta, i)
		}
	}
}

// unitPerpendicular returns the unit direction of axis, a unit vector
// perpendicular to it, and their cross product, all computed after scaling the
// axis by its largest component.
func unitPerpendicular(axis r3.Vec) (n, p, q r3.Vec) {
	scale := math.Max(math.Abs(axis.X), math.Max(math.Abs(axis.Y), math.Abs(axis.Z)))
	n = r3.Vec{X: axis.X / scale, Y: axis.Y / scale, Z: axis.Z / scale}
	n = n.Scale(1 / math.Sqrt(n.Dot(n)))
	e := r3.Vec{X: 1}
	switch {
	case math.Abs(n.Y) <= math.Abs(n.X) && math.Abs(n.Y) <= math.Abs(n.Z):
		e = r3.Vec{Y: 1}
	case math.Abs(n.Z) <= math.Abs(n.X) && math.Abs(n.Z) <= math.Abs(n.Y):
		e = r3.Vec{Z: 1}
	}
	p = n.Cross(e)
	p = p.Scale(1 / math.Sqrt(p.Dot(p)))
	return n, p, n.Cross(p)
}

func TestNewAngleAgreement(t *testing.T) {
	for _, c := range rotCases() {
		rec, _ := build(t, c)
		m := rec.Linear()
		trace := new(big.Rat).Add(m[0][0], new(big.Rat).Add(m[1][1], m[2][2]))
		cosQ := floatOf(new(big.Rat).Quo(trace.Sub(trace, big.NewRat(1, 1)), big.NewRat(2, 1)))
		require.InDelta(t, math.Cos(c.theta), cosQ, 1e-12,
			"%s theta=%v: trace-derived cosine", c.name, c.theta)

		_, p, q := unitPerpendicular(c.axis)
		qp := mulVec(m, ratVec(p))
		turn := floatOf(qp[0])*q.X + floatOf(qp[1])*q.Y + floatOf(qp[2])*q.Z
		require.InDelta(t, math.Sin(c.theta), turn, 1e-12, "%s theta=%v: turn component", c.name, c.theta)
		if math.Abs(math.Sin(c.theta)) > 1e-6 {
			require.Equal(t, math.Signbit(math.Sin(c.theta)), math.Signbit(turn),
				"%s theta=%v: turn sign", c.name, c.theta)
		}
	}
}

func TestNewExtremeAxes(t *testing.T) {
	t.Run("MaxFloat64 pair at a quarter turn is a quarter turn", func(t *testing.T) {
		rec, d, err := exactrotation.New(r3.Vec{}, r3.Vec{X: math.MaxFloat64, Y: math.MaxFloat64}, units.Radians(math.Pi/2))
		require.NoError(t, err)
		m := rec.Linear()
		trace := new(big.Rat).Add(m[0][0], new(big.Rat).Add(m[1][1], m[2][2]))
		// A quarter turn has trace 1 + 2 cos(pi/2) = 1, the identity 3.
		require.InDelta(t, 1.0, floatOf(trace), 1e-12)
		// The axis (1,1,0)/sqrt2 sends z to (1,-1,0)/sqrt2 up to rounding.
		require.InDelta(t, 0.5, floatOf(m[0][0]), 1e-12)
		require.InDelta(t, 0.5, floatOf(m[0][1]), 1e-12)
		require.InDelta(t, math.Sqrt2/2, floatOf(m[0][2]), 1e-12)
		linear, _, ok := rec.Gap(d)
		require.True(t, ok)
		require.Less(t, linear, 1e-12)
	})
	t.Run("tiny axis is the X axis", func(t *testing.T) {
		rec, _, err := exactrotation.New(r3.Vec{}, r3.Vec{X: 0x1p-1074}, units.Radians(math.Pi/2))
		require.NoError(t, err)
		m := rec.Linear()
		require.Zero(t, m[0][0].Cmp(big.NewRat(1, 1)))
		require.InDelta(t, 0.0, floatOf(m[1][1]), 1e-12)
		require.InDelta(t, -1.0, floatOf(m[1][2]), 1e-12)
		require.InDelta(t, 1.0, floatOf(m[2][1]), 1e-12)
		require.True(t, rec.FixesXAxisAtOrigin())
	})
	t.Run("a huge axis with a tiny component keeps its direction", func(t *testing.T) {
		rec, _, err := exactrotation.New(r3.Vec{}, r3.Vec{X: math.MaxFloat64, Y: 0x1p-1074}, units.Radians(math.Pi/2))
		require.NoError(t, err)
		m := rec.Linear()
		require.InDelta(t, 1.0, floatOf(m[0][0]), 1e-12)
		require.InDelta(t, -1.0, floatOf(m[1][2]), 1e-12)
	})
}

func TestNewErrorKinds(t *testing.T) {
	inf := math.Inf(1)
	cases := []struct {
		name         string
		center, axis r3.Vec
		angle        units.Value
		want         error
	}{
		{"zero axis", r3.Vec{}, r3.Vec{}, units.Radians(1), decaderr.ErrDegenerate},
		{"non-finite axis", r3.Vec{}, r3.Vec{X: inf}, units.Radians(1), decaderr.ErrDegenerate},
		{"non-finite center", r3.Vec{Y: math.NaN()}, r3.Vec{X: 1}, units.Radians(1), decaderr.ErrNotFinite},
		{"non-finite angle", r3.Vec{}, r3.Vec{X: 1}, units.Radians(inf), decaderr.ErrNotFinite},
		{"length is not an angle", r3.Vec{}, r3.Vec{X: 1}, units.Millimeters(1), decaderr.ErrUnitKind},
		{"translation overflow", r3.Vec{Y: math.MaxFloat64, Z: math.MaxFloat64}, r3.Vec{X: 1},
			units.Radians(math.Pi), decaderr.ErrNotFinite},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			_, _, err := exactrotation.New(tc.center, tc.axis, tc.angle)
			require.ErrorIs(t, err, tc.want)
		})
	}
}

func TestNewAgainstRefusesLargeGap(t *testing.T) {
	// A float transform for a different angle is far from the exact matrix.
	other, err := r3.RotationAround(r3.Vec{}, r3.Vec{X: 1}, units.Radians(1))
	require.NoError(t, err)
	_, err = exactrotation.NewAgainst(r3.Vec{}, r3.Vec{X: 1}, units.Radians(2), other)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)

	// The matching transform is admitted, and the identity is not a stand-in
	// for a quarter turn.
	quarter, err := r3.RotationAround(r3.Vec{}, r3.Vec{X: 1}, units.Radians(math.Pi/2))
	require.NoError(t, err)
	_, err = exactrotation.NewAgainst(r3.Vec{}, r3.Vec{X: 1}, units.Radians(math.Pi/2), quarter)
	require.NoError(t, err)
	_, err = exactrotation.NewAgainst(r3.Vec{}, r3.Vec{X: 1}, units.Radians(math.Pi/2), r3.Identity())
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

func TestGapCoversRoundingError(t *testing.T) {
	verts := []r3.Vec{{X: 1, Y: 2, Z: 3}, {X: -7.25, Y: 0.1, Z: 1e3}, {X: 1e-3, Y: -5, Z: 2}}
	for _, c := range rotCases() {
		rec, d := build(t, c)
		linear, translation, ok := rec.Gap(d)
		require.True(t, ok, "%s theta=%v", c.name, c.theta)
		require.False(t, math.IsInf(linear, 0) || math.IsNaN(linear))
		require.False(t, math.IsInf(translation, 0) || math.IsNaN(translation))
		m, tr := rec.Linear(), rec.Translation()
		basis := d.Basis()
		cols := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
		td := ratVec(d.Translation())
		for _, v := range verts {
			vv := ratVec(v)
			maxv := math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z)))
			qv := mulVec(m, vv)
			// D v + tD with D read exactly by columns.
			var dm [3][3]*big.Rat
			for j, col := range cols {
				cv := ratVec(col)
				for i := range 3 {
					dm[i][j] = cv[i]
				}
			}
			dv := mulVec(dm, vv)
			linearCap := new(big.Rat).Mul(ratOf(linear), ratOf(maxv))
			for i := range 3 {
				diff := new(big.Rat).Sub(qv[i], dv[i])
				require.LessOrEqual(t, diff.Abs(diff).Cmp(linearCap), 0, "%s theta=%v linear component %d", c.name, c.theta, i)
				tdiff := new(big.Rat).Sub(tr[i], td[i])
				require.LessOrEqual(t, tdiff.Abs(tdiff).Cmp(ratOf(translation)), 0, "%s translation component %d", c.name, i)
			}
		}
	}
}

func TestFixesXAxisAtOrigin(t *testing.T) {
	cases := []struct {
		name         string
		center, axis r3.Vec
		theta        float64
		want         bool
	}{
		{"X axis through origin", r3.Vec{}, r3.Vec{X: 1}, 2 * math.Pi * 23 / 31, true},
		{"X axis on the axis line", r3.Vec{X: 5}, r3.Vec{X: 1}, 1, true},
		{"identity about any axis", r3.Vec{}, r3.Vec{X: 1, Y: 2, Z: 2}, 0, true},
		{"Y axis", r3.Vec{}, r3.Vec{Y: 1}, 1, false},
		{"tilted axis", r3.Vec{}, r3.Vec{X: 1, Y: 1}, 1, false},
		{"offset X axis, tiny angle", r3.Vec{Y: 1, Z: 1}, r3.Vec{X: 1}, 0x1p-60, false},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			rec, _, err := exactrotation.New(tc.center, tc.axis, units.Radians(tc.theta))
			require.NoError(t, err)
			require.Equal(t, tc.want, rec.FixesXAxisAtOrigin())
		})
	}
	// The offset tiny-angle record has a nonzero exact translation even though
	// its float translation can be zero.
	rec, d, err := exactrotation.New(r3.Vec{Y: 1, Z: 1}, r3.Vec{X: 1}, units.Radians(0x1p-60))
	require.NoError(t, err)
	nonzero := false
	for _, q := range rec.Translation() {
		nonzero = nonzero || q.Sign() != 0
	}
	require.True(t, nonzero)
	_ = d
}

func TestAccessorsReturnCopies(t *testing.T) {
	rec, _, err := exactrotation.New(r3.Vec{Y: 1}, r3.Vec{X: 1}, units.Radians(1))
	require.NoError(t, err)
	m := rec.Linear()
	m[0][0].SetInt64(7)
	tr := rec.Translation()
	tr[0].SetInt64(7)
	require.Zero(t, rec.Linear()[0][0].Cmp(big.NewRat(1, 1)))
	require.NotZero(t, rec.Translation()[0].Cmp(big.NewRat(7, 1)))
}
