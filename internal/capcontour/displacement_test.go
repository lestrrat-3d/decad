package capcontour_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// TestDisplacementEnclosesExactLineDirection pins that a line wall's offset
// carrier runs along the direction its recorded endpoints DENOTE, not along
// the walk's held tangent. The corner (−1e−15, 0) sits below half an ulp of 24
// from the origin, so the two walls through it hold the rounded tangents
// (24, 0) and (−24, −32). The hypotenuse's held tangent has the exact length
// 40, so an enclosure built on it is a single point that the float miter lands
// on, while the miter the recorded endpoints denote sits about 1e−15 away. The
// reference miter points are solved at 400 bits from the exact endpoint
// differences.
//
// Shown to fail: with CarrierOver's line branch reading UnitVec(TanInU,
// TanInV) again, Displacement answers 0 against a true distance of 9.9e−16.
func TestDisplacementEnclosesExactLineDirection(t *testing.T) {
	t.Parallel()
	const d = 0.25
	corners := [][2]float64{{-1e-15, 0}, {24, 0}, {24, 32}}
	n := len(corners)
	walks := make([]survey2d.SideWalk, n)
	for i := range n {
		a, b := corners[i], corners[(i+1)%n]
		w, err := boundarywalk.WalkOf(sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: a[0], V: a[1]},
			End:   sectionrecord.Point2{U: b[0], V: b[1]},
			TEnd:  1,
		}, freeform.NewFreeformWork())
		require.NoError(t, err)
		walks[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
	}
	joins, err := offset2d.SectionJoinsBudget(proofbound.NewWorkBudget(t.Context()), walks, 1, d, 1e-9)
	require.NoError(t, err)
	readings := make([]capcontour.Join, n)
	for i, j := range joins {
		require.False(t, j.Arc || j.G1, "every corner of the triangle is a miter")
		readings[i] = capcontour.Join{VU: j.VertU, VV: j.VertV, M: sectionrecord.Point2{U: j.M.U, V: j.M.V}}
	}
	delta, ok := capcontour.Displacement(walks, readings, d, 0)
	require.True(t, ok)

	truth := new(big.Float).SetPrec(400)
	for i, j := range joins {
		prev, cur := corners[(i+n-1)%n], corners[i]
		next := corners[(i+1)%n]
		mu, mv := exactMiter(prev, cur, cur, next, d)
		du := new(big.Float).SetPrec(400).Sub(mu, new(big.Float).SetFloat64(j.M.U))
		dv := new(big.Float).SetPrec(400).Sub(mv, new(big.Float).SetFloat64(j.M.V))
		dist := new(big.Float).SetPrec(400).Add(new(big.Float).Mul(du, du), new(big.Float).Mul(dv, dv))
		dist.Sqrt(dist)
		if dist.Cmp(truth) > 0 {
			truth = dist
		}
	}
	require.Positive(t, truth.Sign(), "the held miters must differ from the denoted ones for the fixture to bite")
	require.GreaterOrEqual(t, new(big.Float).SetFloat64(delta).Cmp(truth), 0,
		"displacement %g must enclose the true distance %s", delta, truth.Text('g', 6))
}

// exactMiter solves, at 400 bits, the meeting point of the two lines a0→a1
// and b0→b1 each moved d along its own left unit normal.
func exactMiter(a0, a1, b0, b1 [2]float64, d float64) (*big.Float, *big.Float) {
	const prec = 400
	f := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
	row := func(p0, p1 [2]float64) (*big.Float, *big.Float, *big.Float) {
		// (−dy, dx)·X = (−dy, dx)·p0 + d·|(dx, dy)|
		dx := new(big.Float).SetPrec(prec).Sub(f(p1[0]), f(p0[0]))
		dy := new(big.Float).SetPrec(prec).Sub(f(p1[1]), f(p0[1]))
		l := new(big.Float).SetPrec(prec).Add(new(big.Float).Mul(dx, dx), new(big.Float).Mul(dy, dy))
		l.Sqrt(l)
		nu := new(big.Float).SetPrec(prec).Neg(dy)
		c := new(big.Float).SetPrec(prec).Add(new(big.Float).Mul(nu, f(p0[0])), new(big.Float).Mul(dx, f(p0[1])))
		c.Add(c, new(big.Float).Mul(f(d), l))
		return nu, dx, c
	}
	au, av, ac := row(a0, a1)
	bu, bv, bc := row(b0, b1)
	det := new(big.Float).SetPrec(prec).Sub(new(big.Float).Mul(au, bv), new(big.Float).Mul(av, bu))
	mu := new(big.Float).SetPrec(prec).Sub(new(big.Float).Mul(ac, bv), new(big.Float).Mul(av, bc))
	mv := new(big.Float).SetPrec(prec).Sub(new(big.Float).Mul(au, bc), new(big.Float).Mul(ac, bu))
	return mu.Quo(mu, det), mv.Quo(mv, det)
}
