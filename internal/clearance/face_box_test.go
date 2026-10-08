package clearance_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// requireBoxHolds requires each coordinate of the exact point p to lie in
// the box, decided over the rationals.
func requireBoxHolds(t *testing.T, box [2]r3.Vec, p [3]*big.Rat) {
	t.Helper()
	lo := [3]float64{box[0].X, box[0].Y, box[0].Z}
	hi := [3]float64{box[1].X, box[1].Y, box[1].Z}
	for i := range 3 {
		require.LessOrEqual(t, new(big.Rat).SetFloat64(lo[i]).Cmp(p[i]), 0, "coordinate %d below the box", i)
		require.GreaterOrEqual(t, new(big.Rat).SetFloat64(hi[i]).Cmp(p[i]), 0, "coordinate %d above the box", i)
	}
}

// TestCircleBoxHoldsATiltedCircle pins CircleBox on an axis tilted 1e-9 rad
// off z, whose z component rounds to 1, so 1 − a_z² is zero in float while
// the circle of radius 10 spans 10·|a_xy|/|a| along z. The box must reach
// that half-width above and below the centre, compared by squares.
//
// Seen red: reading the half-width as r·√(1 − a_z²) puts the box's z extent
// at zero.
func TestCircleBoxHoldsATiltedCircle(t *testing.T) {
	t.Parallel()
	axis := r3.NewVec(0, 1e-9, 1)
	const r = 10.0
	box := clearance.CircleBox(r3.Vec{}, axis, r)
	ay, az := new(big.Rat).SetFloat64(axis.Y), new(big.Rat).SetFloat64(axis.Z)
	ay2 := new(big.Rat).Mul(ay, ay)
	den := new(big.Rat).Add(ay2, new(big.Rat).Mul(az, az))
	want := new(big.Rat).Mul(big.NewRat(100, 1), ay2)
	want.Quo(want, den)
	for _, z := range []float64{box[1].Z, -box[0].Z} {
		got := new(big.Rat).SetFloat64(z)
		require.GreaterOrEqual(t, new(big.Rat).Mul(got, got).Cmp(want), 0, "the box spans %g along z", z)
	}
	require.Less(t, box[1].Z, 1e-7)
}

// TestCapBoxHoldsACancellingLift pins CapBox on a plane whose origin sits 2³⁰
// from the face it bounds and whose u direction is tilted: o + u·x cancels
// to about 1 while each term is about 2³⁰, so a float sum rounds by about
// 1e-7. Every exact corner o + u·x + v·y must lie in the box.
//
// Seen red: reading each corner as the float sum o + u·x + v·y leaves the
// exact corner outside the box.
func TestCapBoxHoldsACancellingLift(t *testing.T) {
	t.Parallel()
	u := r3.NewVec(math.Cos(0.1), math.Sin(0.1), 0)
	v := r3.NewVec(-math.Sin(0.1), math.Cos(0.1), 0)
	const far = 1 << 30
	o := u.Scale(-far)
	x0, x1 := float64(far)+0.3, float64(far)+1.7
	var elems []survey2d.SurveyElem
	corners := [][2]float64{{x0, 0}, {x1, 0}, {x1, 1}, {x0, 1}}
	for i := range corners {
		a, b := corners[i], corners[(i+1)%len(corners)]
		el, ok := survey2d.LineElem(a[0], a[1], b[0], b[1])
		require.True(t, ok)
		elems = append(elems, el)
	}
	f := &clearance.CFace{Kind: clearance.CkPlane, O: o, U: u, V: v, N: r3.NewVec(0, 0, 1), Region: clearance.NewRegion2(elems)}
	box := clearance.CapBox(f)
	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	for _, c := range corners {
		var p [3]*big.Rat
		for i, oc := range [3]float64{o.X, o.Y, o.Z} {
			uc := [3]float64{u.X, u.Y, u.Z}[i]
			vc := [3]float64{v.X, v.Y, v.Z}[i]
			p[i] = new(big.Rat).Add(rat(oc), new(big.Rat).Add(new(big.Rat).Mul(rat(uc), rat(c[0])), new(big.Rat).Mul(rat(vc), rat(c[1]))))
		}
		requireBoxHolds(t, box, p)
	}
}
