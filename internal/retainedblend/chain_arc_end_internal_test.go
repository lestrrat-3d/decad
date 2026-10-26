package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// arcEndFixture is the top rim of the rounded corner (centre (3,3,20), radius 3,
// material inside) as a chain's first arc: it starts on the plane x = 1.5 and
// runs to the tangent point (3,0,20).
func arcEndFixture(t *testing.T, rowsM int) (chainEntry, *chainEndCut) {
	t.Helper()
	n := r3.NewVec(0, 0, 1)
	u, ok := perpendicular(n)
	require.True(t, ok)
	rm := rim{center: r3.NewVec(3, 3, 20), n: n, u: u, w: n.Cross(u), radius: 3, kappa: -1, side: -1, convex: true,
		scale: 40}
	a := arcRim{rim: rm, sign: 1}
	g := plane{n: r3.NewVec(1, 0, 0), o: r3.NewVec(1.5, 0, 0)}
	corner := rm.angle(r3.NewVec(1.5, 3-math.Sqrt(6.75), 20))
	roots, ok := a.ringAzimuths(3, g, corner)
	require.True(t, ok)
	exact := a.atTheta(3, roots[0], 0)
	if math.Abs(roots[1]-corner) < math.Abs(roots[0]-corner) {
		exact = a.atTheta(3, roots[1], 0)
	}
	tangent := r3.NewVec(3, 0, 20)
	sigma := 1.0
	span := sigma * (rm.angle(tangent) - rm.angle(exact))
	if span < 0 {
		sigma, span = -1, -span
	}
	span = math.Mod(span, 2*math.Pi)
	step := blendSpec{fillet: true, r: 1}
	ec := &chainEndCut{exact: exact, g: g, arc: &arcEnd{a: a, g: g}}
	p, _ := a.profile(step)
	ss := make([]float64, rowsM)
	for j := range ss {
		ss[j] = float64(j) / float64(rowsM)
	}
	var err error
	ec.arc.ang, err = ec.arc.endAngles(p, ss, exact)
	require.NoError(t, err)
	en := chainEntry{arc: true, rim: rm, sigma: sigma, phi0: rm.angle(exact), span: span, p0: exact,
		p1: tangent, step: step, freeStart: ec.arc}
	return en, ec
}

func TestArcEndRowsLieOnTheEndFace(t *testing.T) {
	t.Parallel()
	const rowsM = 12
	en, ec := arcEndFixture(t, rowsM)
	start, end := en.fanBounds(rowsM)
	p, _ := ec.arc.a.profile(en.step)
	for j := range rowsM {
		rho, z := p(float64(j) / rowsM)
		// Each row of the fan starts where its circle meets G.
		at := en.rim.at(rho, start[j], z)
		require.InDelta(t, 0, ec.g.offset(at), 1e-12, "row %d", j)
		// And ends on the meridian of the tangent point.
		require.InDelta(t, en.rim.angle(en.p1), end[j], 1e-12, "row %d", j)
		require.Positive(t, en.sigma*(end[j]-start[j]), "row %d runs forward", j)
	}
	require.NoError(t, en.checkFan(rowsM))
}

func TestArcEndCornerAreaMatchesAnIndependentSum(t *testing.T) {
	t.Parallel()
	en, ec := arcEndFixture(t, 4)
	p, _ := ec.arc.a.profile(en.step)
	got := ec.arc.cornerAreaOf(p, ec.exact)
	// The corner region of the plane x = 1.5, in (y, z): between the end
	// curve and the two lines through the corner.
	const n, r, R, h = 200000, 1.0, 3.0, 1.5
	y := func(rho float64) float64 { return R - math.Sqrt(rho*rho-h*h) }
	pts := [][2]float64{{y(R), 0}}
	for i := 0; i <= n; i++ {
		a := math.Pi / 2 * float64(i) / n
		pts = append(pts, [2]float64{y(R - r*(1-math.Sin(a))), -r * (1 - math.Cos(a))})
	}
	twice := 0.0
	for i := range pts {
		q := pts[(i+1)%len(pts)]
		twice += pts[i][0]*q[1] - q[0]*pts[i][1]
	}
	require.InDelta(t, math.Abs(twice)/2, got, 1e-6)
	require.Greater(t, got, 0.1)
}

func TestArcEndAreasMatchAnIndependentSum(t *testing.T) {
	t.Parallel()
	en, _ := arcEndFixture(t, 1)
	const n, r, R, h = 200000, 1.0, 3.0, 1.5
	// The angle a circle of radius rho covers from G to the tangent point.
	cover := func(rho float64) float64 { return math.Pi/2 - math.Acos(h/rho) }
	var base, blend float64
	for i := range n {
		a := math.Pi / 2 * (float64(i) + 0.5) / n
		rho := R - r*(1-math.Sin(a))
		blend += r * (math.Pi / 2 / n) * rho * cover(rho)
		rb := R - r + r*(float64(i)+0.5)/n
		base += (r / n) * rb * cover(rb)
	}
	gotBase, gotWall, gotBlend := en.areasFree(r, r)
	require.InDelta(t, base, gotBase, 1e-6)
	require.InDelta(t, R*cover(R)*r, gotWall, 1e-12)
	require.InDelta(t, blend, gotBlend, 1e-4)
}
