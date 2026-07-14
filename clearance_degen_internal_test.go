package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The degeneracy oracle is the kernel's one gate between a closed form and a
// certificate: it answers YES only on exact arithmetic over the payload's own
// floats, NO only on a residual clearly above the kernel's noise, and UNKNOWN
// in the band between — where a certificate would be a lie. These tests pin all
// three answers, and pin the two cells the boundary tiers cannot out-vote: a
// closed circle edge against a full sphere/torus face (no edges exist there to
// hold the true minimum) and two nearly coaxial FULL tori.

func testKernel() *pairKernel {
	return &pairKernel{scale: 10, tol: 1e-9 * 10, slack: 1e-9 * 10}
}

func TestDegOracleIsThreeValued(t *testing.T) {
	k := testKernel()
	z := r3.NewVec(0, 0, 1)

	t.Run("parallel", func(t *testing.T) {
		require.Equal(t, degYes, k.parallel(z, r3.NewVec(0, 0, -4)), `exactly antiparallel is parallel`)
		require.Equal(t, degNo, k.parallel(z, r3.NewVec(1, 0, 1)), `45° apart`)
		// A tilt of 1e-12 rad: too small to disprove parallelism, not zero.
		require.Equal(t, degUnknown, k.parallel(z, r3.NewVec(1e-12, 0, 1)))
	})

	t.Run("onAxis", func(t *testing.T) {
		anchor := r3.NewVec(0, 0, 3)
		require.Equal(t, degYes, k.onAxis(r3.NewVec(0, 0, -7), anchor, z))
		require.Equal(t, degYes, k.onAxis(anchor, anchor, z), `the anchor itself`)
		require.Equal(t, degNo, k.onAxis(r3.NewVec(0.5, 0, 0), anchor, z))
		// 1e-12 mm off the axis, under the kernel's 1e-8 mm length noise.
		require.Equal(t, degUnknown, k.onAxis(r3.NewVec(1e-12, 0, 0), anchor, z))
	})

	t.Run("coincident", func(t *testing.T) {
		p := r3.NewVec(1, 2, 3)
		require.Equal(t, degYes, k.coincident(p, r3.NewVec(1, 2, 3)))
		require.Equal(t, degNo, k.coincident(p, r3.NewVec(1, 2, 3.5)))
		require.Equal(t, degUnknown, k.coincident(p, r3.NewVec(1, 2, 3+1e-12)))
	})

	t.Run("a non-finite input never proves a degeneracy", func(t *testing.T) {
		bad := r3.NewVec(math.Inf(1), 0, 0)
		require.Equal(t, degUnknown, k.parallel(bad, z))
		require.Equal(t, degUnknown, k.coincident(bad, bad))
	})
}

// cylFace/sphFace/torFace build the kernel's carrier records directly — the
// spine geometry is all these cells read.
func cylFace(anchor, axis r3.Vec, radius float64) *cFace {
	u := perpTo(axis)
	return &cFace{
		kind: ckCylinder, anchor: anchor, axis: axis, refU: u, refV: axis.Cross(u),
		radius: radius, zWin: linWindow{lo: -100, hi: 100}, sweep: angWindow{full: true},
	}
}

func sphFace(center r3.Vec, radius float64) *cFace {
	u := perpTo(r3.NewVec(0, 0, 1))
	return &cFace{
		kind: ckSphere, anchor: center, axis: r3.NewVec(0, 0, 1), refU: u, refV: r3.NewVec(0, 0, 1).Cross(u),
		radius: radius, merid: angWindow{full: true}, sweep: angWindow{full: true},
	}
}

func torFace(center, axis r3.Vec, major, minor float64) *cFace {
	u := perpTo(axis)
	return &cFace{
		kind: ckTorus, anchor: center, axis: axis, refU: u, refV: axis.Cross(u),
		major: major, radius: minor, merid: angWindow{full: true}, sweep: angWindow{full: true},
	}
}

func TestDegSpineSupIsBoundedOnlyWhenProven(t *testing.T) {
	k := testKernel()
	z := r3.NewVec(0, 0, 1)

	t.Run("a cylinder's line spine is unbounded against a point spine", func(t *testing.T) {
		// The Class-A configuration: a rod at (3, 0) and a ball at the origin.
		// The rod's spine is an INFINITE line, so its supremum over the ball's
		// point spine is +∞ — not the 3 mm foot distance.
		rod := cylFace(r3.NewVec(3, 0, 0), z, 1)
		ball := sphFace(r3.NewVec(0, 0, 0), 10)

		sup, ok := k.spineSup(rod, ball)
		require.False(t, ok, `an infinite line has no finite supremum against a point`)
		require.True(t, math.IsInf(sup, 1))

		// The other direction IS bounded: the supremum over ONE point is that
		// point's own distance.
		sup, ok = k.spineSup(ball, rod)
		require.True(t, ok)
		require.InDelta(t, 3.0, sup, 1e-12)
	})

	t.Run("exactly parallel line spines are constant", func(t *testing.T) {
		sup, ok := k.spineSup(cylFace(r3.NewVec(0, 0, 0), z, 5), cylFace(r3.NewVec(4, 0, 0), z, 10))
		require.True(t, ok)
		require.InDelta(t, 4.0, sup, 1e-12)
	})

	t.Run("a tilted line spine is unbounded", func(t *testing.T) {
		tilted, ok := r3.NewVec(1, 0, 1).Normalize()
		require.True(t, ok)
		_, ok = k.spineSup(cylFace(r3.NewVec(0, 0, 0), z, 5), cylFace(r3.NewVec(4, 0, 0), tilted, 10))
		require.False(t, ok)
	})

	t.Run("a circle spine needs exact coaxiality", func(t *testing.T) {
		outer := torFace(r3.NewVec(0, 0, 0), z, 10, 2)
		inner := torFace(r3.NewVec(0, 0, 0), z, 5, 1)
		sup, ok := k.spineSup(inner, outer)
		require.True(t, ok)
		require.InDelta(t, 5.0, sup, 1e-12, `hypot(0, 10 − 5)`)

		off := torFace(r3.NewVec(1e-12, 0, 0), z, 5, 1)
		_, ok = k.spineSup(off, outer)
		require.False(t, ok, `a 1e-12 offset is not coaxial, and has no constant supremum`)
	})
}

func TestDegCertifiedContainment(t *testing.T) {
	k := testKernel()
	z := r3.NewVec(0, 0, 1)

	t.Run("a rod piercing a ball is never nested", func(t *testing.T) {
		rod := cylFace(r3.NewVec(3, 0, 0), z, 1)
		ball := sphFace(r3.NewVec(0, 0, 0), 10)
		require.False(t, k.certifiedContainment(rod, ball),
			`the rod's carrier runs to infinity through the ball's skin`)
		require.False(t, k.certifiedContainment(ball, rod))
	})

	t.Run("the coaxial peg in a hole is nested", func(t *testing.T) {
		peg := cylFace(r3.NewVec(0, 0, 0), z, 5)
		hole := cylFace(r3.NewVec(0, 0, 0), z, 10)
		require.True(t, k.certifiedContainment(peg, hole))
	})

	t.Run("a ball on the hole axis is nested", func(t *testing.T) {
		ball := sphFace(r3.NewVec(0, 0, 4), 5)
		hole := cylFace(r3.NewVec(0, 0, 0), z, 10)
		require.True(t, k.certifiedContainment(ball, hole),
			`the supremum over one point is its own distance — the doc's certified point spine`)
	})

	t.Run("a ball too big for the hole is not nested", func(t *testing.T) {
		ball := sphFace(r3.NewVec(0, 0, 4), 12)
		hole := cylFace(r3.NewVec(0, 0, 0), z, 10)
		require.False(t, k.certifiedContainment(ball, hole))
	})
}

func TestDegPointCircleCritsNeverGuessAnAzimuth(t *testing.T) {
	k := testKernel()
	z := r3.NewVec(0, 0, 1)
	center := r3.NewVec(0, 0, 0)
	u, v := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
	arc := newAngWindow(math.Pi/2, math.Pi) // a quarter arc, away from refU

	t.Run("a point exactly on the axis is one constant critical inside the trim", func(t *testing.T) {
		crits, ok := k.pointCircleCrits(r3.NewVec(0, 0, 8), center, z, u, v, 6, arc)
		require.True(t, ok)
		require.Len(t, crits, 1)
		require.True(t, crits[0].exact)
		require.InDelta(t, 10.0, crits[0].lo, 1e-12, `hypot(8, 6) at every azimuth`)
		// The representative must sit INSIDE the arc, not at a fixed refU.
		th := math.Atan2(crits[0].fb.Y, crits[0].fb.X)
		require.Equal(t, 1, arc.classify(th, 1e-9))
	})

	t.Run("a point provenly off the axis carries the near and far criticals", func(t *testing.T) {
		crits, ok := k.pointCircleCrits(r3.NewVec(2, 0, 0), center, z, u, v, 6, angWindow{full: true})
		require.True(t, ok)
		require.Len(t, crits, 2)
		require.InDelta(t, 4.0, crits[0].lo, 1e-12)
		require.InDelta(t, 8.0, crits[1].lo, 1e-12)
	})

	t.Run("an offset in the undecided band yields no critical at all", func(t *testing.T) {
		// A closed circle edge against a full sphere face: neither carries an
		// edge or vertex tier that could out-vote a wrong Exact here, so the
		// cell must decline rather than pick an arbitrary azimuth.
		_, ok := k.pointCircleCrits(r3.NewVec(1e-12, 0, 8), center, z, u, v, 6, angWindow{full: true})
		require.False(t, ok)
	})
}

func TestDegCircleCircleCritsNeedExactCoaxiality(t *testing.T) {
	k := testKernel()
	z := r3.NewVec(0, 0, 1)
	outer := torFace(r3.NewVec(0, 0, 0), z, 10, 2)

	t.Run("exactly coaxial spines carry the constant closed form", func(t *testing.T) {
		crits, ok := k.circleCircleCrits(torFace(r3.NewVec(0, 0, 0), z, 5, 1), outer)
		require.True(t, ok)
		require.Len(t, crits, 1)
		require.True(t, crits[0].exact)
		require.InDelta(t, 5.0, crits[0].lo, 1e-12)
	})

	t.Run("a 1e-12 offset ACROSS the axis carries none", func(t *testing.T) {
		_, ok := k.circleCircleCrits(torFace(r3.NewVec(1e-12, 0, 0), z, 5, 1), outer)
		require.False(t, ok, `two full tori have no boundary tier to out-vote a wrong Exact`)
	})

	t.Run("an offset ALONG the axis is still coaxial", func(t *testing.T) {
		crits, ok := k.circleCircleCrits(torFace(r3.NewVec(0, 0, 3), z, 5, 1), outer)
		require.True(t, ok)
		require.True(t, crits[0].exact)
		require.InDelta(t, math.Hypot(3, 5), crits[0].lo, 1e-12)
	})
}
