package exactbool_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/exactbool"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

type ratVec [3]*big.Rat

func ratOf(v r3.Vec) ratVec {
	return ratVec{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
}

func (a ratVec) sub(b ratVec) ratVec {
	return ratVec{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
}

func (a ratVec) cross(b ratVec) ratVec {
	m := func(x, y, z, w *big.Rat) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(x, y), new(big.Rat).Mul(z, w))
	}
	return ratVec{m(a[1], b[2], a[2], b[1]), m(a[2], b[0], a[0], b[2]), m(a[0], b[1], a[1], b[0])}
}

func (a ratVec) dot(b ratVec) *big.Rat {
	s := new(big.Rat)
	for i := range a {
		s.Add(s, new(big.Rat).Mul(a[i], b[i]))
	}
	return s
}

// refOrient is the triple product (b−a)×(c−a)·(d−a) in rationals, written without the package under test.
func refOrient(a, b, c, d r3.Vec) int {
	ra := ratOf(a)
	return ratOf(b).sub(ra).cross(ratOf(c).sub(ra)).dot(ratOf(d).sub(ra)).Sign()
}

// scene draws points from a small integer lattice (many coplanar quadruples), from the same lattice with one
// coordinate nudged by an ulp (near-coplanar), and from wide random floats.
func scene(rng *rand.Rand, kind int) r3.Vec {
	switch kind {
	case 0:
		return r3.NewVec(float64(rng.IntN(5)), float64(rng.IntN(5)), float64(rng.IntN(5)))
	case 1:
		p := r3.NewVec(float64(rng.IntN(3)), float64(rng.IntN(3)), 0)
		return r3.NewVec(p.X, p.Y, math.Nextafter(0, float64(rng.IntN(3)-1)))
	default:
		return r3.NewVec(rng.NormFloat64()*1e3, rng.NormFloat64()*1e-3, rng.NormFloat64())
	}
}

func TestOrient3DMatchesRationalReference(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(1, 2))
	zero := 0
	for i := range 60000 {
		kind := i % 3
		a, b, c, d := scene(rng, kind), scene(rng, kind), scene(rng, kind), scene(rng, kind)
		want := refOrient(a, b, c, d)
		if want == 0 {
			zero++
		}
		require.Equal(t, want, exactbool.Orient3D(a, b, c, d), "%v %v %v %v", a, b, c, d)
	}
	require.Greater(t, zero, 1000, "the lattice draws must include many exactly coplanar quadruples")
}

func TestOrient3DOfRoundedCoplanarPointsMatchesReference(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(5, 6))
	nonzero := 0
	for range 40000 {
		a := r3.NewVec(rng.NormFloat64(), rng.NormFloat64(), rng.NormFloat64())
		b := r3.NewVec(rng.NormFloat64(), rng.NormFloat64(), rng.NormFloat64())
		c := r3.NewVec(rng.NormFloat64(), rng.NormFloat64(), rng.NormFloat64())
		s, u := rng.NormFloat64(), rng.NormFloat64()
		// d lies on the plane through a, b, c up to the rounding of this sum.
		d := a.Add(b.Sub(a).Scale(s)).Add(c.Sub(a).Scale(u))
		want := refOrient(a, b, c, d)
		if want != 0 {
			nonzero++
		}
		require.Equal(t, want, exactbool.Orient3D(a, b, c, d))
	}
	require.Greater(t, nonzero, 1000)
}

func TestOrient3DNearCoplanarFloatsAreDecidedExactly(t *testing.T) {
	t.Parallel()
	// d is one ulp above the plane z = 0 through three points far from the origin: the float determinant is zero or noise.
	a, b, c := r3.NewVec(1e8, 0, 0), r3.NewVec(0, 1e8, 0), r3.NewVec(-1e8, -1e8, 0)
	above := r3.NewVec(0.1, 0.1, math.Nextafter(0, 1))
	below := r3.NewVec(0.1, 0.1, math.Nextafter(0, -1))
	require.Equal(t, 1, exactbool.Orient3D(a, b, c, above))
	require.Equal(t, -1, exactbool.Orient3D(a, b, c, below))
	require.Equal(t, 0, exactbool.Orient3D(a, b, c, r3.NewVec(0.1, 0.1, 0)))
}

// refMeet solves the three plane equations by Cramer's rule in rationals.
func refMeet(p, q, s [4]*big.Rat) (ratVec, bool) {
	n := func(pl [4]*big.Rat) ratVec { return ratVec{pl[0], pl[1], pl[2]} }
	d := n(p).dot(n(q).cross(n(s)))
	if d.Sign() == 0 {
		return ratVec{}, false
	}
	var x ratVec
	hq := n(q).cross(n(s))
	hs := n(s).cross(n(p))
	hp := n(p).cross(n(q))
	for i := range x {
		sum := new(big.Rat).Mul(p[3], hq[i])
		sum.Add(sum, new(big.Rat).Mul(q[3], hs[i]))
		sum.Add(sum, new(big.Rat).Mul(s[3], hp[i]))
		x[i] = sum.Quo(sum, d)
	}
	return x, true
}

func refPlane(a, b, c r3.Vec) [4]*big.Rat {
	ra := ratOf(a)
	n := ratOf(b).sub(ra).cross(ratOf(c).sub(ra))
	return [4]*big.Rat{n[0], n[1], n[2], n.dot(ra)}
}

func TestImplicitPointsMatchRationalReference(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(3, 4))
	meets := 0
	for i := range 6000 {
		kind := i % 3
		var pts [9]r3.Vec
		for k := range pts {
			pts[k] = scene(rng, kind)
		}
		p := exactbool.PlaneThrough(pts[0], pts[1], pts[2])
		q := exactbool.PlaneThrough(pts[3], pts[4], pts[5])
		s := exactbool.PlaneThrough(pts[6], pts[7], pts[8])
		rp, rq, rs := refPlane(pts[0], pts[1], pts[2]), refPlane(pts[3], pts[4], pts[5]), refPlane(pts[6], pts[7], pts[8])
		want, wantOK := refMeet(rp, rq, rs)
		x, ok := exactbool.Meet(p, q, s)
		require.Equal(t, wantOK, ok)
		if !ok {
			continue
		}
		meets++
		// The point lies on all three planes, exactly.
		for _, pl := range []exactbool.Plane{p, q, s} {
			require.Equal(t, 0, pl.SideOfPoint(x))
		}
		// Side of a fourth plane, and the order of two points, agree with the rational coordinates.
		other := exactbool.PlaneThrough(pts[0], pts[4], pts[8])
		ro := refPlane(pts[0], pts[4], pts[8])
		sum := new(big.Rat)
		for k := range 3 {
			sum.Add(sum, new(big.Rat).Mul(ro[k], want[k]))
		}
		require.Equal(t, sum.Sub(sum, ro[3]).Sign(), other.SideOfPoint(x))
		y, yok := exactbool.Meet(q, s, other)
		if yok {
			ry, _ := refMeet(rq, rs, ro)
			for axis := range 3 {
				require.Equal(t, want[axis].Cmp(ry[axis]), exactbool.Order(axis, x, y))
			}
			require.Equal(t, want[0].Cmp(ry[0]) == 0 && want[1].Cmp(ry[1]) == 0 && want[2].Cmp(ry[2]) == 0,
				exactbool.Equal(x, y))
		}
		// The rounded point is the nearest float and its bound covers the exact point.
		f, bound := x.Float()
		for axis, c := range []float64{f.X, f.Y, f.Z} {
			exact, _ := want[axis].Float64()
			require.Equal(t, exact, c)
			require.LessOrEqual(t, math.Abs(c-exact), bound)
		}
	}
	require.Greater(t, meets, 3000)
}

func TestPlaneThroughCollinearPointsIsDegenerate(t *testing.T) {
	t.Parallel()
	p := exactbool.PlaneThrough(r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 1), r3.NewVec(2, 2, 2))
	require.True(t, p.Degenerate())
	_, ok := exactbool.Meet(p, exactbool.PlaneThrough(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)),
		exactbool.PlaneThrough(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 1), r3.NewVec(0, 1, 1)))
	require.False(t, ok)
}
