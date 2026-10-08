package clearance_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/clearance/facepair"
	"github.com/lestrrat-3d/decad/internal/clearance/tier"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// requireEnclosesSurd requires [lo, hi] to hold a + s·√q, s = ±1, decided
// exactly over the rationals.
func requireEnclosesSurd(t *testing.T, lo, hi float64, a *big.Rat, s int, q *big.Rat) {
	t.Helper()
	x := new(big.Rat).Sub(new(big.Rat).SetFloat64(lo), a)
	y := new(big.Rat).Sub(new(big.Rat).SetFloat64(hi), a)
	if s < 0 {
		x, y = new(big.Rat).Sub(a, new(big.Rat).SetFloat64(hi)), new(big.Rat).Sub(a, new(big.Rat).SetFloat64(lo))
	}
	atLeastX := x.Sign() <= 0 || new(big.Rat).Mul(x, x).Cmp(q) <= 0
	atMostY := y.Sign() >= 0 && new(big.Rat).Mul(y, y).Cmp(q) >= 0
	require.Truef(t, atLeastX && atMostY, "[%.17g, %.17g] excludes %v %+d·√%v", lo, hi, a, s, q)
}

// TestDistReadingsEncloseTheTruth pins each closed-form reading against its
// exact value: an irrational one is enclosed and not exact, a value the
// float holds is that single point.
func TestDistReadingsEncloseTheTruth(t *testing.T) {
	t.Parallel()
	zero := new(big.Rat)
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	for _, tc := range []struct {
		name string
		d    clearance.Dist
		a    *big.Rat
		s    int
		q    *big.Rat
	}{
		{"point × point √3", clearance.PointPointDist(r3.Vec{}, r3.NewVec(1, 1, 1)), zero, 1, big.NewRat(3, 1)},
		{"point × line off the axes", clearance.PointLineDist(x, r3.Vec{}, proofarith.DyVec(r3.NewVec(1, 1, 0))), zero, 1, big.NewRat(1, 2)},
		{"height over a slanted plane", clearance.Height(r3.NewVec(2, 0, 0), r3.Vec{}, r3.NewVec(1, 1, 0)), zero, 1, big.NewRat(2, 1)},
		{"height below a plane", clearance.Height(r3.NewVec(0, 0, -3), r3.Vec{}, z), big.NewRat(-3, 1), 1, zero},
		{"skew lines", clearance.LineLineDist(r3.Vec{}, proofarith.DyVec(x), r3.NewVec(0, 0, 1), proofarith.DyVec(r3.NewVec(1, 1, 0))), big.NewRat(1, 1), 1, zero},
		{"point × circle near", clearance.PointCircleDist(r3.NewVec(3, 4, 1), r3.Vec{}, z, 3, 1), zero, 1, big.NewRat(5, 1)},
		{"point × circle far", clearance.PointCircleDist(r3.NewVec(3, 4, 1), r3.Vec{}, z, 3, -1), zero, 1, big.NewRat(65, 1)},
		{"point on the circle's axis", clearance.PointCircleDist(r3.NewVec(0, 0, 4), r3.Vec{}, z, 3, 1), big.NewRat(5, 1), 1, zero},
		{"coaxial circles", clearance.CircleCircleCoaxialDist(r3.NewVec(0, 0, 3), r3.Vec{}, z, 5, 10), zero, 1, big.NewRat(34, 1)},
		{"amplitude", clearance.Amplitude(r3.NewVec(1, 0, 1), z, 2), zero, 1, big.NewRat(2, 1)},
		{"radius difference", clearance.PointDist(1).Sub(0.3), new(big.Rat).Sub(big.NewRat(1, 1), new(big.Rat).SetFloat64(0.3)), 1, zero},
		{"radius sum", clearance.PointDist(0.3).Add(0.6), new(big.Rat).Add(new(big.Rat).SetFloat64(0.3), new(big.Rat).SetFloat64(0.6)), 1, zero},
		{"in-plane axis", clearance.PointLineDist(r3.NewVec(5, 7, 9), r3.NewVec(5, 7, 0), proofarith.DyVec(y)), big.NewRat(9, 1), 1, zero},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			requireEnclosesSurd(t, tc.d.Lo, tc.d.Hi, tc.a, tc.s, tc.q)
			q, _ := tc.q.Float64()
			if q == 0 {
				// A rational truth: exact exactly where the float holds it.
				held := new(big.Rat).SetFloat64(tc.d.Lo)
				require.Equal(t, held.Cmp(tc.a) == 0, tc.d.Exact())
				return
			}
			isSquare := new(big.Rat).SetFloat64(proofbound.RatSqrtDown(tc.q))
			if new(big.Rat).Mul(isSquare, isSquare).Cmp(tc.q) != 0 {
				require.False(t, tc.d.Exact(), "an irrational truth is never a single float")
			}
		})
	}
}

// TestRevolvedCellsEncloseTheTruth pins the closed-form cells only a revolve
// reaches — vertex × sphere, vertex × torus, cone × sphere and vertex × cone —
// at a point whose true distance no float holds. A published row through
// them is today widened by the revolve's own angular term, so the held
// interval is pinned here directly.
//
// Shown to fail: before the cells read their distances as Dist enclosures,
// each held a single point marked exact: vertex × sphere 2.1961524227066320
// against 3·√3 − 3, vertex × torus 1.1622776601683795 against √10 − 2,
// cone × sphere and vertex × cone, at slope 1, 6·√2 − 1/4 and 6·√2 read
// 8.2352813742385695 and 8.4852813742385695.
func TestRevolvedCellsEncloseTheTruth(t *testing.T) {
	t.Parallel()
	const tol = 1e-9 * 64
	t.Run("vertex × sphere", func(t *testing.T) {
		t.Parallel()
		sink := &clearance.CellSink{}
		k := tier.New(t.Context(), tol, tol)
		require.NoError(t, k.VertexFace(proofbound.NewWorkBudget(t.Context()), r3.NewVec(3, 3, 3), sphereFace(r3.Vec{}, 3), sink))
		lo, hi, exact, ok := sink.Interval()
		require.True(t, ok)
		require.False(t, exact)
		requireEnclosesSurd(t, lo, hi, big.NewRat(-3, 1), 1, big.NewRat(27, 1))
	})
	t.Run("vertex × torus", func(t *testing.T) {
		t.Parallel()
		sink := &clearance.CellSink{}
		k := tier.New(t.Context(), tol, tol)
		require.NoError(t, k.VertexFace(proofbound.NewWorkBudget(t.Context()), r3.NewVec(13, 0, 1), torusFace(10, 2), sink))
		lo, hi, exact, ok := sink.Interval()
		require.True(t, ok)
		require.False(t, exact)
		requireEnclosesSurd(t, lo, hi, big.NewRat(-2, 1), 1, big.NewRat(10, 1))
	})
	// The cone of slope 1 swept from (2²⁰, 0) about the x axis; P sits s
	// along the wall's outward normal (−1, 1) from the wall point at
	// m = 2¹⁵, so it lies s·√2 from the cone.
	const z0, k, s = 1048576.0, 65536.0, 6.0
	const m = k / 2
	p := r3.NewVec(z0+m-s, m+s, 0)
	t.Run("cone × sphere", func(t *testing.T) {
		t.Parallel()
		cone := steepCone(t, z0, k, 1, 1)
		sink := &clearance.CellSink{}
		fk := facepair.New(t.Context(), 1e-9*(z0+k), 1e-9*(z0+k))
		fk.FaceCell(cone, sphereFace(p, 0.25), sink)
		require.NoError(t, fk.Err())
		lo, hi, exact, ok := sink.Interval()
		require.True(t, ok)
		require.False(t, exact)
		requireEnclosesSurd(t, lo, hi, big.NewRat(-1, 4), 1, big.NewRat(2*s*s, 1))
	})
	t.Run("vertex × cone", func(t *testing.T) {
		t.Parallel()
		cone := steepCone(t, z0, k, 1, 1)
		sink := &clearance.CellSink{}
		tk := tier.New(t.Context(), 1e-9*(z0+k), 1e-9*(z0+k))
		require.NoError(t, tk.VertexFace(proofbound.NewWorkBudget(t.Context()), p, cone, sink))
		lo, hi, exact, ok := sink.Interval()
		require.True(t, ok)
		require.False(t, exact)
		requireEnclosesSurd(t, lo, hi, new(big.Rat), 1, big.NewRat(2*s*s, 1))
	})
}

// torusFace is a whole torus of major radius major and minor radius minor
// about the z axis through the origin.
func torusFace(major, minor float64) *clearance.CFace {
	axis := r3.NewVec(0, 0, 1)
	u := r3.NewVec(1, 0, 0)
	ext := major + minor
	return &clearance.CFace{
		Kind:   clearance.CkTorus,
		Axis:   axis,
		RefU:   u,
		RefV:   axis.Cross(u),
		Radius: minor,
		Major:  major,
		Sweep:  clearance.AngWindow{Full: true},
		Merid:  clearance.AngWindow{Full: true},
		Box:    [2]r3.Vec{r3.NewVec(-ext, -ext, -minor), r3.NewVec(ext, ext, minor)},
		Wit:    []r3.Vec{u.Scale(ext)},
	}
}
