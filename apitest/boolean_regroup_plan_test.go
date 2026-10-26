package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The hand-built plans of agent-docs/boolean-chain-regroup-design.md (Test plan): each
// fixture's sequential chain refuses, and the regrouped plan builds through public calls.
// The test pins the premises the regroup depends on, namely which steps take the mesh path.

// regroupBoxVolume is the volume of retentionScene.roundedBox: 40x20x20 with four vertical
// edges filleted at r=3.
var regroupBoxVolume = (800 - 4*(1-math.Pi/4)*9) * 20

// regroupLens is the area of two intersecting discs' overlap.
func regroupLens(r1, r2, d float64) float64 {
	return r1*r1*math.Acos((d*d+r1*r1-r2*r2)/(2*d*r1)) +
		r2*r2*math.Acos((d*d+r2*r2-r1*r1)/(2*d*r2)) -
		0.5*math.Sqrt((-d+r1+r2)*(d+r1-r2)*(d-r1+r2)*(d+r1+r2))
}

// regroupCrossVolume is the volume shared by two Ø8 and Ø6 cylinders with crossing axes:
// 36 times the integral of cos²φ·√(16−9·sin²φ) over [−π/2, π/2], by the trapezoid rule on a
// smooth, π-periodic integrand.
func regroupCrossVolume(n int) float64 {
	h := math.Pi / float64(n)
	sum := 0.0
	for i := range n {
		phi := -math.Pi/2 + float64(i)*h
		c, s := math.Cos(phi), math.Sin(phi)
		sum += c * c * math.Sqrt(16-9*s*s)
	}
	return 36 * sum * h
}

// regroupCylinderZ is t3: a circle r=3 at (20, 10) on XY, extruded 20 along.
func regroupCylinderZ(t *testing.T, sc *retentionScene) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(20, 10)
	s.Fix(c)
	s.CreateCircle(c, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// regroupSlabC is C: a rectangle x 18..26, y 2..18 on XY, extruded 20 along.
func regroupSlabC(t *testing.T, sc *retentionScene) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(18, 2, 26, 18)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func requireRegroupResult(t *testing.T, got *decad.Body, want float64) {
	t.Helper()
	require.True(t, tookMeshPath(got), "the one mesh step is honest")
	requireBodyWatertight(t, got)
	v, err := got.Volume()
	require.NoError(t, err)
	mag := volumeMM(t, v)
	bound := v.Bound.Mag()
	require.InDelta(t, want, mag, bound+1e-6, "volume %v bound %v", mag, bound)
	require.Less(t, bound, 1e-3*want, "the volume bound stays usable")
}

func TestRegroupPlansBuildByHand(t *testing.T) {
	t.Parallel()

	t.Run("F1 two overlapping parallel holes", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		first, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		// A Copy ends the history chain, so the final call is the sequential gate, not a regroup.
		held, err := decad.Copy(t.Context(), first)
		require.NoError(t, err)
		_, err = decad.Cut(t.Context(), held, slabYTool(t, sc, 22, 3))
		require.ErrorIs(t, err, decad.ErrUnsupported)

		sc = newRetentionScene()
		box, t1, t2 := sc.roundedBox(t), sc.tool(t, 20), slabYTool(t, sc, 22, 3)
		u, err := decad.Union(t.Context(), t1, t2)
		require.NoError(t, err)
		require.False(t, tookMeshPath(u), "Union(t1, t2) is analytic")
		got, err := decad.Cut(t.Context(), box, u)
		require.NoError(t, err)
		requireRegroupResult(t, got, regroupBoxVolume-20*(25*math.Pi-regroupLens(4, 3, 2)))
	})

	t.Run("F2 a straight hole into an earlier hole", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		first, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		// A Copy ends the history chain, so the final call is the sequential gate, not a regroup.
		held, err := decad.Copy(t.Context(), first)
		require.NoError(t, err)
		_, err = decad.Cut(t.Context(), held, regroupCylinderZ(t, sc))
		require.ErrorIs(t, err, decad.ErrUnsupported)

		sc = newRetentionScene()
		box, t1, t3 := sc.roundedBox(t), sc.tool(t, 20), regroupCylinderZ(t, sc)
		step, err := decad.Cut(t.Context(), box, t3)
		require.NoError(t, err)
		require.False(t, tookMeshPath(step), "Cut(box, t3) is analytic")
		got, err := decad.Cut(t.Context(), step, t1)
		require.NoError(t, err)
		vx := regroupCrossVolume(4096)
		require.InDelta(t, vx, regroupCrossVolume(2048), 1e-12)
		requireRegroupResult(t, got, regroupBoxVolume-320*math.Pi-180*math.Pi+vx)
	})

	t.Run("F3 a box beside a plate carrying a boss", func(t *testing.T) {
		t.Parallel()
		build := func(t *testing.T) (plate, box2, boss *decad.Body) {
			doc := decad.New(decad.ExactOnly())
			plate = boxBody(t, doc, 0, 0, 40, 20, 10)
			box2 = boxBody(t, doc, 40, 0, 60, 20, 10)
			boss = translated(t, boxBodyAtZ(t, doc, 10, 8, 14, 12, 5, 20), 0.125, 0, 0)
			return plate, box2, boss
		}
		plate, box2, boss := build(t)
		first, err := decad.Union(t.Context(), plate, boss)
		require.NoError(t, err)
		_, err = decad.Union(t.Context(), first, box2)
		require.ErrorIs(t, err, decad.ErrUnsupported)

		plate, box2, boss = build(t)
		step, err := decad.Union(t.Context(), plate, box2)
		require.NoError(t, err)
		require.False(t, tookMeshPath(step), "Union(plate, box2) is analytic")
		got, err := decad.Union(t.Context(), step, boss)
		require.NoError(t, err)
		requireRegroupResult(t, got, 12240)
	})

	t.Run("F4 an analytic intersect after a mesh cut", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		first, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		// A Copy ends the history chain, so the final call is the sequential gate, not a regroup.
		held, err := decad.Copy(t.Context(), first)
		require.NoError(t, err)
		_, err = decad.Intersect(t.Context(), held, regroupSlabC(t, sc))
		require.ErrorIs(t, err, decad.ErrUnsupported)

		sc = newRetentionScene()
		box, t1, c := sc.roundedBox(t), sc.tool(t, 20), regroupSlabC(t, sc)
		step, err := decad.Intersect(t.Context(), box, c)
		require.NoError(t, err)
		require.False(t, tookMeshPath(step), "Intersect(box, C) is analytic")
		got, err := decad.Cut(t.Context(), step, t1)
		require.NoError(t, err)
		requireRegroupResult(t, got, 2560-16*(32*math.Pi/3+4*math.Sqrt(3)))
	})

	t.Run("D1 a Copy keeps the sequential refusal", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		first, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		cp, err := decad.Copy(t.Context(), first)
		require.NoError(t, err)
		_, err = decad.Cut(t.Context(), cp, slabYTool(t, sc, 22, 3))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	t.Run("D4 a tilted second hole stays on the mesh path", func(t *testing.T) {
		t.Parallel()
		tilted := func(t *testing.T, sc *retentionScene) *decad.Body {
			tr, err := r3.RotationAround(r3.NewVec(22, 10, 10), r3.NewVec(1, 0, 0), units.Radians(0.01))
			require.NoError(t, err)
			b, err := decad.Transform(t.Context(), slabYTool(t, sc, 22, 3), tr)
			require.NoError(t, err)
			return b
		}
		sc := newRetentionScene()
		first, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		// A Copy ends the history chain, so the final call is the sequential gate, not a regroup.
		held, err := decad.Copy(t.Context(), first)
		require.NoError(t, err)
		_, err = decad.Cut(t.Context(), held, tilted(t, sc))
		require.ErrorIs(t, err, decad.ErrUnsupported)

		sc = newRetentionScene()
		u, err := decad.Union(t.Context(), sc.tool(t, 20), tilted(t, sc))
		require.NoError(t, err)
		require.True(t, tookMeshPath(u), "Union(t1, t2') takes the mesh path")
	})
}
