package clearance_test

import (
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/clearance/facepair"
	"github.com/lestrrat-3d/decad/internal/clearance/tier"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// steepCone is the cone carrier BuildRevolveCarriers sweeps from the wall
// (z0, 0) → (z0 + a·k, b·k) a full turn about the world x axis: its apex sits
// on the axis at x = z0 and its slope is b/a.
func steepCone(t *testing.T, z0, k, a, b float64) *clearance.CFace {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	seg := survey2d.SegmentWalk{
		Kind: survey2d.WalkLine, StartU: z0, StartV: 0, EndU: z0 + a*k, EndV: b * k,
		TanInU: a, TanInV: b, TanOutU: a, TanOutV: b,
	}
	built := clearance.BuildRevolveCarriers(clearance.RevolveCarrierInput{
		Lift:      revolvemesh.RevolveLift{Frame: frame, DU: 1},
		Transform: r3.Identity(),
		Full:      true,
		Walls: []clearance.RevolveWall{{
			Walk: survey2d.SideWalk{SegmentWalk: seg, Segs: []int{0}},
			Kind: revolveaxis.WallCone, First: seg, Last: seg,
		}},
	})
	require.Len(t, built.Faces, 1)
	require.Zero(t, built.AxisGap, "an integer wall about a coordinate axis is its own record")
	return built.Faces[0]
}

// TestConeCellsReadTheSlope pins how the two closed-form cone cells read the
// carrier: through the slope b/a of the walk it was swept from, never through
// a float half angle. The cone sweeps (2²⁰, 0) → (2²⁰ + a·2¹⁶, b·2¹⁶) about
// the x axis, with a² + b² = c², so atan(b/a) is no rational multiple of π.
// The point P sits c along the wall's outward normal (−b, a)/c from the wall
// point at slant c·2¹⁵, so it lies exactly c from the cone and the ball of
// radius 1/4 about it exactly c − 1/4. Both readings are closed form and
// marked exact, so each must equal its truth to the last bit: every product
// of the meridian reading is an integer and c divides the sum exactly.
//
// Shown to fail: with the cells reading math.Sincos of the carrier's
// math.Atan2 half angle, the cone × sphere cell read a = 3, b = 4 as
// 4.7500000000145519 and the vertex × cone tier read 5.0000000000145519,
// one unit in the last place of the products, about 1.5e-11 off.
func TestConeCellsReadTheSlope(t *testing.T) {
	t.Parallel()
	const z0, k = 1048576.0, 65536.0
	for _, tc := range []struct{ a, b, c float64 }{{3, 4, 5}, {5, 12, 13}, {8, 15, 17}} {
		t.Run(fmt.Sprintf("slope=%v/%v", tc.b, tc.a), func(t *testing.T) {
			t.Parallel()
			cone := steepCone(t, z0, k, tc.a, tc.b)
			const m = k / 2
			p := r3.NewVec(z0+tc.a*m-tc.b, tc.b*m+tc.a, 0)
			tol := 1e-9 * (z0 + tc.a*k)

			sink := &clearance.CellSink{}
			fk := facepair.New(t.Context(), tol, tol)
			fk.FaceCell(cone, sphereFace(p, 0.25), sink)
			require.NoError(t, fk.Err())
			lo, hi, exact, ok := sink.Interval()
			require.True(t, ok)
			require.True(t, exact, "the cone × sphere cell is closed form")
			require.Equal(t, tc.c-0.25, lo, "cone × sphere reads the exact gap")
			require.Equal(t, lo, hi)

			sink = &clearance.CellSink{}
			tk := tier.New(t.Context(), tol, tol)
			require.NoError(t, tk.VertexFace(proofbound.NewWorkBudget(t.Context()), p, cone, sink))
			lo, hi, exact, ok = sink.Interval()
			require.True(t, ok)
			require.True(t, exact, "the vertex × cone tier is closed form")
			require.Equal(t, tc.c, lo, "vertex × cone reads the exact distance")
			require.Equal(t, lo, hi)
		})
	}
}

// sphereFace is a whole sphere of radius r about centre.
func sphereFace(centre r3.Vec, r float64) *clearance.CFace {
	axis := r3.NewVec(0, 0, 1)
	u := clearance.PerpTo(axis)
	return &clearance.CFace{
		Kind:   clearance.CkSphere,
		Anchor: centre,
		Axis:   axis,
		RefU:   u,
		RefV:   axis.Cross(u),
		Radius: r,
		Sweep:  clearance.AngWindow{Full: true},
		Merid:  clearance.AngWindow{Full: true},
		Box:    [2]r3.Vec{centre.Sub(r3.NewVec(r, r, r)), centre.Add(r3.NewVec(r, r, r))},
		Wit:    []r3.Vec{centre.Add(u.Scale(r))},
	}
}
