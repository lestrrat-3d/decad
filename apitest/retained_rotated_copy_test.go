package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixture of agent-docs/faceted-source-retention-design.md §1 (P2): a
// filleted box cut by a placed tool, then copied by an exact rotation.

func rotatedRetentionCopy(t *testing.T, degrees float64) (*decad.Body, *decad.Body, r3.Transform) {
	t.Helper()
	sc := newRetentionScene()
	drilled, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
	require.NoError(t, err)
	center, axis := r3.NewVec(20, 10, 0), r3.NewVec(0, 0, 1)
	copied, err := decad.Copy(t.Context(), drilled, decad.WithCopyRotation(center, axis, units.Degrees(degrees)))
	require.NoError(t, err)
	turn, err := r3.RotationAround(center, axis, units.Degrees(degrees))
	require.NoError(t, err)
	return drilled, copied, turn
}

func TestRetainedRotatedCopy(t *testing.T) {
	t.Parallel()

	t.Run("a 20 degree copy keeps planes and cylinders at the rotated directions", func(t *testing.T) {
		t.Parallel()
		_, copied, turn := rotatedRetentionCopy(t, 20)
		kinds := retentionKinds(copied)
		require.Zero(t, kinds["faceted"], "kinds %v", kinds)
		require.Equal(t, 6, kinds["plane"], "kinds %v", kinds)
		require.Equal(t, 5, kinds["cylinder"], "kinds %v", kinds)

		axes := []r3.Vec{{X: 1}, {X: -1}, {Y: 1}, {Y: -1}, {Z: 1}, {Z: -1}}
		matched := map[int]int{}
		blends, holes := 0, 0
		for _, f := range copied.Faces() {
			p := firstVertex(t, f)
			n, err := f.NormalAt(p)
			require.NoError(t, err)
			bound := n.Bound.Base()
			require.False(t, math.IsNaN(bound) || math.IsInf(bound, 0), "bound %v", bound)
			require.Less(t, bound, 1.0)
			require.Positive(t, bound, "a 20 degree image is a rounded float image of the exact one")
			switch s := f.Surface().(type) {
			case decad.Plane:
				// The tag's normal is the face normal up to orientation, and
				// the reported normal is a rotated box axis within its bound.
				require.InDelta(t, 1, math.Abs(s.Frame.N().Dot(n.Value)), 1e-9)
				found := false
				for i, a := range axes {
					if n.Value.Sub(turn.ApplyDir(a)).Len() <= bound+1e-9 {
						matched[i]++
						found = true
					}
				}
				require.True(t, found, "plane normal %v matches no rotated box axis", n.Value)
			case decad.Cylinder:
				require.InDelta(t, 1, s.Axis.Len(), 1e-9)
				switch {
				case s.Axis.Cross(turn.ApplyDir(r3.Vec{Z: 1})).Len() < 1e-9:
					blends++
				case s.Axis.Cross(turn.ApplyDir(r3.Vec{Y: 1})).Len() < 1e-9:
					holes++
				default:
					require.Fail(t, "cylinder axis is neither the rotated z nor the rotated y", "%v", s.Axis)
				}
			}
		}
		for i := range axes {
			require.Equal(t, 1, matched[i], "rotated axis %d matched by %d planes", i, matched[i])
		}
		require.Equal(t, 4, blends)
		require.Equal(t, 1, holes)
	})

	t.Run("the copy keeps the source volume within its bounds", func(t *testing.T) {
		t.Parallel()
		src, copied, _ := rotatedRetentionCopy(t, 20)
		want, err := src.Volume()
		require.NoError(t, err)
		got, err := copied.Volume()
		require.NoError(t, err)
		slack := want.Bound.Base() + got.Bound.Base()
		require.LessOrEqual(t, math.Abs(got.Value.Base()-want.Value.Base()), slack)
	})

	t.Run("a 90 degree copy keeps its tags with usable bounds", func(t *testing.T) {
		t.Parallel()
		_, copied, _ := rotatedRetentionCopy(t, 90)
		kinds := retentionKinds(copied)
		require.Zero(t, kinds["faceted"], "kinds %v", kinds)
		for _, f := range copied.Faces() {
			n, err := f.NormalAt(firstVertex(t, f))
			require.NoError(t, err)
			require.Less(t, n.Bound.Base(), 1.0)
		}
	})
}
