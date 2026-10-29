package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// bevelledPlate is the 45 degree bevelled plate of the retained-blend tests: the 40x20x20 plate with its y=0, z=20
// edge cut from (y=0, z=18) to (y=2, z=20) by a rotated tool. Its volume is 15920.
func bevelledPlate(t *testing.T) *decad.Body {
	t.Helper()
	return slicedPlate(t, func(sc *retentionScene) *sketch.Plane { return sc.w.YZ() },
		[4]float64{-50, -50, 2, 50}, 100, rotation(t, r3.NewVec(0, 2, 20), r3.NewVec(1, 0, 0), -math.Pi/4))
}

// faceNormal is the face's normal at the mean of its outer loop's distinct corners, which lies inside a convex face.
func faceNormal(t *testing.T, f *decad.Face) r3.Vec {
	t.Helper()
	var sum r3.Vec
	seen := map[r3.Vec]struct{}{}
	for _, e := range f.Loops()[0].Edges() {
		for _, v := range []*decad.Vertex{e.Start(), e.End()} {
			if _, ok := seen[v.Position().Value]; !ok {
				seen[v.Position().Value] = struct{}{}
				sum = sum.Add(v.Position().Value)
			}
		}
	}
	n, err := f.NormalAt(sum.Scale(1 / float64(len(seen))))
	require.NoError(t, err)
	return n.Value
}

func TestTrimGraphClosedShell(t *testing.T) {
	t.Parallel()

	t.Run("bevelled plate", func(t *testing.T) {
		t.Parallel()
		beveled := bevelledPlate(t)
		shelled, err := decad.Shell(t.Context(), beveled, nil, units.Millimeters(1), decad.WithNoOpenings())
		require.NoError(t, err)

		require.Len(t, shelled.Lumps(), 1)
		shells := shelled.Shells()
		require.Len(t, shells, 2)
		require.False(t, shells[0].IsVoid())
		require.True(t, shells[1].IsVoid())
		void := shells[1].Faces()
		require.Len(t, void, 7)

		// Each void face is a Plane whose normal points opposite its outer face's.
		outer := shells[0].Faces()
		require.Len(t, outer, 7)
		for _, vf := range void {
			_, isPlane := vf.Surface().(decad.Plane)
			require.True(t, isPlane)
			vn := faceNormal(t, vf)
			matches := 0
			for _, of := range outer {
				if on := faceNormal(t, of); vn.Dot(on) < -1+1e-9 {
					matches++
				}
			}
			require.Equal(t, 1, matches, "void face with normal %v", vn)
		}

		// V = 15920 - 38 * (324 - (2 - 2 + sqrt 2)^2 / 2) = 15920 - 38 * 323.
		vol, err := shelled.Volume()
		require.NoError(t, err)
		want := new(big.Rat).SetInt64(15920 - 38*323)
		held := new(big.Rat).SetFloat64(vol.Value.Base())
		bound := new(big.Rat).SetFloat64(vol.Bound.Base())
		require.Positive(t, vol.Bound.Base())
		require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, want)).Cmp(bound), 0,
			"volume %v +- %v, want %v", vol.Value.Base(), vol.Bound.Base(), want)

		mesh, err := shelled.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())

		report, err := shelled.Document().Verify(t.Context())
		require.NoError(t, err)
		reading, err := report.ForBody(shelled)
		require.NoError(t, err)
		require.Equal(t, decad.Sound, reading.Status)
	})

	t.Run("a dropped bevel face", func(t *testing.T) {
		t.Parallel()
		_, err := decad.Shell(t.Context(), bevelledPlate(t), nil, units.Millimeters(4), decad.WithNoOpenings())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "(trim TG6)")
	})

	t.Run("outward stays refused", func(t *testing.T) {
		t.Parallel()
		_, err := decad.Shell(t.Context(), bevelledPlate(t), nil, units.Millimeters(1), decad.WithNoOpenings(),
			decad.WithShellSense(decad.Outward))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "modify-reach SX9")
	})

	t.Run("a concave receiver stays refused", func(t *testing.T) {
		t.Parallel()
		// A 4 mm groove tilted 20 degrees about x: its two floor edges are concave.
		grooved := slicedPlate(t, func(sc *retentionScene) *sketch.Plane { return sc.w.YZ() },
			[4]float64{8, 14, 12, 50}, 100, rotation(t, r3.NewVec(0, 10, 20), r3.NewVec(1, 0, 0), math.Pi/9))
		_, err := decad.Shell(t.Context(), grooved, nil, units.Millimeters(1), decad.WithNoOpenings())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "modify-reach SX9")
	})
}
