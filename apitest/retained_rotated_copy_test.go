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

// The fixture of agent-docs/faceted-source-retention-design.md §1 (P2): a
// filleted box cut by a placed tool, then copied by an exact rotation.

func rotatedRetentionCopy(t *testing.T, degrees float64) (*decad.Body, *decad.Body, r3.Transform) {
	t.Helper()
	_, drilled, copied, turn := rotatedRetentionScene(t, degrees)
	return drilled, copied, turn
}

// rotatedRetentionScene is rotatedRetentionCopy with the scene that owns both
// bodies, so a later Extrude can name the copy as its stop.
func rotatedRetentionScene(t *testing.T, degrees float64) (*retentionScene, *decad.Body, *decad.Body, r3.Transform) {
	t.Helper()
	sc := newRetentionScene()
	drilled, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
	require.NoError(t, err)
	center, axis := r3.NewVec(20, 10, 0), r3.NewVec(0, 0, 1)
	copied, err := decad.Copy(t.Context(), drilled, decad.WithCopyRotation(center, axis, units.Degrees(degrees)))
	require.NoError(t, err)
	turn, err := r3.RotationAround(center, axis, units.Degrees(degrees))
	require.NoError(t, err)
	return sc, drilled, copied, turn
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
			// A rotation about z fixes the +z and -z cap planes exactly, so those
			// tags keep bound 0. Every other plane image is a rounded float one.
			if math.Abs(n.Value.Z) > 1-1e-9 {
				require.Zero(t, bound, "a cap plane normal to the rotation axis is its own exact image")
			} else {
				require.Positive(t, bound, "a 20 degree image is a rounded float image of the exact one")
			}
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

	t.Run("ToFace stops exactly at the top cap of a rotated copy", func(t *testing.T) {
		t.Parallel()
		sc, _, copied, _ := rotatedRetentionScene(t, 20)
		top := decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(0, 0, 1)))
		sk := rotatedCopySketch(t, sc.w)
		pin, err := decad.Extrude(t.Context(), sc.doc, sk, sk.Profiles()[0], decad.ToFace{Body: copied, Face: top})
		require.NoError(t, err)
		box, err := pin.Bounds()
		require.NoError(t, err)
		require.Equal(t, 20.0, box.Max.Z, "the pin ends on the cap's plane z=20")
		require.Zero(t, box.Min.Z)
		vol, err := pin.Volume()
		require.NoError(t, err)
		require.InDelta(t, 35*35*20, vol.Value.Base(), vol.Bound.Base()+1e-9)
	})

	t.Run("MirrorFace across the top cap of a rotated copy is exact", func(t *testing.T) {
		t.Parallel()
		_, copied, _ := rotatedRetentionCopy(t, 20)
		top := decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(0, 0, 1)))
		mirrored, err := decad.MirroredCopy(t.Context(), copied, decad.MirrorFace{Body: copied, Face: top})
		require.NoError(t, err)
		src, err := copied.Bounds()
		require.NoError(t, err)
		got, err := mirrored.Bounds()
		require.NoError(t, err)
		require.InDelta(t, 2*20-src.Max.Z, got.Min.Z, 1e-9)
		require.InDelta(t, 2*20-src.Min.Z, got.Max.Z, 1e-9)
		want, err := copied.Volume()
		require.NoError(t, err)
		vol, err := mirrored.Volume()
		require.NoError(t, err)
		require.InDelta(t, want.Value.Base(), vol.Value.Base(), want.Bound.Base()+vol.Bound.Base())
	})

	t.Run("a tilted rotation still refuses ToFace and MirrorFace", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		drilled, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		copied, err := decad.Copy(t.Context(), drilled,
			decad.WithCopyRotation(r3.NewVec(20, 10, 0), r3.NewVec(1, 0, 0), units.Degrees(20)))
		require.NoError(t, err)
		tilted := decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(0, -0.3420201433256687, 0.9396926207859084)))
		faces, err := tilted.SelectFaces(copied)
		require.NoError(t, err)
		require.Len(t, faces, 1)
		_, err = decad.MirroredCopy(t.Context(), copied, decad.MirrorFace{Body: copied, Face: tilted})
		require.ErrorIs(t, err, decad.ErrUnsupported)
		sk := rotatedCopySketch(t, sc.w)
		_, err = decad.Extrude(t.Context(), sc.doc, sk, sk.Profiles()[0], decad.ToFace{Body: copied, Face: tilted})
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}

// rotatedCopySketch is a square at the origin of the XY plane.
func rotatedCopySketch(t *testing.T, w *sketch.World) *sketch.Sketch {
	t.Helper()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(-30, -30, 5, 5)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s
}
