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

// The plate and circular boss both come from real Sketch profiles. The
// transition from the ledge to the inner cylinder is one quarter torus.
func TestShellRoundStackedBoss(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		cx, cy float64
	}{
		{"centered", 10, 10},
		{"offset", 11, 9},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			stack := roundBossFromSketch(t, doc, tc.cx, tc.cy)
			require.Len(t, stack.Faces(), 8)
			shell, err := stack.Shell(t.Context(),
				decad.Faces(decad.FaceCreatedBy(decad.CapEnd(stack))).Exactly(1), units.Millimeters(1))
			require.NoError(t, err)
			require.Len(t, shell.Faces(), 16)
			volume, err := shell.Volume()
			require.NoError(t, err)
			want := 1028 + 76*math.Pi/3 - 3*math.Pi*math.Pi/2
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base())
			centroid, err := shell.Centroid()
			require.NoError(t, err)
			bossNet := want - 1028
			require.LessOrEqual(t,
				math.Abs(centroid.Value.X-(1028*10+bossNet*tc.cx)/want), centroid.Bound.Base())
			require.LessOrEqual(t,
				math.Abs(centroid.Value.Y-(1028*10+bossNet*tc.cy)/want), centroid.Bound.Base())
			wantZ := (2570 + 2605*math.Pi/12 - 6*math.Pi*math.Pi) / want
			require.InDelta(t, wantZ, centroid.Value.Z, 1e-12)
			var tori int
			for _, face := range shell.Faces() {
				if face.Surface().Kind() == decad.KindTorus {
					tori++
				}
			}
			require.Equal(t, 1, tori)
			mesh, err := shell.Tessellate(t.Context(), units.Millimeters(0.05),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.VolumeVerified())
			require.LessOrEqual(t, math.Abs(shellMeshSignedVolume(mesh)-volume.Value.Base()),
				1+volume.Bound.Base())
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)
			movedBy, err := r3.Translation(r3.NewVec(3, -2, 4))
			require.NoError(t, err)
			moved, err := shell.Placed(t.Context(), movedBy)
			require.NoError(t, err)
			movedVolume, err := moved.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(movedVolume.Value.Base()-want), movedVolume.Bound.Base())
			movedMesh, err := moved.Tessellate(t.Context(), units.Millimeters(0.05),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, movedMesh.VolumeVerified())
			if tc.name == "centered" {
				mirror, err := r3.NewFrame(r3.NewVec(0, 0, 0),
					r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
				require.NoError(t, err)
				reflection, err := r3.Reflection(mirror)
				require.NoError(t, err)
				reflected, err := moved.Placed(t.Context(), reflection)
				require.NoError(t, err)
				reflectedMesh, err := reflected.Tessellate(t.Context(), units.Millimeters(0.05),
					decad.WithVerification(decad.VerifyAll))
				require.NoError(t, err)
				require.True(t, reflectedMesh.VolumeVerified())
				require.LessOrEqual(t, math.Abs(shellMeshSignedVolume(reflectedMesh)-want), 1.0)
			}
		})
	}
}

func TestShellRoundStackedBossClearance(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name              string
		cx, cy, thickness float64
	}{
		{"floor", 10, 10, 2.5},
		{"root clearance", 15, 10, 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			stack := roundBossFromSketch(t, doc, tc.cx, tc.cy)
			_, err := stack.Shell(t.Context(),
				decad.Faces(decad.FaceCreatedBy(decad.CapEnd(stack))).Exactly(1),
				units.Millimeters(tc.thickness))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			_, err = stack.Volume()
			require.NoError(t, err, "a refused Shell leaves the source live")
		})
	}
}

func roundBossFromSketch(t *testing.T, doc *decad.Document, cx, cy float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 20, 20)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	plate, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	plane, err = w.CreateOffsetPlane(w.XY(), 5)
	require.NoError(t, err)
	s, err = w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(cx, cy)
	s.Fix(c)
	s.CreateCircle(c, 4)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	boss, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	stack, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	return stack
}
