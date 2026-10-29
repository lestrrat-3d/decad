package decad

import (
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestMirrorFaceStatedAxialDelta checks that a face whose plane states an
// AxialDelta mirrors only inside the arithmetic allowance: a larger delta puts
// the image up to twice that off the true reflection, with no bound to carry it.
//
// Shown to fail with the AxialDelta gate in resolveMirrorFace removed: the 1e-6
// case then mirrors instead of reading ErrUnsupported.
func TestMirrorFaceStatedAxialDelta(t *testing.T) {
	t.Parallel()
	setup := func(t *testing.T, delta float64) (*Body, MirrorFace) {
		t.Helper()
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		corners := [][2]float64{{0, 0}, {20, 0}, {20, 10}, {0, 10}}
		points := make([]*sketch.Point, len(corners))
		for i, c := range corners {
			points[i] = s.CreatePoint(c[0], c[1])
			s.Fix(points[i])
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		body, err := Extrude(t.Context(), New(), s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along})
		require.NoError(t, err)
		sel := Faces(FaceCreatedBy(CapEnd(body)))
		faces, err := sel.SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces, 1)
		faces[0].n.HasAxialDelta, faces[0].n.AxialDelta = true, delta
		return body, MirrorFace{Body: body, Face: sel}
	}

	t.Run("a stated displacement is unsupported", func(t *testing.T) {
		t.Parallel()
		body, plane := setup(t, 1e-6)
		_, err := MirroredCopy(t.Context(), body, plane)
		require.ErrorIs(t, err, ErrUnsupported)
	})
	t.Run("a unit-conversion sized delta still mirrors", func(t *testing.T) {
		t.Parallel()
		body, plane := setup(t, 2.9e-16)
		img, err := MirroredCopy(t.Context(), body, plane)
		require.NoError(t, err)
		box, err := img.Bounds()
		require.NoError(t, err)
		require.InDelta(t, 10, box.Min.Z, 1e-9)
		require.InDelta(t, 20, box.Max.Z, 1e-9)
	})
}
