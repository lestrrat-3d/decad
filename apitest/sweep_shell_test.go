package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func straightSweepForShell(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 20, 20)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 10)})
	require.NoError(t, err)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, s.Profiles()[0], path)
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, body, units.CubicMillimeters(4000), decadtest.Exactly())
	return doc, body
}

func TestStraightSweepShellCapOpenings(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name        string
		selectFaces func(*decad.Body) *decad.FaceQuery
		volume      float64
		placed      bool
	}{
		{"end cap", func(b *decad.Body) *decad.FaceQuery {
			return decad.Faces(decad.FaceCreatedBy(decad.CapEnd(b))).Exactly(1)
		}, 1084, false},
		{"start cap", func(b *decad.Body) *decad.FaceQuery {
			return decad.Faces(decad.FaceCreatedBy(decad.CapStart(b))).Exactly(1)
		}, 1084, false},
		{"both caps", func(*decad.Body) *decad.FaceQuery {
			return decad.Faces(decad.NormalTo(r3.NewVec(0, 0, 1))).Exactly(2)
		}, 760, false},
		{"placed end cap", func(b *decad.Body) *decad.FaceQuery {
			return decad.Faces(decad.FaceCreatedBy(decad.CapEnd(b))).Exactly(1)
		}, 1084, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, body := straightSweepForShell(t)
			if tc.placed {
				move, err := r3.Translation(r3.NewVec(3, -4, 5))
				require.NoError(t, err)
				body, err = decad.Transform(t.Context(), body, move)
				require.NoError(t, err)
			}
			result, err := body.Shell(t.Context(), tc.selectFaces(body), units.Millimeters(1))
			require.NoError(t, err)
			decadtest.MeasuresVolume(t, result, units.CubicMillimeters(tc.volume), decadtest.Exactly())
			requireSoundAndMeshed(t, doc, result, tc.volume)
		})
	}
}

func TestStraightSweepClosedShell(t *testing.T) {
	t.Parallel()
	doc, body := straightSweepForShell(t)
	result, err := body.Shell(t.Context(), nil, units.Millimeters(1), decad.WithNoOpenings())
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, result, units.CubicMillimeters(1408), decadtest.Exactly())
	require.Len(t, result.Shells(), 2)
	require.True(t, result.Shells()[1].IsVoid())
	requireSoundAndMeshed(t, doc, result, 1408)
}

func TestStraightSweepShellStillRefusesSideOpening(t *testing.T) {
	t.Parallel()
	doc, body := straightSweepForShell(t)
	_, err := body.Shell(t.Context(), decad.Faces(decad.NormalTo(r3.NewVec(1, 0, 0))), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{body}, doc.Bodies())
}
