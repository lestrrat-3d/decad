package apitest_test

import (
	"math"
	"strings"
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

func TestStraightSweepShellSideOpening(t *testing.T) {
	t.Parallel()
	doc, body := straightSweepForShell(t)
	side := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0))).Exactly(1)
	result, err := body.Shell(t.Context(), side, units.Millimeters(1))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, result, units.CubicMillimeters(1264), decadtest.Exactly())
	mesh, err := result.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	verification, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, verification.Status)
}

func TestStraightSweepShellSideOpeningCapsSenseAndPlacement(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		endCap  bool
		both    bool
		outward bool
		placed  bool
		volume  float64
	}{
		{"end cap removed", true, false, false, false, 922},
		{"both caps removed", false, true, false, false, 580},
		{"outward wall", false, false, true, false, 1544 - 6*(4-math.Pi)},
		{"translated receiver", false, false, false, true, 1264},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, body := straightSweepForShell(t)
			if tc.placed {
				move, err := r3.Translation(r3.NewVec(3, -4, 5))
				require.NoError(t, err)
				body, err = decad.Transform(t.Context(), body, move)
				require.NoError(t, err)
			}
			sel := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))
			if tc.endCap || tc.both {
				sel = sel.Or(decad.FaceCreatedBy(decad.CapEnd(body)))
			}
			if tc.both {
				sel = sel.Or(decad.FaceCreatedBy(decad.CapStart(body)))
			}
			var opts []decad.ShellOption
			if tc.outward {
				opts = append(opts, decad.WithShellSense(decad.Outward))
			}
			result, err := body.Shell(t.Context(), sel, units.Millimeters(1), opts...)
			require.NoError(t, err)
			if tc.outward {
				volume, err := result.Volume()
				require.NoError(t, err)
				require.InDelta(t, tc.volume, volume.Value.Base(), 1e-9)
			} else {
				decadtest.MeasuresVolume(t, result, units.CubicMillimeters(tc.volume), decadtest.Exactly())
			}
			mesh, err := result.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

func TestStraightSweepShellSideOpeningRefusesWholeBoundaryAndHole(t *testing.T) {
	t.Parallel()
	t.Run("all outer sides", func(t *testing.T) {
		doc, body := straightSweepForShell(t)
		allSides := decad.Faces(decad.NormalTo(r3.NewVec(1, 0, 0))).
			Or(decad.NormalTo(r3.NewVec(0, 1, 0))).Exactly(4)
		out, err := body.Shell(t.Context(), allSides, units.Millimeters(1))
		require.Nil(t, out)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "not all")
		require.Equal(t, []*decad.Body{body}, doc.Bodies())
	})
	t.Run("hole wall", func(t *testing.T) {
		world := sketch.NewWorld()
		s, err := world.CreateSketch(world.XY())
		require.NoError(t, err)
		outer := s.CreateRectangle(0, 0, 20, 20)
		s.Fix(outer.A)
		s.CreateRectangle(5, 5, 15, 15)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		var profile *sketch.Profile
		for _, candidate := range s.Profiles() {
			if candidate.Valid && len(candidate.Holes) == 1 {
				profile = candidate
				break
			}
		}
		require.NotNil(t, profile)
		path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 10)})
		require.NoError(t, err)
		doc := decad.New()
		body, err := decad.Sweep(t.Context(), doc, s, profile, path)
		require.NoError(t, err)
		var hole *decad.Face
		for _, face := range body.Faces() {
			for _, origin := range face.Origins() {
				if strings.HasPrefix(origin.Role, "side(0,1,") {
					hole = face
					break
				}
			}
		}
		require.NotNil(t, hole)
		out, err := body.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(hole.Origins()[0])).Exactly(1),
			units.Millimeters(1))
		require.Nil(t, out)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "outer loop")
		require.Equal(t, []*decad.Body{body}, doc.Bodies())
	})
}

func TestSweepShellSideOpeningRefusesOtherPaths(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		path func(*testing.T) *decad.Path
	}{
		{"curved path", mustPathArc},
		{"two lines", func(t *testing.T) *decad.Path {
			return mustPath(t, r3.Vec{}, r3.NewVec(0, 0, 5), r3.NewVec(0, 0, 10))
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, profile := plateSketch(t)
			doc := decad.New()
			body, err := decad.Sweep(t.Context(), doc, s, profile, tc.path(t))
			require.NoError(t, err)
			var side *decad.Face
			for _, face := range body.Faces() {
				for _, origin := range face.Origins() {
					if strings.HasPrefix(origin.Role, "side(") {
						side = face
						break
					}
				}
			}
			require.NotNil(t, side)
			out, err := body.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(side.Origins()[0])).Exactly(1),
				units.Millimeters(1))
			require.Nil(t, out)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Equal(t, []*decad.Body{body}, doc.Bodies())
		})
	}
}
