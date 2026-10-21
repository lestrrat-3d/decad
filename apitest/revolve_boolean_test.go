package apitest_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// A shared axial disk is a positive-area face contact whose two source
// tessellations need not share chords. The meridian arrangement decides it.
func TestCommonAxisRevolveUnionSharesAxialDisk(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	left := revolveMeridian(t, doc, [][2]float64{{0, 0}, {4, 0}, {4, 2}, {0, 2}}, decad.FullRevolution{})
	right := revolveMeridian(t, doc, [][2]float64{{4, 0}, {6, 0}, {6, 3}, {4, 3}}, decad.FullRevolution{})
	sourceFaces := [][]*decad.Face{left.Faces(), right.Faces()}

	joined, err := decad.Union(t.Context(), left, right)
	require.NoError(t, err)
	requireAnalyticVolume(t, joined, 34*math.Pi)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	for _, source := range sourceFaces {
		for _, face := range source {
			if face.Surface().Kind() != decad.KindCylinder {
				continue
			}
			found, err := decad.Faces(decad.FaceCreatedBy(face.Origins()[0])).AtLeast(1).SelectFaces(joined)
			require.NoError(t, err)
			require.NotEmpty(t, found)
		}
	}
}

// The same planar arrangement also handles a shoulder against a larger
// coaxial cylinder. All three public operations keep their volume proof.
func TestCommonAxisRevolveSteppedBooleanOperations(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		want float64
		run  func(context.Context, *decad.Body, *decad.Body) (*decad.Body, error)
	}{
		{name: "common axis union", want: 84 * math.Pi, run: func(ctx context.Context, a, b *decad.Body) (*decad.Body, error) {
			return decad.Union(ctx, a, b)
		}},
		{name: "common axis cut", want: 36 * math.Pi, run: pairCut},
		{name: "common axis intersect", want: 8 * math.Pi, run: pairIntersect},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			shaft := revolveMeridian(t, doc,
				[][2]float64{{0, 0}, {6, 0}, {6, 2}, {4, 2}, {4, 3}, {0, 3}}, decad.FullRevolution{})
			large := revolveMeridian(t, doc,
				[][2]float64{{4, 0}, {7, 0}, {7, 4}, {4, 4}}, decad.FullRevolution{})
			result, err := tc.run(t.Context(), shaft, large)
			require.NoError(t, err)
			requireAnalyticVolume(t, result, tc.want)
			mesh, err := result.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

// A tube and a solid cylinder meet across only part of an annular cap. The
// connected result has one blind cavity, so its meridian remains one loop.
func TestCommonAxisRevolveUnionMakesBlindCavity(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	tube := revolveMeridian(t, doc, [][2]float64{{0, 2}, {4, 2}, {4, 4}, {0, 4}}, decad.FullRevolution{})
	end := revolveMeridian(t, doc, [][2]float64{{4, 0}, {6, 0}, {6, 3}, {4, 3}}, decad.FullRevolution{})
	result, err := decad.Union(t.Context(), tube, end)
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, result, units.CubicMillimeters((4*(16-4)+2*9)*math.Pi))
	require.Len(t, result.Lumps(), 1)
	mesh, err := result.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestCommonAxisRevolveUnionKeepsBothCoincidentWallSources(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	profile := [][2]float64{{0, 0}, {4, 0}, {4, 2}, {0, 2}}
	a := revolveMeridian(t, doc, profile, decad.FullRevolution{})
	b := revolveMeridian(t, doc, profile, decad.FullRevolution{})
	wallOrigins := make([]decad.FeatureRef, 0, 2)
	for _, body := range []*decad.Body{a, b} {
		for _, face := range body.Faces() {
			if face.Surface().Kind() == decad.KindCylinder {
				wallOrigins = append(wallOrigins, face.Origins()[0])
			}
		}
	}
	require.Len(t, wallOrigins, 2)
	result, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	requireAnalyticVolume(t, result, 16*math.Pi)
	for _, origin := range wallOrigins {
		faces, err := decad.Faces(decad.FaceCreatedBy(origin)).AtLeast(1).SelectFaces(result)
		require.NoError(t, err)
		require.NotEmpty(t, faces)
	}
	mesh, err := result.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestCommonAxisRevolveBooleanRefusalsLeaveInputsLive(t *testing.T) {
	t.Parallel()
	t.Run("edge contact", func(t *testing.T) {
		doc := decad.New()
		left := revolveMeridian(t, doc, [][2]float64{{0, 0}, {4, 0}, {4, 2}, {0, 2}}, decad.FullRevolution{})
		ring := revolveMeridian(t, doc, [][2]float64{{4, 2}, {6, 2}, {6, 3}, {4, 3}}, decad.FullRevolution{})
		before := doc.Bodies()
		_, err := decad.Union(t.Context(), left, ring)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		var booleanErr *decad.BooleanError
		require.ErrorAs(t, err, &booleanErr)
		require.Equal(t, decad.BooleanUnsupportedContact, booleanErr.Code)
		require.Equal(t, before, doc.Bodies())
	})
	t.Run("empty intersection", func(t *testing.T) {
		doc := decad.New()
		a := revolveMeridian(t, doc, [][2]float64{{0, 0}, {4, 0}, {4, 2}, {0, 2}}, decad.FullRevolution{})
		b := revolveMeridian(t, doc, [][2]float64{{4, 0}, {6, 0}, {6, 3}, {4, 3}}, decad.FullRevolution{})
		before := doc.Bodies()
		_, err := decad.Intersect(t.Context(), a, b)
		require.ErrorIs(t, err, decad.ErrBooleanFailed)
		var booleanErr *decad.BooleanError
		require.ErrorAs(t, err, &booleanErr)
		require.Equal(t, decad.BooleanEmpty, booleanErr.Code)
		require.Equal(t, before, doc.Bodies())
	})
	t.Run("near contact", func(t *testing.T) {
		doc := decad.New()
		a := revolveMeridian(t, doc, [][2]float64{{0, 0}, {4, 0}, {4, 2}, {0, 2}}, decad.FullRevolution{})
		b := revolveMeridian(t, doc, [][2]float64{{4.00001, 0}, {6, 0}, {6, 3}, {4.00001, 3}},
			decad.FullRevolution{})
		before := doc.Bodies()
		result, err := decad.Union(t.Context(), a, b)
		if err != nil {
			require.True(t, errors.Is(err, decad.ErrUnsupported) || errors.Is(err, decad.ErrBooleanFailed))
			require.Equal(t, before, doc.Bodies())
		} else {
			require.Len(t, result.Lumps(), 2)
		}
	})
	t.Run("sub-band gap", func(t *testing.T) {
		doc := decad.New()
		a := revolveMeridian(t, doc, [][2]float64{{0, 0}, {4, 0}, {4, 2}, {0, 2}}, decad.FullRevolution{})
		b := revolveMeridian(t, doc,
			[][2]float64{{4.0000000000005, 0}, {6, 0}, {6, 3}, {4.0000000000005, 3}},
			decad.FullRevolution{})
		before := doc.Bodies()
		result, err := decad.Union(t.Context(), a, b)
		if err != nil {
			require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
			require.Equal(t, before, doc.Bodies())
		} else {
			require.Len(t, result.Lumps(), 2)
		}
	})
	t.Run("foreign source", func(t *testing.T) {
		docA, docB := decad.New(), decad.New()
		a := revolveMeridian(t, docA, [][2]float64{{0, 0}, {4, 0}, {4, 2}, {0, 2}}, decad.FullRevolution{})
		b := revolveMeridian(t, docB, [][2]float64{{4, 0}, {6, 0}, {6, 3}, {4, 3}}, decad.FullRevolution{})
		beforeA, beforeB := docA.Bodies(), docB.Bodies()
		_, err := decad.Union(t.Context(), a, b)
		require.ErrorIs(t, err, decad.ErrForeignBody)
		require.Equal(t, beforeA, docA.Bodies())
		require.Equal(t, beforeB, docB.Bodies())
	})
	t.Run("canceled", func(t *testing.T) {
		doc := decad.New()
		a := revolveMeridian(t, doc, [][2]float64{{0, 0}, {4, 0}, {4, 2}, {0, 2}}, decad.FullRevolution{})
		b := revolveMeridian(t, doc, [][2]float64{{4, 0}, {6, 0}, {6, 3}, {4, 3}}, decad.FullRevolution{})
		before := doc.Bodies()
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err := decad.Union(ctx, a, b)
		require.ErrorIs(t, err, context.Canceled)
		require.Equal(t, before, doc.Bodies())
	})
}
