package apitest_test

import (
	"context"
	"errors"
	"math"
	"reflect"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// pointLoftSliceSheet cuts a radial tab fan parallel to its far profile.
// A world-XY Sketch line and two-sided chain Extrude make the finite sheet.
func pointLoftSliceSheet(t *testing.T, doc *decad.Document, offset, halfWidth float64) *decad.Body {
	return pointLoftSliceSheetDirected(t, doc, offset, halfWidth, false)
}

func pointLoftSliceSheetDirected(t *testing.T, doc *decad.Document,
	offset, halfWidth float64, reverse bool) *decad.Body {
	t.Helper()
	angle := math.Atan2(4, 4)
	uX, uY := math.Sin(angle), -math.Cos(angle)
	nX, nY := -math.Cos(angle), -math.Sin(angle)
	parentX := (4 / math.Sin(angle)) / math.Cos(angle)
	x, y := parentX+nX*offset, nY*offset
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	a := s.CreatePoint(x-halfWidth*uX, y-halfWidth*uY)
	b := s.CreatePoint(x+halfWidth*uX, y+halfWidth*uY)
	s.Fix(a)
	s.Fix(b)
	if reverse {
		a, b = b, a
	}
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	sheet, err := decad.Extrude(t.Context(), doc, s, s.Chains()[0], decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(30)},
		Two: decad.DistanceSide{D: units.Millimeters(30)},
	})
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, sheet.Kind())
	return sheet
}

func TestPointLoftParallelSheetReversedLineKeepsOutwardCapNormals(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "forward", true: "reverse"}[reverse], func(t *testing.T) {
			doc := decad.New()
			tooth := pointLoftRadialTab(t, doc)
			span := pointLoftSliceSpan()
			first := pointLoftSliceSheetDirected(t, doc, span/6, 30, reverse)
			pieces, err := decad.Split(t.Context(), tooth, first)
			require.NoError(t, err)
			require.Len(t, pieces, 2)
			second := pointLoftSliceSheetDirected(t, doc, span/3, 30, !reverse)
			parts, err := decad.Split(t.Context(), pieces[0], second)
			require.NoError(t, err)
			require.Len(t, parts, 2)
			away := r3.NewVec(math.Cos(math.Pi/4), math.Sin(math.Pi/4), 0)
			for _, tc := range []struct {
				body *decad.Body
				role decad.FeatureRef
				want float64
			}{
				{pieces[1], decad.CapStart(pieces[1]), -1},
				{parts[1], decad.CapStart(parts[1]), -1},
				{parts[1], decad.CapEnd(parts[1]), 1},
				{parts[0], decad.CapEnd(parts[0]), 1},
			} {
				faces, err := decad.Faces(decad.FaceCreatedBy(tc.role)).SelectFaces(tc.body)
				require.NoError(t, err)
				require.Len(t, faces, 1)
				plane, ok := faces[0].Surface().(decad.Plane)
				require.True(t, ok)
				normal, err := faces[0].NormalAt(plane.Frame.Origin())
				require.NoError(t, err)
				require.Greater(t, normal.Value.Dot(away)*tc.want, 0.99)
			}
		})
	}
}

func TestPointLoftParallelSheetFarCapFollowsApexSide(t *testing.T) {
	for _, apexX := range []float64{0, 20} {
		t.Run(map[float64]string{0: "apex_below", 20: "apex_above"}[apexX], func(t *testing.T) {
			doc := decad.New()
			world := sketch.NewWorld()
			frame, err := r3.NewFrame(r3.NewVec(10, 0, 0),
				r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
			require.NoError(t, err)
			plane, err := world.CreatePlaneFromFrame(frame)
			require.NoError(t, err)
			profileSketch, err := world.CreateSketch(plane)
			require.NoError(t, err)
			corners := [][2]float64{{-2, -2}, {2, -2}, {2, 2}, {-2, 2}}
			points := make([]*sketch.Point, len(corners))
			for i, corner := range corners {
				points[i] = profileSketch.CreatePoint(corner[0], corner[1])
				profileSketch.Fix(points[i])
			}
			for i := range points {
				profileSketch.CreateLine(points[i], points[(i+1)%len(points)])
			}
			_, err = profileSketch.Solve(t.Context())
			require.NoError(t, err)
			require.Len(t, profileSketch.Profiles(), 1)
			tooth, err := decad.LoftFromPoint(t.Context(), doc, r3.NewVec(apexX, 0, 0),
				profileSketch, profileSketch.Profiles()[0])
			require.NoError(t, err)
			farFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(tooth))).SelectFaces(tooth)
			require.NoError(t, err)
			require.Len(t, farFaces, 1)
			farCenter, err := farFaces[0].Centroid()
			require.NoError(t, err)
			require.Less(t, farCenter.Bound.Base(), 0.01)
			require.InDelta(t, 10, farCenter.Value.X, farCenter.Bound.Base()+1e-12)
			require.InDelta(t, 0, farCenter.Value.Y, farCenter.Bound.Base()+1e-12)
			require.InDelta(t, 0, farCenter.Value.Z, farCenter.Bound.Base()+1e-12)
			for _, face := range tooth.Faces() {
				if face == farFaces[0] {
					continue
				}
				_, err := face.Centroid()
				require.ErrorIs(t, err, decad.ErrUnsupported)
			}
			_, _, err = tooth.ExtremeFacesByCentroid(t.Context(), r3.NewVec(1, 0, 0))
			require.ErrorIs(t, err, decad.ErrUnsupported)

			sheetSketch, err := world.CreateSketch(world.XY())
			require.NoError(t, err)
			cutX := (apexX + 10) / 2
			a := sheetSketch.CreatePoint(cutX, -30)
			b := sheetSketch.CreatePoint(cutX, 30)
			sheetSketch.Fix(a)
			sheetSketch.Fix(b)
			sheetSketch.CreateLine(a, b)
			_, err = sheetSketch.Solve(t.Context())
			require.NoError(t, err)
			require.Len(t, sheetSketch.Chains(), 1)
			sheet, err := decad.Extrude(t.Context(), doc, sheetSketch, sheetSketch.Chains()[0], decad.TwoSided{
				One: decad.DistanceSide{D: units.Millimeters(30)},
				Two: decad.DistanceSide{D: units.Millimeters(30)},
			})
			require.NoError(t, err)
			pieces, err := decad.Split(t.Context(), tooth, sheet)
			require.NoError(t, err)
			require.Len(t, pieces, 2)
			away := 1.0
			if apexX > 10 {
				away = -1
			}
			for _, tc := range []struct {
				body    *decad.Body
				role    decad.FeatureRef
				want    float64
				centerX float64
			}{
				{pieces[0], decad.CapEnd(pieces[0]), away, cutX},
				{pieces[1], decad.CapStart(pieces[1]), -away, cutX},
				{pieces[1], decad.CapEnd(pieces[1]), away, 10},
			} {
				faces, err := decad.Faces(decad.FaceCreatedBy(tc.role)).SelectFaces(tc.body)
				require.NoError(t, err)
				require.Len(t, faces, 1)
				plane, ok := faces[0].Surface().(decad.Plane)
				require.True(t, ok)
				normal, err := faces[0].NormalAt(plane.Frame.Origin())
				require.NoError(t, err)
				require.Greater(t, normal.Value.X*tc.want, 0.99)
				center, err := faces[0].Centroid()
				require.NoError(t, err)
				require.Less(t, center.Bound.Base(), 0.01)
				require.InDelta(t, tc.centerX, center.Value.X, center.Bound.Base()+1e-12)
				require.InDelta(t, 0, center.Value.Y, center.Bound.Base()+1e-12)
				require.InDelta(t, 0, center.Value.Z, center.Bound.Base()+1e-12)
			}
			motion, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Radians(0.2))
			require.NoError(t, err)
			placed, err := decad.Copy(t.Context(), pieces[1], decad.WithCopyPlacement(motion))
			require.NoError(t, err)
			placedCap, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(pieces[1]))).SelectFaces(placed)
			require.NoError(t, err)
			require.Len(t, placedCap, 1)
			placedCenter, err := placedCap[0].Centroid()
			require.NoError(t, err)
			require.InDelta(t, cutX, placedCenter.Value.X, placedCenter.Bound.Base()+1e-12)
			for _, candidate := range []*decad.Body{pieces[1], placed} {
				least, greatest, err := candidate.ExtremeFacesByCentroid(t.Context(), r3.NewVec(1, 0, 0))
				require.NoError(t, err)
				if apexX < 10 {
					require.Contains(t, least.Origins(), decad.CapStart(pieces[1]))
					require.Contains(t, greatest.Origins(), decad.CapEnd(pieces[1]))
				} else {
					require.Contains(t, least.Origins(), decad.CapEnd(pieces[1]))
					require.Contains(t, greatest.Origins(), decad.CapStart(pieces[1]))
				}
				reverseLeast, reverseGreatest, err := candidate.ExtremeFacesByCentroid(t.Context(), r3.NewVec(-1, 0, 0))
				require.NoError(t, err)
				require.Same(t, greatest, reverseLeast)
				require.Same(t, least, reverseGreatest)
			}
			shift, err := r3.Translation(r3.NewVec(2, 0, 0))
			require.NoError(t, err)
			unshift, err := r3.Translation(r3.NewVec(-2, 0, 0))
			require.NoError(t, err)
			shifted, err := decad.Copy(t.Context(), pieces[1], decad.WithCopyPlacement(shift))
			require.NoError(t, err)
			roundTrip, err := decad.Transform(t.Context(), shifted, unshift)
			require.NoError(t, err)
			_, _, err = roundTrip.ExtremeFacesByCentroid(t.Context(), r3.NewVec(1, 0, 0))
			require.NoError(t, err)
			innerSheetSketch, err := world.CreateSketch(world.XY())
			require.NoError(t, err)
			innerX := (cutX + 10) / 2
			innerA := innerSheetSketch.CreatePoint(innerX, -30)
			innerB := innerSheetSketch.CreatePoint(innerX, 30)
			innerSheetSketch.Fix(innerA)
			innerSheetSketch.Fix(innerB)
			innerSheetSketch.CreateLine(innerA, innerB)
			_, err = innerSheetSketch.Solve(t.Context())
			require.NoError(t, err)
			innerSheet, err := decad.Extrude(t.Context(), doc, innerSheetSketch, innerSheetSketch.Chains()[0], decad.TwoSided{
				One: decad.DistanceSide{D: units.Millimeters(30)},
				Two: decad.DistanceSide{D: units.Millimeters(30)},
			})
			require.NoError(t, err)
			beforeRoundTripSplit := doc.Bodies()
			_, err = decad.Split(t.Context(), roundTrip, innerSheet)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Equal(t, beforeRoundTripSplit, doc.Bodies())
			for _, piece := range pieces {
				mesh, err := piece.Tessellate(t.Context(), units.Millimeters(0.1),
					decad.WithVerification(decad.VerifyAll))
				require.NoError(t, err)
				require.True(t, mesh.BoundaryVerified())
				require.True(t, mesh.VolumeVerified())
			}
			canceledSheet, err := decad.Extrude(t.Context(), doc, sheetSketch, sheetSketch.Chains()[0], decad.TwoSided{
				One: decad.DistanceSide{D: units.Millimeters(30)},
				Two: decad.DistanceSide{D: units.Millimeters(30)},
			})
			require.NoError(t, err)
			before := doc.Bodies()
			ctx, cancel := context.WithCancel(t.Context())
			cancel()
			_, _, err = pieces[1].ExtremeFacesByCentroid(ctx, r3.NewVec(1, 0, 0))
			require.ErrorIs(t, err, context.Canceled)
			_, err = decad.Split(ctx, pieces[0], canceledSheet)
			require.ErrorIs(t, err, context.Canceled)
			require.Equal(t, before, doc.Bodies())
			retained, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(pieces[0]))).SelectFaces(pieces[0])
			require.NoError(t, err)
			require.Len(t, retained, 1)
			_, err = retained[0].Centroid()
			require.NoError(t, err)
		})
	}
}

func pointLoftSliceSpan() float64 { return 1.8 }

func TestPointLoftParallelSheetEightSlices(t *testing.T) {
	doc := decad.New()
	tooth := pointLoftRadialTab(t, doc)
	original, err := tooth.Volume()
	require.NoError(t, err)
	var sourceSides []*decad.Face
	for _, face := range tooth.Faces() {
		if len(face.Origins()) == 1 && face.Origins()[0] != decad.CapEnd(tooth) {
			sourceSides = append(sourceSides, face)
		}
	}
	require.GreaterOrEqual(t, len(sourceSides), 2)
	span := pointLoftSliceSpan()
	require.Positive(t, span)
	inner := tooth
	kept := make([]*decad.Body, 0, 8)
	for k := 1; k <= 8; k++ {
		sheet := pointLoftSliceSheet(t, doc, span*float64(k)/6, 30)
		pieces, err := decad.Split(t.Context(), inner, sheet)
		require.NoError(t, err)
		require.Len(t, pieces, 2)
		inner = pieces[0]
		kept = append(kept, pieces[1])
		for _, piece := range pieces {
			require.Equal(t, decad.BodySolid, piece.Kind())
			require.True(t, piece.IsSolid())
			mesh, err := piece.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		}
		for _, cap := range []decad.FeatureRef{
			decad.CapStart(pieces[1]), decad.CapEnd(pieces[1]),
		} {
			faces, err := decad.Faces(decad.FaceCreatedBy(cap)).SelectFaces(pieces[1])
			require.NoError(t, err)
			require.Len(t, faces, 1)
		}
		for _, source := range sourceSides {
			faces, err := decad.Faces(decad.FaceCreatedBy(source.Origins()[0])).SelectFaces(pieces[1])
			require.NoError(t, err)
			require.Len(t, faces, 1)
			require.Equal(t, reflect.TypeOf(source.Surface()), reflect.TypeOf(faces[0].Surface()))
		}
	}
	require.Len(t, kept, 8)
	volumes := append(kept, inner)
	sum, totalBound := 0.0, original.Bound.Base()
	for _, slab := range volumes {
		volume, err := slab.Volume()
		require.NoError(t, err)
		require.Positive(t, volume.Value.Base())
		sum += volume.Value.Base()
		totalBound += volume.Bound.Base()
	}
	require.InDelta(t, original.Value.Base(), sum, totalBound)
}

func TestPointLoftParallelSheetRefusesShortToolAtomically(t *testing.T) {
	doc := decad.New()
	tooth := pointLoftRadialTab(t, doc)
	sheet := pointLoftSliceSheet(t, doc, pointLoftSliceSpan()/6, 0.1)
	before := doc.Bodies()
	pieces, err := decad.Split(t.Context(), tooth, sheet)
	require.Error(t, err)
	require.True(t, errors.Is(err, decad.ErrUnsupported))
	require.Empty(t, pieces)
	require.Equal(t, before, doc.Bodies())
}
