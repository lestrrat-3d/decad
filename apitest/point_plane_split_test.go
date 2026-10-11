package apitest_test

import (
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

// pointBevelSliceSheet follows the gallery's back-cone slice construction.
// The line belongs to a world-XY Sketch and its two-sided ExtrudeChain is
// the finite sheet passed to the public Split operation.
func pointBevelSliceSheet(t *testing.T, doc *decad.Document, offset, halfWidth float64) *decad.Body {
	return pointBevelSliceSheetDirected(t, doc, offset, halfWidth, false)
}

func pointBevelSliceSheetDirected(t *testing.T, doc *decad.Document,
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
	sheet, err := doc.ExtrudeChain(s, s.Chains()[0], decad.TwoSided{
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
			tooth, _ := bevelReferenceTooth(t, doc)
			span := pointBevelRootSpan()
			first := pointBevelSliceSheetDirected(t, doc, span/6, 30, reverse)
			pieces, err := doc.Split(t.Context(), tooth, first)
			require.NoError(t, err)
			require.Len(t, pieces, 2)
			second := pointBevelSliceSheetDirected(t, doc, span/3, 30, !reverse)
			parts, err := doc.Split(t.Context(), pieces[0], second)
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
			tooth, err := doc.LoftFromPoint(t.Context(), r3.NewVec(apexX, 0, 0),
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
			sheet, err := doc.ExtrudeChain(sheetSketch, sheetSketch.Chains()[0], decad.TwoSided{
				One: decad.DistanceSide{D: units.Millimeters(30)},
				Two: decad.DistanceSide{D: units.Millimeters(30)},
			})
			require.NoError(t, err)
			pieces, err := doc.Split(t.Context(), tooth, sheet)
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
			placed, err := pieces[1].Placed(t.Context(), motion)
			require.NoError(t, err)
			placedCap, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(pieces[1]))).SelectFaces(placed)
			require.NoError(t, err)
			require.Len(t, placedCap, 1)
			placedCenter, err := placedCap[0].Centroid()
			require.NoError(t, err)
			require.InDelta(t, cutX, placedCenter.Value.X, placedCenter.Bound.Base()+1e-12)
			for _, piece := range pieces {
				mesh, err := piece.Tessellate(t.Context(), units.Millimeters(0.1),
					decad.WithVerification(decad.VerifyAll))
				require.NoError(t, err)
				require.True(t, mesh.BoundaryVerified())
				require.True(t, mesh.VolumeVerified())
			}
		})
	}
}

func pointBevelRootSpan() float64 {
	section, _ := bevelReferenceSection()
	rootX, rootY := section[3].U, section[3].V
	rootNorm := math.Hypot(rootX, rootY)
	coneX, coneY := rootX/rootNorm, rootY/rootNorm
	toeMidX := (section[4].U + section[5].U) / 2
	toeMidY := (section[4].V + section[5].V) / 2
	heelMidX := (section[3].U + section[2].U) / 2
	heelMidY := (section[3].V + section[2].V) / 2
	return (heelMidX-toeMidX)*coneX + (heelMidY-toeMidY)*coneY
}

func TestPointLoftParallelSheetEightSlices(t *testing.T) {
	doc := decad.New()
	tooth, _ := bevelReferenceTooth(t, doc)
	original, err := tooth.Volume()
	require.NoError(t, err)
	var sourceSides []*decad.Face
	for _, face := range tooth.Faces() {
		if len(face.Origins()) == 1 && face.Origins()[0] != decad.CapEnd(tooth) {
			sourceSides = append(sourceSides, face)
		}
	}
	require.GreaterOrEqual(t, len(sourceSides), 2)
	span := pointBevelRootSpan()
	require.Positive(t, span)
	inner := tooth
	kept := make([]*decad.Body, 0, 8)
	for k := 1; k <= 8; k++ {
		sheet := pointBevelSliceSheet(t, doc, span*float64(k)/6, 30)
		pieces, err := doc.Split(t.Context(), inner, sheet)
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
	tooth, _ := bevelReferenceTooth(t, doc)
	sheet := pointBevelSliceSheet(t, doc, pointBevelRootSpan()/6, 0.1)
	before := doc.Bodies()
	pieces, err := doc.Split(t.Context(), tooth, sheet)
	require.Error(t, err)
	require.True(t, errors.Is(err, decad.ErrUnsupported))
	require.Empty(t, pieces)
	require.Equal(t, before, doc.Bodies())
}
