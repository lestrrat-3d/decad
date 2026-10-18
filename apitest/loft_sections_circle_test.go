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

func threeArcCircleSections(t *testing.T, radii, heights [3]float64) [3]decad.LoftSection {
	t.Helper()
	w := sketch.NewWorld()
	var sections [3]decad.LoftSection
	for i, radius := range radii {
		plane := w.XY()
		if i != 0 {
			var err error
			plane, err = w.CreateOffsetPlane(w.XY(), heights[i])
			require.NoError(t, err)
		}
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		center := s.CreatePoint(0, 0)
		s.Fix(center)
		coords := [4][2]float64{{radius, 0}, {0, radius}, {-radius, 0}, {0, -radius}}
		var points [4]*sketch.Point
		for j, xy := range coords {
			points[j] = s.CreatePoint(xy[0], xy[1])
			s.Fix(points[j])
		}
		for j := range points {
			s.CreateArc(center, points[j], points[(j+1)%len(points)])
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, profile := range s.Profiles() {
			if profile.Valid && len(profile.Outer) == 4 && len(profile.Holes) == 0 {
				sections[i] = decad.LoftSection{Sketch: s, Profile: profile}
				break
			}
		}
		require.NotNil(t, sections[i].Profile, "Sketch must produce a four-arc disk")
	}
	return sections
}

func TestLoftSectionsVaryingCirclesKeepVolumeAndMeshProof(t *testing.T) {
	t.Parallel()
	sections := threeArcCircleSections(t, [3]float64{8, 10, 12}, [3]float64{0, 5, 10})
	doc := decad.New()
	body, err := decad.LoftSections(t.Context(), doc, sections[:]...)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, body.Kind())
	require.Len(t, doc.Bodies(), 1)
	require.Len(t, body.Faces(), 6)

	roles := make(map[string]bool)
	planes, nurbs := 0, 0
	for _, face := range body.Faces() {
		for _, origin := range face.Origins() {
			roles[origin.Role] = true
		}
		switch face.Surface().Kind() {
		case decad.KindPlane:
			planes++
		case decad.KindNURBS:
			nurbs++
		default:
			t.Fatalf("unexpected face kind %v", face.Surface().Kind())
		}
	}
	require.Equal(t, 2, planes)
	require.Equal(t, 4, nurbs)
	for _, suffix := range []string{"Start", "End"} {
		require.True(t, roles["cap"+suffix], "missing cap role %s", suffix)
	}
	for _, role := range []string{"side(0,0)", "side(0,1)", "side(0,2)", "side(0,3)"} {
		require.True(t, roles[role], "missing source role %s", role)
	}

	// A linearly changing radius has exact volume (3040/3)*pi. These
	// independent decimal endpoints enclose pi more tightly than its two
	// neighbouring float64 values, which are wider than the published bound.
	volume, err := body.Volume()
	require.NoError(t, err)
	piLo, ok := new(big.Rat).SetString("3.14159265358979323846")
	require.True(t, ok)
	piHi, ok := new(big.Rat).SetString("3.14159265358979323847")
	require.True(t, ok)
	coefficient := big.NewRat(3040, 3)
	reading := new(big.Rat).SetFloat64(volume.Value.Base())
	bound := new(big.Rat).SetFloat64(volume.Bound.Base())
	require.LessOrEqual(t, new(big.Rat).Sub(reading, bound).Cmp(new(big.Rat).Mul(coefficient, piLo)), 0)
	require.GreaterOrEqual(t, new(big.Rat).Add(reading, bound).Cmp(new(big.Rat).Mul(coefficient, piHi)), 0)
	require.Less(t, volume.Bound.Base(), 5.0)

	area, err := body.Area()
	require.NoError(t, err)
	trueArea := math.Pi * (8*8 + 12*12 + (8+12)*math.Hypot(10, 4))
	require.LessOrEqual(t, math.Abs(area.Value.Base()-trueArea), area.Bound.Base())
	centroid, err := body.Centroid()
	require.NoError(t, err)
	z := 10 * (float64(8*8)/2 + float64(2*8*4)/3 + float64(4*4)/4) /
		(float64(8*8) + float64(8*4) + float64(4*4)/3)
	require.LessOrEqual(t, math.Abs(centroid.Value.Z-z), centroid.Bound.Base())
	require.LessOrEqual(t, math.Abs(centroid.Value.X), centroid.Bound.Base())
	require.LessOrEqual(t, math.Abs(centroid.Value.Y), centroid.Bound.Base())

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.05)
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
	live := make(map[*decad.Face]bool)
	for _, face := range body.Faces() {
		live[face] = true
	}
	for _, face := range mesh.SourceFaces() {
		require.True(t, live[face], "each facet must name a live source face")
	}
	_, err = body.Tessellate(t.Context(), units.Millimeters(0.001), decad.WithVerification(decad.VerifyAll))
	require.ErrorIs(t, err, decad.ErrUnsupported)

	move, err := r3.Translation(r3.NewVec(16, 0, 0))
	require.NoError(t, err)
	placed, err := decad.Copy(t.Context(), body, decad.WithCopyPlacement(move))
	require.NoError(t, err)
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.BoundaryVerified())
	require.True(t, placedMesh.VolumeVerified())
}

func TestLoftSectionsVaryingCirclesRefuseOtherShapes(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		radii   [3]float64
		heights [3]float64
	}{
		{"nonlinear radius", [3]float64{8, 15, 12}, [3]float64{0, 5, 10}},
		{"unequal spacing", [3]float64{8, 10, 12}, [3]float64{0, 4, 10}},
		{"scale outside gate", [3]float64{8, 20, 32}, [3]float64{0, 5, 10}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			sections := threeArcCircleSections(t, tc.radii, tc.heights)
			doc := decad.New()
			body, err := decad.LoftSections(t.Context(), doc, sections[:]...)
			require.Nil(t, body)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Empty(t, doc.Bodies())
		})
	}
}
