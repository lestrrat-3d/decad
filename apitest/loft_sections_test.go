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

func threeSquareSections(t *testing.T, half, heights [3]float64) [3]decad.LoftSection {
	return threeRectangleSections(t, half, half, [3]float64{}, heights)
}

func threeRectangleSections(t *testing.T, halfX, halfY, centerX, heights [3]float64) [3]decad.LoftSection {
	t.Helper()
	w := sketch.NewWorld()
	var sections [3]decad.LoftSection
	for i := range sections {
		plane := w.XY()
		if i != 0 {
			var err error
			plane, err = w.CreateOffsetPlane(w.XY(), heights[i])
			require.NoError(t, err)
		}
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		r := s.CreateRectangle(centerX[i]-halfX[i], -halfY[i], centerX[i]+halfX[i], halfY[i])
		s.Fix(r.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		sections[i] = decad.LoftSection{Sketch: s, Profile: s.Profiles()[0]}
	}
	return sections
}

func TestLoftSectionsInterpolatesThreeSketchesWithCertifiedMesh(t *testing.T) {
	t.Parallel()
	sections := threeSquareSections(t, [3]float64{10, 15, 8}, [3]float64{0, 5, 10})
	body, err := decad.New().LoftSections(t.Context(), sections[:]...)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, body.Kind())

	// q(t) = 1 + 2.2t - 2.4t². The closed-form integral is independent of
	// the held mesh's band count and catches a signed-volume-only allowance.
	const a, b = -2.4, 2.2
	trueVolume := 4000 * (1 + b + (b*b+2*a)/3 + a*b/2 + a*a/5)
	volume, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, volume.Exactness)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-trueVolume), volume.Bound.Base())
	require.Less(t, volume.Bound.Base(), 20.0)

	faces := body.Faces()
	require.Len(t, faces, 6)
	planes, nurbs := 0, 0
	for _, face := range faces {
		switch face.Surface().Kind() {
		case decad.KindPlane:
			planes++
		case decad.KindNURBS:
			nurbs++
		default:
			t.Fatalf("unexpected loft face kind: %v", face.Surface().Kind())
		}
	}
	require.Equal(t, 2, planes)
	require.Equal(t, 4, nurbs)

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Len(t, mesh.Triangles(), 132)
	require.LessOrEqual(t, mesh.Bound().Base(), 0.1)
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
	_, err = body.Tessellate(t.Context(), units.Millimeters(0.01), decad.WithVerification(decad.VerifyAll))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	live := make(map[*decad.Face]struct{}, len(faces))
	for _, face := range faces {
		live[face] = struct{}{}
	}
	for _, face := range mesh.SourceFaces() {
		_, ok := live[face]
		require.True(t, ok, "each facet must name a live Body face")
	}

	// The quadratic is wider than its middle control section between sampled
	// planes. Bounds must enclose that interior extremum as well as the rings.
	box, err := body.Bounds()
	require.NoError(t, err)
	peakT := -b / (2 * a)
	peak := 10 * (1 + b*peakT + a*peakT*peakT)
	require.GreaterOrEqual(t, box.Max.X+box.Bound.Base(), peak)
	require.LessOrEqual(t, box.Min.X-box.Bound.Base(), -peak)

	// An independent area integral checks the held area allowance. For each
	// square edge the smooth density is 200*q*sqrt(1+q'²).
	const steps = 100000
	areaIntegral := 0.0
	for i := 0; i <= steps; i++ {
		x := float64(i) / steps
		q, qp := 1+b*x+a*x*x, b+2*a*x
		weight := 2.0
		if i == 0 || i == steps {
			weight = 1
		} else if i%2 == 1 {
			weight = 4
		}
		areaIntegral += weight * q * math.Sqrt(1+qp*qp)
	}
	trueArea := 400*(1+0.8*0.8) + 800*areaIntegral/(3*steps)
	area, err := body.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-trueArea), area.Bound.Base())
}

func TestLoftSectionsPlacedCapsRemainPlanar(t *testing.T) {
	t.Parallel()
	sections := threeSquareSections(t, [3]float64{10, 15, 8}, [3]float64{0, 5, 10})
	body, err := decad.New().LoftSections(t.Context(), sections[:]...)
	require.NoError(t, err)
	checkNormals := func(b *decad.Body, expected map[float64]float64) {
		t.Helper()
		seen := 0
		for _, face := range b.Faces() {
			plane, ok := face.Surface().(decad.Plane)
			if !ok {
				continue
			}
			want, ok := expected[plane.Frame.Origin().Z]
			require.True(t, ok)
			normal, err := face.NormalAt(plane.Frame.Origin())
			require.NoError(t, err)
			require.Equal(t, want, normal.Value.Z)
			seen++
		}
		require.Equal(t, 2, seen)
	}
	checkNormals(body, map[float64]float64{0: -1, 10: 1})
	move, err := r3.Translation(r3.NewVec(40, 0, 0))
	require.NoError(t, err)
	placed, err := body.PlacedCopy(t.Context(), move)
	require.NoError(t, err)
	planes := 0
	for _, face := range placed.Faces() {
		if plane, ok := face.Surface().(decad.Plane); ok {
			planes++
			require.GreaterOrEqual(t, plane.Frame.Origin().X, 40.0)
		}
	}
	require.Equal(t, 2, planes)
	checkNormals(placed, map[float64]float64{0: -1, 10: 1})
	mesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	mirror, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	reflection, err := r3.Reflection(mirror)
	require.NoError(t, err)
	reflected, err := body.PlacedCopy(t.Context(), reflection)
	require.NoError(t, err)
	checkNormals(reflected, map[float64]float64{0: 1, -10: -1})
	_, err = reflected.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
}

func TestLoftSectionsRefusesUnprovenScaleAndSpacing(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		half    [3]float64
		halfY   [3]float64
		centerX [3]float64
		heights [3]float64
	}{
		{"pinched scale", [3]float64{10, 1, 10}, [3]float64{10, 1, 10}, [3]float64{}, [3]float64{0, 5, 10}},
		{"unequal spacing", [3]float64{10, 15, 8}, [3]float64{10, 15, 8}, [3]float64{}, [3]float64{0, 4, 10}},
		{"nonhomothetic section", [3]float64{10, 15, 8}, [3]float64{10, 15, 9}, [3]float64{}, [3]float64{0, 5, 10}},
		{"scale origin outside profile", [3]float64{5, 7.5, 4}, [3]float64{5, 7.5, 4},
			[3]float64{15, 22.5, 12}, [3]float64{0, 5, 10}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			sections := threeRectangleSections(t, tc.half, tc.halfY, tc.centerX, tc.heights)
			doc := decad.New()
			body, err := doc.LoftSections(t.Context(), sections[:]...)
			require.Nil(t, body)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Empty(t, doc.Bodies())
		})
	}
}

func TestLoftSectionsRequiresThreeSections(t *testing.T) {
	t.Parallel()
	sections := threeSquareSections(t, [3]float64{10, 15, 8}, [3]float64{0, 5, 10})
	doc := decad.New()
	body, err := doc.LoftSections(t.Context(), sections[:2]...)
	require.Nil(t, body)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Empty(t, doc.Bodies())
}
