package apitest_test

import (
	"fmt"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func emergingFlatSections(t *testing.T, flatXs ...float64) []decad.LoftSection {
	t.Helper()
	return emergingFlatSectionsShape(t, false, 0, flatXs...)
}

func emergingFlatSectionsShape(t *testing.T, invalid bool, originX float64, flatXs ...float64) []decad.LoftSection {
	t.Helper()
	w := sketch.NewWorld()
	sections := make([]decad.LoftSection, 2+len(flatXs))
	for i := range sections {
		z := float64(i)
		if i >= 2 {
			z++
		}
		frame, err := r3.NewFrame(r3.Vec{X: originX, Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
		require.NoError(t, err)
		plane, err := w.CreatePlaneFromFrame(frame)
		require.NoError(t, err)
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		point := func(x, y float64) *sketch.Point {
			p := s.CreatePoint(x, y)
			s.Fix(p)
			return p
		}
		if i < 2 {
			c := point(0, 0)
			p := []*sketch.Point{point(1, 0), point(0, 1), point(-1, 0), point(0, -1)}
			for j := range p {
				s.CreateArc(c, p[j], p[(j+1)%4])
			}
		} else {
			flatX := flatXs[i-2]
			high, low := point(flatX, 1), point(flatX, -1)
			_, err = s.CreateSpline(high, point(-2, 2), point(-2, -2), low)
			require.NoError(t, err)
			joinX := flatX
			if invalid {
				joinX += 0.125
			}
			a, b := point(joinX, -0.5), point(flatX, 0.5)
			s.CreateLine(low, a)
			s.CreateLine(a, b)
			s.CreateLine(b, high)
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, profile := range s.Profiles() {
			if profile.Valid && len(profile.Holes) == 0 && len(profile.Outer) == 4 {
				sections[i] = decad.LoftSection{Sketch: s, Profile: profile}
			}
		}
		require.NotNil(t, sections[i].Profile)
	}
	return sections
}

func emergingFlatCorrespondence() decad.CircularLoftCorrespondence {
	var correspondence decad.CircularLoftCorrespondence
	for i := range correspondence.Numerators {
		correspondence.Numerators[i] = []int64{0, 1, 4, 6}
		correspondence.Denominators[i] = 6
	}
	correspondence.EmergingFlat = true
	return correspondence
}

func TestCircularLoftEmergingFlat(t *testing.T) {
	t.Parallel()
	sections := emergingFlatSections(t, 1)
	body, err := decad.LoftSectionsWithCircularCorrespondence(t.Context(), decad.New(), emergingFlatCorrespondence(), sections...)
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	for i := range 4 {
		ref := featureRefWithRole(t, body, fmt.Sprintf("side(0,%d)", i))
		_, err := decad.Faces(decad.FaceCreatedBy(ref)).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
	}
	closed, signed := meshIsWatertight(mesh)
	require.True(t, closed)
	require.Positive(t, signed)
	vertices := mesh.Vertices()
	for _, z := range []float64{0, 1} {
		count := 0
		for _, p := range vertices {
			if p.X == 1 && p.Y == 0 && p.Z == z {
				count++
			}
		}
		require.Equal(t, 1, count, "one shared source apex at z=%g", z)
	}
	areas := map[string]float64{}
	for i, tri := range mesh.Triangles() {
		a, b, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
		if a.X != 1 || b.X != 1 || c.X != 1 || a.Z < 1 || b.Z < 1 || c.Z < 1 {
			continue
		}
		face := mesh.SourceFaces()[i]
		for _, origin := range face.Origins() {
			if strings.HasPrefix(origin.Role, "side(2,") {
				areas[origin.Role] += b.Sub(a).Cross(c.Sub(a)).Len() / 2
			}
		}
	}
	require.Len(t, areas, 3, "each recorded flat edge must keep its target role")
	for role := range areas {
		ref := featureRefWithRole(t, body, role)
		_, err := decad.Faces(decad.FaceCreatedBy(ref), decad.Planar()).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
	}
	var area float64
	for _, a := range areas {
		area += a
	}
	require.InDelta(t, 2, area, 1e-12)
	for _, a := range areas {
		require.True(t, math.Abs(a-0.5) < 1e-12 || math.Abs(a-1) < 1e-12)
	}
	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, 10.4470351474, volume.Value.Base(), volume.Bound.Base()+0.000000003)
}

func TestCircularLoftEmergingFlatRefusesOffTangent(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	_, err := decad.LoftSectionsWithCircularCorrespondence(t.Context(), doc, emergingFlatCorrespondence(), emergingFlatSections(t, 1.125)...)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Empty(t, doc.Bodies())
}

func TestCircularLoftEmergingFlatRefusesLaterOffTangent(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	_, err := decad.LoftSectionsWithCircularCorrespondence(t.Context(), doc, emergingFlatCorrespondence(), emergingFlatSections(t, 1, 1.125)...)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Empty(t, doc.Bodies())
}

func TestCircularLoftEmergingFlatRefusesBentChain(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	_, err := decad.LoftSectionsWithCircularCorrespondence(t.Context(), doc, emergingFlatCorrespondence(), emergingFlatSectionsShape(t, true, 0, 1)...)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "three ascending collinear vertical lines and a spline")
	require.Empty(t, doc.Bodies())
}

func TestCircularLoftEmergingFlatRefusesUnrepresentablePlane(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	_, err := decad.LoftSectionsWithCircularCorrespondence(t.Context(), doc, emergingFlatCorrespondence(), emergingFlatSectionsShape(t, false, 0.1, 1)...)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "tangent plane world X must be exactly representable")
	require.Empty(t, doc.Bodies())
}
