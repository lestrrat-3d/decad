package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// A shallow planar cut of a sphere must keep enough mesh detail for a section
// to read the remaining wall. The former contact sweep hit its work ceiling,
// retried at a coarser chord and left a bound above the section tolerance.
func TestSectionOutlineOfSphereCutByExtrudedDisk(t *testing.T) {
	doc := decad.New()
	s, profile := circleSketch(t, 0, 0, 12)
	sphere, err := decad.Revolve(doc, s, profile, uAxis, decad.FullRevolution{}).Do(t.Context())
	require.NoError(t, err)
	tool := slabCylinder(t, doc, r3.NewVec(11, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1), 30, 15)
	body, err := decad.Cut(sphere, tool).Do(t.Context())
	require.NoError(t, err)
	const z = 0.37
	outline, err := body.SectionOutline(t.Context(), outlinePlane(t, r3.NewVec(0, 0, z), r3.NewVec(0, 0, 1)),
		units.Millimeters(0.05))
	require.NoError(t, err)
	require.Len(t, outline.Loops, 1)
	radius := math.Sqrt(144 - z*z)
	capArea := radius*radius*math.Acos(11/radius) - 11*math.Sqrt(radius*radius-121)
	require.InDelta(t, math.Pi*radius*radius-capArea, outline.PolygonArea().Base(), 1)
	require.LessOrEqual(t, outline.OffSurface.Base(), 0.05)
}
