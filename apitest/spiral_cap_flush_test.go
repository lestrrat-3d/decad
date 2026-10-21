package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// spiralConeSlack is how far a held cone-face vertex may sit from its cone in
// mm. The fixture's chord sag stays under 0.005 mm. The temporary cap proxy of
// the join moves its vertices 0.032 mm in Y, about 0.02 mm off the cone.
const spiralConeSlack = 0.01

// requireConeFacesFlush fails when a vertex of a Cone face sits farther than
// spiralConeSlack from that face's cone, and returns how many vertices it
// checked.
func requireConeFacesFlush(t *testing.T, body *decad.Body) int {
	t.Helper()
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	verts, faces := mesh.Vertices(), mesh.SourceFaces()
	checked := 0
	for i, tri := range mesh.Triangles() {
		cone, ok := faces[i].Surface().(decad.Cone)
		if !ok {
			continue
		}
		for _, vi := range tri {
			v := verts[vi].Sub(cone.Origin)
			axial := v.Dot(cone.Axis)
			radial := v.Sub(cone.Axis.Scale(axial)).Len()
			radius := cone.Radius.Base() + axial*math.Tan(cone.HalfAngle.Base())
			off := math.Abs(radial-radius) * math.Cos(cone.HalfAngle.Base())
			require.LessOrEqual(t, off, spiralConeSlack, "vertex %d of a cone face at %v", vi, verts[vi])
			checked++
		}
	}
	return checked
}

// TestSpiralFirstToothUnionKeepsCapsOnTheirCones checks that the joined tooth's
// toe and heel faces lie on the cones that trimmed them, not on the proxy
// offset the join used to decide the Boolean.
func TestSpiralFirstToothUnionKeepsCapsOnTheirCones(t *testing.T) {
	doc := decad.New()
	blank, tooth := spiralBlankAndTooth(t, doc)
	joined, err := decad.Union(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.Greater(t, requireConeFacesFlush(t, joined), 1000)
}

// TestSpiralCopiedToothUnionKeepsCapsOnTheirCones is the same check on a join
// of the first tooth and an exact copy, whose proxy caps are placed.
func TestSpiralCopiedToothUnionKeepsCapsOnTheirCones(t *testing.T) {
	doc := decad.New()
	blank, tooth := spiralBlankAndTooth(t, doc)
	copyTooth, err := decad.Copy(t.Context(), tooth,
		decad.WithCopyRotation(r3.Vec{}, r3.Vec{X: 1}, units.Radians(2*math.Pi*23/31)))
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), blank, tooth, copyTooth)
	require.NoError(t, err)
	require.Greater(t, requireConeFacesFlush(t, joined), 2000)
}
