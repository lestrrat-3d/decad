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
// mm. The fixture's chord sag stays under 0.005 mm and a restored cap stands
// 0.003 mm above its cone. The temporary cap proxy of the join moves its
// vertices 0.032 mm in Y, about 0.02 mm off the cone.
const spiralConeSlack = 0.01

// requireConeFacesFlush fails when more than maxOff vertices of Cone faces sit
// farther than spiralConeSlack from their face's cone, and returns how many
// vertices it checked. The join leaves a vertex of a restored cap at its proxy
// position where lowering it would fold a facet against the blank.
func requireConeFacesFlush(t *testing.T, body *decad.Body, maxOff int) int {
	t.Helper()
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	verts, faces := mesh.Vertices(), mesh.SourceFaces()
	checked, offCone := 0, 0
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
			if off > spiralConeSlack {
				offCone++
			}
			checked++
		}
	}
	t.Logf("off-cone vertices: %d of %d", offCone, checked)
	require.LessOrEqual(t, offCone, maxOff, "vertices of cone faces off their cone")
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
	require.Greater(t, requireConeFacesFlush(t, joined, 0), 1000)
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
	require.Greater(t, requireConeFacesFlush(t, joined, 0), 2000)
}

// TestSpiralFirstToothUnionSurvivesPlacement checks that a joined tooth can
// still be turned: the held mesh must stay embedded through the rounding of
// the placement.
func TestSpiralFirstToothUnionSurvivesPlacement(t *testing.T) {
	doc := decad.New()
	blank, tooth := spiralBlankAndTooth(t, doc)
	joined, err := decad.Union(t.Context(), blank, tooth)
	require.NoError(t, err)
	turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(math.Pi/2))
	require.NoError(t, err)
	_, err = decad.Transform(t.Context(), joined, turn)
	require.NoError(t, err)
}
