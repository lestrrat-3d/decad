package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The box crosses the rounded profile's lower-right arc in the middle twist
// station. Its Union exercises a real cut subset of an authenticated arc
// wall, then consumes the Sweep's cut-stable area allowance.
func TestTwistLocalAreaRealBooleanArcSubset(t *testing.T) {
	doc, swept, source := roundedTwistLocalAreaSource(t)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(1, -0.4, 2.5, 0.4)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{
		D: units.Millimeters(1), Dir: Along,
	})
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(0, 0, 2))
	require.NoError(t, err)
	box, err = Transform(t.Context(), box, move)
	require.NoError(t, err)
	joined, err := Union(t.Context(), swept, box)
	require.NoError(t, err)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.2), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	// A result arc facet uses a new rim vertex. That confirms the source
	// cell was actually split by the Boolean rather than merely carried
	// through unchanged beside the box.
	sourceVertex := make(map[r3.Vec]struct{}, len(source.Mesh.Vertices))
	for _, vertex := range source.Mesh.Vertices {
		sourceVertex[vertex] = struct{}{}
	}
	splitArc := false
	for i, face := range mesh.n.Source {
		if len(face.Origins) == 0 || face.Origins[0].Role != "side(0,0,1)" {
			continue
		}
		for _, index := range mesh.n.Triangles[i] {
			if _, exists := sourceVertex[mesh.n.Vertices[index]]; !exists {
				splitArc = true
			}
		}
	}
	require.True(t, splitArc, "the real Boolean must cut a rounded source wall")
	area, err := joined.Area()
	require.NoError(t, err)
	// The Boolean carries the source's local allowance through its cut
	// subset, then adds its own trim-motion and facet charges.
	require.Less(t, source.Mesh.AreaSlack, 50.0)
	require.Less(t, area.Bound.Base(), 80.0)
	t.Logf("source area allowance=%g result area bound=%g split_arc=%t",
		source.Mesh.AreaSlack, area.Bound.Base(), splitArc)
}
