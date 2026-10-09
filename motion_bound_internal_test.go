package decad

import (
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestPrismMoverRecordRadiusReadsArcsTightly pins the prism arm of
// moverRecordRadius on a slot whose semicircles of radius 5 sit at u = ±15,
// swept 10 mm: the radius covers the L1 norm of every vertex of the body's
// mesh and stays within twice the largest.
//
// Shown to fail first: read through the walks' own CoordUpper, which charge
// an ArcSeg its coordinates' L1 sizes as a radius, the record radius was 180
// against a largest vertex near 32.
func TestPrismMoverRecordRadiusReadsArcsTightly(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	slot, err := s.CreateSlot(-15, 0, 15, 0, 5)
	require.NoError(t, err)
	s.Fix(slot.C1)
	s.Fix(slot.C2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := New().Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, err)
	radius := moverRecordRadius(t.Context(), b)
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.01))
	require.NoError(t, err)
	worst := 0.0
	for _, v := range mesh.vertices {
		worst = max(worst, vecL1(v))
	}
	require.LessOrEqual(t, worst, radius)
	require.LessOrEqual(t, radius, 2*worst)
}
