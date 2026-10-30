package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestFacetedRetainedBlendAreaCharge blends one body edge by edge
// (agent-docs/retained-edge-blend-design.md §6.3, Area). A plate 80×60×10
// carries a 20×20 boss 15 high, a counterbored M6 hole and a round boss of
// radius 6 placed by a Transform, so every body stays on the mesh path. Each
// of the four side fillets rebuilds the plate's top, whose geometric area term
// the rebuilt body charges afresh; the rim chamfer then rebuilds the round
// boss's wall and top. The area bound stays within Verify's tolerance only
// when each rewrite charges the shortfall of the faces' terms, not their old
// terms again on top of the new ones.
func TestFacetedRetainedBlendAreaCharge(t *testing.T) {
	t.Parallel()
	sc := newRetentionScene()
	plate := rectBox(t, sc.w, sc.doc, 0, 0, 80, 60, 10)
	top := decad.CapEnd(plate)
	body, err := decad.Union(t.Context(), plate, placed(t, rectBox(t, sc.w, sc.doc, 30, 20, 50, 40, 15), 10))
	require.NoError(t, err)
	body, err = decad.Hole(t.Context(), body, decad.HoleSpec{Face: decad.Faces(decad.FaceCreatedBy(top)),
		At: r3.NewVec(15, 15, 10), Diameter: units.Millimeters(6.6), Depth: decad.ThroughHole{},
		Head: decad.Counterbore{Diameter: units.Millimeters(11), Depth: units.Millimeters(6.4)}})
	require.NoError(t, err)
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, 6)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	post, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{X: 65, Y: 45, Z: 10})
	require.NoError(t, err)
	post, err = decad.Transform(t.Context(), post, move)
	require.NoError(t, err)
	body, err = decad.Union(t.Context(), body, post)
	require.NoError(t, err)
	exact := 54000 - math.Pi*3.3*3.3*10 - math.Pi*(5.5*5.5-3.3*3.3)*6.4 + math.Pi*36*8
	requireBossVolume(t, body, exact)

	// Each side loses r²(1 − π/4) of section over its 15 mm.
	const r = 2.0
	filleted, err := decad.Fillet(t.Context(), body, bossSides(t, body, boxBoss.corners()), units.Millimeters(r))
	require.NoError(t, err)
	exact -= 4 * 15 * r * r * (1 - math.Pi/4)
	requireBossVolume(t, filleted, exact)

	// The rim chamfer removes a 1 × 1 triangle swept about the axis at its
	// centroid's radius 6 − 1/3. The Union shifts the round boss clear of the
	// plate (agent-docs/gap-tolerance-design.md §5.2), so its rim is found
	// near height 18.
	var rim *decad.EdgeQuery
	for _, e := range filleted.Edges() {
		p := e.Start().Position().Value
		if _, ok := e.Curve().(decad.Circle3); ok && math.Abs(p.Z-18) < 0.05 && math.Abs(math.Hypot(p.X-65, p.Y-45)-6) < 0.05 {
			require.Nil(t, rim, "one rim at the round boss's top")
			rim = decad.Edges(decad.EndpointAt(p), decad.Circular()).Exactly(1)
		}
	}
	require.NotNil(t, rim)
	chamfered, err := decad.Chamfer(t.Context(), filleted, rim, units.Millimeters(1))
	require.NoError(t, err)
	exact -= 2 * math.Pi * (6 - 1.0/3) * 0.5
	requireBossVolume(t, chamfered, exact)

	report, err := chamfered.Document().Verify(t.Context())
	require.NoError(t, err)
	reading, err := report.ForBody(chamfered)
	require.NoError(t, err)
	area, err := chamfered.Area()
	require.NoError(t, err)
	t.Logf("area %.6f ± %.4g", area.Value.Mag(), area.Bound.Mag())
	require.Equal(t, decad.ToleranceSatisfied, reading.Area.Tolerance.State)
	require.Equal(t, decad.Sound, reading.Status)
}
