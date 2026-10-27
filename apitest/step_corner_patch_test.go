package apitest_test

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestSTEPWritesCornerPatches writes, through the public API, three bodies
// whose corners are single-loop cone and sphere patches
// (agent-docs/step-export-design.md), and requires the writer each takes.
//
// The rounded box (20x20x10, vertical edges filleted r = 3, top loop
// chamfered by 1) holds four cone patches, each two arcs and two generators,
// and the pocket mouth chamfer holds four three-edge cone patches meeting at
// their apex vertices. A loop chamfer's corner cones are cap-blend band
// patches whose tags depart from the ruled patches the build assembles by a
// stated bound (agent-docs/modify-reach-design.md §8.3), so both bodies take
// the faceted writer: no CONICAL_SURFACE, one planar ADVANCED_FACE per mesh
// triangle. The rounded box's volume is 9·(400 − 9(4 − π)) + ∫₀¹ (20 − 2d)² −
// (4 − π)(3 − d)² dd = 3612 + 262π/3; ReadSTEP reads its file back closed,
// within the 0.1 mm chord times the curved faces' area of it. The cube
// filleted r = 2 on every edge holds eight three-arc sphere patches, whose
// tags are their surfaces, and takes the analytic writer; ReadSTEP reads no
// sphere patch other than a cap, so that file is checked by its surfaces. The
// cone legs were shown to fail with supportsAnalyticSTEP's tag gate deleted
// (both bodies were written analytic), and the sphere leg with
// supportsAnalyticSpherePatch refusing every loop (the body fell to the
// faceted writer).
func TestSTEPWritesCornerPatches(t *testing.T) {
	t.Parallel()
	up := r3.NewVec(0, 0, 1)
	t.Run("convex cone patches", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		rounded, err := decad.Fillet(t.Context(), boxBody(t, doc, 0, 0, 20, 20, 10),
			decad.Edges(decad.ParallelTo(up)).Exactly(4), units.Millimeters(3))
		require.NoError(t, err)
		top := planeFacing(t, rounded, up, r3.NewVec(0, 0, 10))
		body, err := decad.Chamfer(t.Context(), rounded, loopEdgeQuery(top.Loops()[0]), units.Millimeters(1))
		require.NoError(t, err)
		text := string(writeSTEP(t, body))
		require.Contains(t, text, "faceted decad solid")
		require.Zero(t, strings.Count(text, "=CONICAL_SURFACE("))
		require.Equal(t, strings.Count(text, "=ADVANCED_FACE("), strings.Count(text, "=PLANE("))

		curved := 0.0
		for _, f := range body.Faces() {
			switch f.Surface().(type) {
			case decad.Cone, decad.Cylinder:
				area, err := f.Area()
				require.NoError(t, err)
				curved += area.Value.Base()
			}
		}
		m, err := readSTEPBytes(t, []byte(text))
		require.NoError(t, err)
		require.True(t, m.Report().Closed, "the faceted file reads back closed")
		volume, err := m.Volume()
		require.NoError(t, err)
		require.InDelta(t, 3612+262*math.Pi/3, volume.Value.Base(), 0.1*curved)
	})
	t.Run("reflex cone patches", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		pocket, err := decad.Cut(t.Context(), boxBody(t, doc, 0, 0, 40, 40, 10), boxBodyAtZ(t, doc, 10, 15, 30, 25, 5, 5))
		require.NoError(t, err)
		mouth := planeFacing(t, pocket, up, r3.NewVec(0, 0, 10))
		body, err := decad.Chamfer(t.Context(), pocket, loopEdgeQuery(mouth.Loops()[1]), units.Millimeters(1))
		require.NoError(t, err)
		text := string(writeSTEP(t, body))
		require.Contains(t, text, "faceted decad solid")
		require.Zero(t, strings.Count(text, "=CONICAL_SURFACE("))
		require.Equal(t, strings.Count(text, "=ADVANCED_FACE("), strings.Count(text, "=PLANE("))
	})
	t.Run("sphere patches", func(t *testing.T) {
		t.Parallel()
		body, err := decad.Fillet(t.Context(), boxBody(t, decad.New(), 0, 0, 20, 20, 20), decad.Edges(), units.Millimeters(2))
		require.NoError(t, err)
		text := string(writeSTEP(t, body))
		require.Contains(t, text, "analytic decad solid")
		require.Equal(t, 26, strings.Count(text, "=ADVANCED_FACE("))
		require.Equal(t, 8, strings.Count(text, "=SPHERICAL_SURFACE("))
	})
}
