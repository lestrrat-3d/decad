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

// TestSTEPWritesCornerPatchesAnalytic writes, through the public API, three
// bodies whose corners are single-loop cone and sphere patches
// (agent-docs/step-export-design.md), and requires the analytic writer with
// one ADVANCED_FACE per body face.
//
// The rounded box (20x20x10, vertical edges filleted r = 3, top loop
// chamfered by 1) holds four cone patches, each two arcs and two generators.
// Its volume is 9·(400 − 9(4 − π)) + ∫₀¹ (20 − 2d)² − (4 − π)(3 − d)² dd =
// 3612 + 262π/3. ReadSTEP reads the file back closed; its arcs are chords
// inscribed in their circles within 0.01 mm on a convex body, so the read
// volume lies under the closed form and within 0.01 mm times the curved
// faces' area of it. The pocket mouth chamfer holds four three-edge cone
// patches meeting at their apex vertices, and the cube filleted r = 2 on
// every edge holds eight three-arc sphere patches; ReadSTEP reads neither a
// cone's apex nor a sphere patch other than a cap, so those two files are
// checked by their surfaces. Shown to fail with supportsAnalyticConePatch and
// supportsAnalyticSpherePatch each refusing every loop (the bodies fell to
// the faceted writer), and with the cone patch's face sense inverted (ReadSTEP
// turned the cone triangles inward and the read volume left its bound).
func TestSTEPWritesCornerPatchesAnalytic(t *testing.T) {
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
		require.Contains(t, text, "analytic decad solid")
		require.Equal(t, len(body.Faces()), strings.Count(text, "=ADVANCED_FACE("))
		require.Equal(t, 4, strings.Count(text, "=CONICAL_SURFACE("))

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
		require.True(t, m.Report().Closed, "the cone patches' chords meet their neighbours'")
		volume, err := m.Volume()
		require.NoError(t, err)
		exact := 3612 + 262*math.Pi/3
		require.Less(t, volume.Value.Base(), exact)
		require.Greater(t, volume.Value.Base(), exact-0.01*curved)
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
		require.Contains(t, text, "analytic decad solid")
		require.Equal(t, len(body.Faces()), strings.Count(text, "=ADVANCED_FACE("))
		require.Equal(t, 4, strings.Count(text, "=CONICAL_SURFACE("))
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
