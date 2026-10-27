package apitest_test

import (
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestSTEPRefusesAnOffsetToFaceCap extrudes a 4 x 3 rectangle 10 mm as a
// plate, then the same profile up to the plate's top cap offset by 0.1 mm.
// The pin's top cap is a Plane held at z = 10.1, while the level it stands for
// is 10 + float(0.1), which no float holds: its corner vertices read
// Approximate with a positive bound, and the cap states a nonzero axial
// displacement from its tag. STEP writes the pin faceted (one PLANE per
// ADVANCED_FACE), and ReadSTEP reads it back as the 4 x 3 x 10.1 block. The
// plate, whose every level is exact, writes analytic: six planes. Shown to
// fail with facedeparture.TagDeparts' AxialDelta leg deleted (the pin was
// written analytic).
func TestSTEPRefusesAnOffsetToFaceCap(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 4, 3)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	plate, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	pin, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.ToFace{
		Body:   plate,
		Face:   capEndFace(plate),
		Offset: units.Millimeters(0.1),
	})
	require.NoError(t, err)

	top := 0
	for _, v := range pin.Vertices() {
		p := v.Position()
		if p.Value.Z != 10.1 {
			continue
		}
		require.Equal(t, decad.Approximate, p.Exactness)
		require.Positive(t, p.Bound.Base())
		top++
	}
	require.Equal(t, 4, top, "the pin's four top corners sit at the held level")

	text := string(writeSTEP(t, pin))
	require.Contains(t, text, "faceted decad solid")
	require.Equal(t, strings.Count(text, "=ADVANCED_FACE("), strings.Count(text, "=PLANE("))
	m, err := readSTEPBytes(t, []byte(text))
	require.NoError(t, err)
	requireMeshVolume(t, m, 4*3*10.1)

	plateText := string(writeSTEP(t, plate))
	require.Contains(t, plateText, "analytic decad solid")
	require.Equal(t, 6, strings.Count(plateText, "=ADVANCED_FACE("))
}
