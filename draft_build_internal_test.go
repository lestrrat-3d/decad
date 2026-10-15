package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// draftSlot extrudes docs/draft-design.md F3's slot: lines 30 mm long joined
// by semicircles of radius 5 at G1 joins, swept 8 mm at a 3 degree taper.
func draftSlot(t *testing.T) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	slot, err := s.CreateSlot(-15, 0, 15, 0, 5)
	require.NoError(t, err)
	s.Fix(slot.C1)
	s.Fix(slot.C2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := Extrude(t.Context(), New(), s, s.Profiles()[0], Distance{D: units.Millimeters(8), Dir: Along}, WithTaper(units.Degrees(3)))
	require.NoError(t, err)
	return b
}

// TestDraftPayloadRecordsTheFarSection pins Table BD row BD1: the body's
// payload carries the far section the build made, one loop per near loop, and
// the positive displacement its built contour carries.
func TestDraftPayloadRecordsTheFarSection(t *testing.T) {
	t.Parallel()
	b := draftSlot(t)
	dp, ok := b.n.Payload.(draftPayload)
	require.True(t, ok)
	require.True(t, dp.NearStart)
	require.Greater(t, dp.D, 0.0)
	require.Greater(t, dp.FarDelta, 0.0)
	require.Len(t, dp.Far.Outer.Segments, 4)
	require.Empty(t, dp.Far.Holes)
	for _, seg := range dp.Far.Outer.Segments {
		if arc, ok := seg.(arcSeg); ok {
			require.InDelta(t, 5-dp.D, math.Hypot(arc.Start.U-arc.Center.U, arc.Start.V-arc.Center.V), 1e-12)
		}
	}
}

// TestDraftConeWallsReadNoChordLocusTerm pins §8's chord-versus-locus term at
// zero: both corners of each of the slot's Cone walls are G1 feet on the
// wall's own radials, so the band's proven corner skews are zero, and with
// them capband's chord-versus-locus allowance. It reads the patch geometry
// the build recorded from capblend.BuildBand.
func TestDraftConeWallsReadNoChordLocusTerm(t *testing.T) {
	t.Parallel()
	b := draftSlot(t)
	dp := b.n.Payload.(draftPayload)
	cones := 0
	for _, p := range dp.Patches {
		if !p.Geom.Circular {
			continue
		}
		cones++
		require.Zero(t, capblend.PatchWindowSkew(p.Geom))
	}
	require.Equal(t, 2, cones)
}
