package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// crossingPocketTool extrudes the x -1..3, y -1..3 rectangle from the plane
// z = planeZ by depth in the given direction. Against a 20x20x10 plate with a
// 4x4 through hole at x -4..0, y -2..2, the rectangle overlaps the hole in
// 1x3 mm, so each millimetre of pocket depth inside the plate removes
// 4*4 - 1*3 = 13 mm^3 beyond the through hole.
func crossingPocketTool(t *testing.T, doc *decad.Document, planeZ float64, depth units.Value, dir decad.Direction) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), planeZ)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(-1, -1, 3, 3)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: depth, Dir: dir})
	require.NoError(t, err)
	return body
}

func crossingHolePlate(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	plate := boxBody(t, doc, -10, -10, 10, 10, 10)
	through := boxBodyAtZ(t, doc, -4, -2, 0, 2, 0, 10)
	part, err := decad.Cut(t.Context(), plate, through)
	require.NoError(t, err)
	return part
}

// crossingHolePlateOfHeight is the same 20x20 plate with the 4x4 through hole,
// extruded to a height that may be an inch conversion, so its top level
// carries a displacement. The through tool overshoots both caps.
func crossingHolePlateOfHeight(t *testing.T, doc *decad.Document, height units.Value, toolHeight float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-10, -10, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	plate, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: height, Dir: decad.Along})
	require.NoError(t, err)
	part, err := decad.Cut(t.Context(), plate, boxBodyAtZ(t, doc, -4, -2, 0, 2, -1, toolHeight))
	require.NoError(t, err)
	return part
}

// TestCrossingBlindPocketExtensions cuts a pocket across a through hole when
// the pocket opens at the plate's bottom, overshoots a cap, or ends at a level
// that carries an inch-conversion displacement. The plate is 4000 mm^3 and the
// through hole removes 160, so a pocket reaching depth d into the plate
// removes a further 13*d.
func TestCrossingBlindPocketExtensions(t *testing.T) {
	t.Parallel()
	const base = 4000.0 - 160
	mm := units.Millimeters
	in := units.Inches
	for _, tc := range []struct {
		name  string
		build func(*testing.T, *decad.Document) *decad.Body
		depth float64
		// displaced is true when the pocket level is an inch conversion: it
		// carries a displacement, so the volume is bounded, not exact.
		displaced bool
	}{
		{"bottom-start", func(t *testing.T, d *decad.Document) *decad.Body {
			return crossingPocketTool(t, d, 0, mm(4), decad.Along)
		}, 4, false},
		{"top-overshoot", func(t *testing.T, d *decad.Document) *decad.Body {
			return crossingPocketTool(t, d, 6, mm(8), decad.Along)
		}, 4, false},
		{"bottom-overshoot", func(t *testing.T, d *decad.Document) *decad.Body {
			return crossingPocketTool(t, d, 4, mm(8), decad.Against)
		}, 4, false},
		{"inch-depth-from-top", func(t *testing.T, d *decad.Document) *decad.Body {
			return crossingPocketTool(t, d, 10, in(0.2), decad.Against)
		}, 5.08, true},
		{"inch-depth-from-bottom", func(t *testing.T, d *decad.Document) *decad.Body {
			return crossingPocketTool(t, d, 0, in(0.2), decad.Along)
		}, 5.08, true},
		{"inch-depth-overshoot", func(t *testing.T, d *decad.Document) *decad.Body {
			return crossingPocketTool(t, d, 6, in(0.5), decad.Along)
		}, 4, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			part, err := decad.Cut(t.Context(), crossingHolePlate(t, doc), tc.build(t, doc))
			require.NoError(t, err)
			require.False(t, tookMeshPath(part), "class %s", decadtest.PayloadClass(part))
			requireEveryEdgeOnTwoFaces(t, part)
			volume, err := part.Volume()
			require.NoError(t, err)
			if tc.displaced {
				require.Equal(t, decad.Approximate, volume.Exactness)
				require.Greater(t, volume.Bound.Base(), 0.0)
			} else {
				require.Equal(t, decad.Exact, volume.Exactness)
			}
			require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-(base-13*tc.depth)), volume.Bound.Base()+1e-9)
			mesh, err := part.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.VolumeVerified())
			requireWatertight(t, mesh)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)
		})
	}
}

// TestCrossingBlindPocketOnDisplacedPlate cuts a millimetre pocket across the
// hole of a plate whose top is an inch conversion. The pocket's top lies above
// the plate's top by more than the conversion's displacement, so the order is
// proven and the volume is bounded, not exact. The second case sits above the
// plate's top by about 9e-16 mm, which is only three times the displacement.
func TestCrossingBlindPocketOnDisplacedPlate(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		height    float64
		plate     units.Value
		toolDepth float64
	}{
		{"pocket-well-above-top", 7.62, units.Inches(0.3), 6},
		{"pocket-just-above-top", 7.25, units.Inches(7.25 / 25.4), 1.25},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := crossingHolePlateOfHeight(t, doc, tc.plate, tc.height+2)
			pocket := crossingPocketTool(t, doc, 6, units.Millimeters(tc.toolDepth), decad.Along)
			part, err := decad.Cut(t.Context(), plate, pocket)
			require.NoError(t, err)
			require.False(t, tookMeshPath(part), "class %s", decadtest.PayloadClass(part))
			requireEveryEdgeOnTwoFaces(t, part)
			volume, err := part.Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, volume.Exactness)
			require.Greater(t, volume.Bound.Base(), 0.0)
			want := (400.0-16)*tc.height - 13*(tc.height-6)
			require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-want), volume.Bound.Base()+1e-9)
			mesh, err := part.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.VolumeVerified())
			requireWatertight(t, mesh)
		})
	}
}

// TestCrossingBlindPocketDeclinesUnprovenLevels cuts pockets whose end level
// is not provably ordered against a plate cap. Each plate is built through an
// inch conversion, so its top level carries a displacement. The millimetre
// pocket ends at the top the conversion denotes, which is equal with a
// displacement, and the inch pocket ends inside the plate's displacement. The
// crossing route does not build, and the pair never becomes an exact brep.
func TestCrossingBlindPocketDeclinesUnprovenLevels(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name     string
		height   float64
		inchTool bool
	}{
		// 7.5 mm denotes exactly 7.5 with a displacement of 3.6e-16.
		{"equal-with-displacement", 7.5, false},
		// Both tops are inch conversions 4.4e-16 mm apart, inside their displacements.
		{"unequal-within-displacement", 9.25, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := crossingHolePlateOfHeight(t, doc, units.Inches(tc.height/25.4), tc.height+2)
			depth := units.Millimeters(tc.height - 6)
			if tc.inchTool {
				depth = units.Inches((tc.height - 6) / 25.4)
			}
			pocket := crossingPocketTool(t, doc, 6, depth, decad.Along)
			part, err := decad.Cut(t.Context(), plate, pocket)
			if err != nil {
				return
			}
			require.True(t, tookMeshPath(part), "class %s", decadtest.PayloadClass(part))
		})
	}
}
