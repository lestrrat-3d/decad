package apitest_test

import (
	"io"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// closedShellBox shells the 100×60×20 plate with no openings.
func closedShellBox(t *testing.T, th float64, opts ...decad.ShellOption) (*decad.Document, *decad.Body) {
	t.Helper()
	doc, box := shellBox(t)
	closed, err := box.Shell(t.Context(), nil, units.Millimeters(th), append(opts, decad.WithNoOpenings())...)
	require.NoError(t, err)
	return doc, closed
}

// TestShellClosedPrism is modify-reach BX5: WithNoOpenings on a hole-free
// prism keeps every face and lines it with a wall, so the body is one lump of
// an outer shell around one void shell, the cavity.
func TestShellClosedPrism(t *testing.T) {
	t.Parallel()
	h := shellBoxHeight
	tests := []struct {
		th    float64
		sense decad.ShellSense
		// volume is the closed form; cavity the cavity shell's face count.
		volume    float64
		exact     bool
		cavity    int
		outer     int
		cavityLoZ float64
		cavityHiZ float64
	}{
		{
			// Inward 5 mm: the 100×60×20 box less the 90×50×10 cavity, every
			// coordinate a float.
			th: 5, sense: decad.Inward, volume: 100*60*h - 90*50*(h-10), exact: true,
			cavity: 6, outer: 6, cavityLoZ: 5, cavityHiZ: 15,
		},
		{
			// Outward 2 mm: a 104×64 box with radius-2 vertical edges, 24 tall,
			// around the original box as its cavity.
			th: 2, sense: decad.Outward, volume: (104*64-(4-math.Pi)*4)*(h+4) - 100*60*h,
			cavity: 6, outer: 8 + 2, cavityLoZ: 0, cavityHiZ: h,
		},
	}
	for _, tt := range tests {
		t.Run(tt.sense.String(), func(t *testing.T) {
			t.Parallel()
			doc, closed := closedShellBox(t, tt.th, decad.WithShellSense(tt.sense))
			require.True(t, closed.IsSolid())
			requireManifold(t, closed)
			lumps := closed.Lumps()
			require.Len(t, lumps, 1)
			shells := lumps[0].Shells()
			require.Len(t, shells, 2)
			require.False(t, shells[0].IsVoid(), `the outer shell comes first`)
			require.True(t, shells[1].IsVoid(), `the cavity is the void shell`)
			require.Len(t, shells[0].Faces(), tt.outer)
			require.Len(t, shells[1].Faces(), tt.cavity)

			vol, err := closed.Volume()
			require.NoError(t, err)
			if tt.exact {
				require.Equal(t, decad.Exact, vol.Exactness)
				require.Equal(t, tt.volume, volumeMM(t, vol))
			}
			requireVolumeNear(t, closed, tt.volume)

			// The cavity's floor faces up into it and its ceiling down.
			for _, f := range shells[1].Faces() {
				plane, ok := f.Surface().(decad.Plane)
				if !ok {
					continue
				}
				n := plane.Frame.N()
				if math.Abs(n.Z) < 0.5 {
					continue
				}
				z := plane.Frame.Origin().Z
				switch {
				case n.Z > 0:
					require.InDelta(t, tt.cavityLoZ, z, 1e-12, `the floor sits at the cavity's bottom`)
				default:
					require.InDelta(t, tt.cavityHiZ, z, 1e-12, `the ceiling sits at the cavity's top`)
				}
			}

			// Both caps stay selectable by their roles.
			for _, cap := range []decad.FeatureRef{decad.CapStart(closed), decad.CapEnd(closed)} {
				faces, err := decad.Faces(decad.FaceCreatedBy(cap)).SelectFaces(closed)
				require.NoError(t, err)
				require.Len(t, faces, 1)
			}

			mesh, err := closed.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			requireWatertight(t, mesh)
			require.True(t, mesh.VolumeVerified())
			// The chorded corner arcs (outward only) inscribe their curves, so
			// the mesh may under-enclose by at most their chord deficit over
			// the outer height.
			mv := meshVolume(mesh)
			require.Less(t, mv, tt.volume+1e-6)
			require.Greater(t, mv, tt.volume-2*math.Pi*tt.th*0.05*(h+2*tt.th)-1e-6)

			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)

			rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
			require.NoError(t, err)
			placed, err := closed.Placed(t.Context(), rotation)
			require.NoError(t, err)
			require.Len(t, placed.Lumps(), 1)
			require.Len(t, placed.Shells(), 2)
			require.True(t, placed.Shells()[1].IsVoid())
			requireVolumeNear(t, placed, tt.volume)
		})
	}
}

// TestShellClosedPrismCylinder hollows a radius-10, 20 mm cylinder with a
// 2 mm inward wall: π·(10²·20 − 8²·16).
func TestShellClosedPrismCylinder(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	rod, err := decad.New().Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	closed, err := rod.Shell(t.Context(), nil, units.Millimeters(2), decad.WithNoOpenings())
	require.NoError(t, err)
	require.Len(t, closed.Shells(), 2)
	require.Len(t, closed.Shells()[1].Faces(), 3, `a cylinder wall, a floor and a ceiling`)
	requireVolumeNear(t, closed, math.Pi*(100*20-64*16))
	mesh, err := closed.Tessellate(t.Context(), units.Millimeters(0.01))
	require.NoError(t, err)
	requireWatertight(t, mesh)
}

// TestShellClosedPrismGatesAndConsumers covers the closed prism shell's own
// gates and what later operations do with it.
func TestShellClosedPrismGatesAndConsumers(t *testing.T) {
	t.Parallel()
	t.Run("two kept caps eat the sweep (SX11)", func(t *testing.T) {
		// 10 mm < the inradius 30, so the section survives, but the two
		// 10 mm caps meet in the 20 mm sweep.
		doc, box := shellBox(t)
		_, err := box.Shell(t.Context(), nil, units.Millimeters(10), decad.WithNoOpenings())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "SX11")
		require.Equal(t, []*decad.Body{box}, doc.Bodies())
	})
	t.Run("t past the inradius (S10)", func(t *testing.T) {
		_, box := shellBox(t)
		_, err := box.Shell(t.Context(), nil, units.Millimeters(35), decad.WithNoOpenings())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.NotContains(t, err.Error(), "SX11")
	})
	t.Run("an outward wall has no cavity limit", func(t *testing.T) {
		_, box := shellBox(t)
		closed, err := box.Shell(t.Context(), nil, units.Millimeters(12), decad.WithNoOpenings(), decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		require.Len(t, closed.Shells(), 2)
	})
	t.Run("a modify op on the closed shell has no face view", func(t *testing.T) {
		doc, closed := closedShellBox(t, 5)
		_, err := closed.Fillet(t.Context(), verticalEdges(), units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "cavity")
		require.Equal(t, []*decad.Body{closed}, doc.Bodies())
	})
	t.Run("a through cut opens the cavity", func(t *testing.T) {
		// A 10×10 bar through both kept caps removes 10·10·(5 + 5) of plate
		// and opens the cavity to the outside.
		doc, closed := closedShellBox(t, 5)
		bar := boxBodyAtZ(t, doc, 45, 25, 55, 35, -1, shellBoxHeight+2)
		cut, err := decad.Cut(t.Context(), closed, bar)
		require.NoError(t, err)
		requireVolumeNear(t, cut, 75000-1000)
		for _, sh := range cut.Shells() {
			require.False(t, sh.IsVoid(), `the opened cavity is no void`)
		}
	})
	t.Run("STEP refuses the void shell", func(t *testing.T) {
		_, closed := closedShellBox(t, 5)
		err := export.STEP(t.Context(), io.Discard, closed, units.Millimeters(0.1),
			export.WithSTEPName("closed"), export.WithSTEPAuthor("apitest"), export.WithSTEPOrganization("decad"))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Contains(t, err.Error(), "single non-void shell")
	})
	t.Run("the wall survey stays staged", func(t *testing.T) {
		doc, _ := closedShellBox(t, 5)
		report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
		require.NoError(t, err)
		require.Equal(t, decad.Suspect, report.Status)
		_, staged := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedSurveyPayload)
		require.True(t, staged)
	})
}
