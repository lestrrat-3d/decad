package apitest_test

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// polygonHoledPrism extrudes the 20×20 square [0, 20]² by 10 along z, less
// the polygon hole through the fixed vertices pts.
func polygonHoledPrism(t *testing.T, pts [][2]float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 20, 20)
	s.Fix(rect.A)
	s.Fix(rect.C)
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
		s.Fix(sp[i])
	}
	for i := range sp {
		s.CreateLine(sp[i], sp[(i+1)%len(sp)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			body, err := decad.Extrude(t.Context(), decad.New(), s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			require.NoError(t, err)
			return body
		}
	}
	require.FailNow(t, `the sketch holds no holed profile`)
	return nil
}

// lHoledPrism holds an L-shaped hole, [5, 15]×[5, 9] ∪ [5, 9]×[5, 15]: area
// 64, perimeter 40, five convex corners and one reflex corner at (9, 9).
func lHoledPrism(t *testing.T) *decad.Body {
	t.Helper()
	return polygonHoledPrism(t, [][2]float64{{5, 5}, {15, 5}, {15, 9}, {9, 9}, {9, 15}, {5, 15}})
}

// dumbbellHoledPrism holds two 5×11 hole lobes, x in [3, 8] and [12, 17],
// joined by a 1-wide waist at y in [9, 10].
func dumbbellHoledPrism(t *testing.T) *decad.Body {
	t.Helper()
	return polygonHoledPrism(t, [][2]float64{
		{3, 3}, {8, 3}, {8, 9}, {12, 9}, {12, 3}, {17, 3},
		{17, 14}, {12, 14}, {12, 10}, {8, 10}, {8, 14}, {3, 14},
	})
}

// axialDrilledBar is the 20×20×10 bar centred on the z axis with a radius-3
// hole drilled through it along z: Cut builds it as a holed prism.
func axialDrilledBar(t *testing.T) *decad.Body {
	t.Helper()
	doc := decad.New()
	box := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 10)
	rod := circleBodyAtZ(t, doc, 0, 3, -1, 12)
	drilled, err := decad.Cut(t.Context(), box, rod)
	require.NoError(t, err)
	return drilled
}

// crossBar is modify-general §1's P1 (crossDrilledBar): the 40×20×20 box
// with a Ø6 hole along y through (20, ·, 10). Cut builds it as a brep record,
// which reads as a prism along y.
func crossBar(t *testing.T) *decad.Body {
	t.Helper()
	_, bar := crossDrilledBar(t)
	require.False(t, tookMeshPath(bar), `the cross drill builds a brep body`)
	return bar
}

// TestShellClosedDrilledBar is modify-general §3.2's closed shell of a holed
// prism: WithNoOpenings keeps every face, lines the outside with an inset wall
// and the hole with a tube of radius r + t, and closes the ring between them
// into one void shell.
func TestShellClosedDrilledBar(t *testing.T) {
	t.Parallel()
	tests := []struct {
		name  string
		build func(*testing.T) *decad.Body
		// volume is the closed form: the drilled bar less the ring cavity,
		// the inset section less the dilated hole over h − 2t.
		volume float64
		// curved is the circumference of the two cylinders times their
		// length, which bounds the chorded mesh's volume deficit per unit of
		// tolerance.
		curved float64
	}{
		{
			name: "axial drill (holed prism)", build: axialDrilledBar,
			volume: (400-9*math.Pi)*10 - (18*18-16*math.Pi)*8,
			curved: 2 * math.Pi * (3 + 4) * 10,
		},
		{
			name: "cross drill (brep)", build: crossBar,
			volume: (800-9*math.Pi)*20 - (38*18-16*math.Pi)*18,
			curved: 2 * math.Pi * (3 + 4) * 20,
		},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			bar := tc.build(t)
			doc := bar.Document()
			closed, err := decad.Shell(t.Context(), bar, nil, units.Millimeters(1), decad.WithNoOpenings())
			require.NoError(t, err)
			require.True(t, closed.IsSolid())
			requireManifold(t, closed)
			requireVolumeNear(t, closed, tc.volume)

			// Tessellation runs the per-shell winding audit, so it refuses a body
			// whose cavity reads as an inside-out lump.
			mesh, err := closed.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			requireWatertight(t, mesh)
			require.True(t, mesh.VolumeVerified())
			require.InDelta(t, tc.volume, meshVolume(mesh), tc.curved*0.05)

			lumps := closed.Lumps()
			require.Len(t, lumps, 1)
			shells := lumps[0].Shells()
			require.Len(t, shells, 2)
			require.False(t, shells[0].IsVoid(), `the outer shell comes first`)
			require.True(t, shells[1].IsVoid(), `the ring around the hole's lining is the void shell`)
			// The outer shell keeps the bar's six faces and its hole wall; the
			// cavity holds four inset walls, the lining cylinder, a floor and a
			// ceiling, both ring-shaped.
			require.Len(t, shells[0].Faces(), 7)
			require.Len(t, shells[1].Faces(), 7)
			rings := 0
			for _, f := range shells[1].Faces() {
				if _, ok := f.Surface().(decad.Plane); ok && len(f.Loops()) == 2 {
					rings++
				}
			}
			require.Equal(t, 2, rings, `the cavity's floor and ceiling are each a region with one hole`)

			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)

			data := string(writeSTEP(t, closed))
			require.Equal(t, 1, strings.Count(data, "BREP_WITH_VOIDS("))
			require.Equal(t, 0, strings.Count(data, "MANIFOLD_SOLID_BREP("))
			read, err := readSTEPBytes(t, []byte(data))
			require.NoError(t, err)
			require.True(t, read.Report().Closed)
			require.Equal(t, 2, read.Report().Shells, `the outer skin and the ring cavity`)
			held, err := read.Volume()
			require.NoError(t, err)
			require.InDelta(t, tc.volume, held.Value.Base(), tc.curved*0.1)
		})
	}
}

// TestShellClosedDrilledBarOutward is reach §9.2's outward closed shell of a
// holed prism: the outer wall is the section dilated by t (each convex edge
// along the hole's axis rounded with radius t, the hole shrunk to r − t), the
// caps grow t beyond each end, and the cavity is the drilled bar itself, a
// ring around the hole's lining, which now grows into the hole.
func TestShellClosedDrilledBarOutward(t *testing.T) {
	t.Parallel()
	tests := []struct {
		name  string
		build func(*testing.T) *decad.Body
		// volume is the closed form: the dilated section over h + 2t less the
		// drilled bar. The dilation of the w×d section with a radius-3 hole by
		// 1 is w·d + 2(w + d) + π less the radius-2 hole.
		volume float64
		// curved is the circumference of the curved walls times their length.
		curved float64
		// outer and void are the face counts of the outer and void shells.
		outer, void int
	}{
		{
			name: "axial drill (holed prism)", build: axialDrilledBar,
			volume: (480+math.Pi-4*math.Pi)*12 - (400-9*math.Pi)*10,
			curved: 2*math.Pi*(2+3)*12 + 2*math.Pi*12,
			// Four flat and four rounded walls, two caps and the shrunk
			// hole's wall; the cavity holds the bar's four walls, the
			// lining's outer cylinder (the original hole wall), a floor and
			// a ceiling.
			outer: 11, void: 7,
		},
		{
			name: "cross drill (brep)", build: crossBar,
			volume: (920+math.Pi-4*math.Pi)*22 - (800-9*math.Pi)*20,
			curved: 2*math.Pi*(2+3)*22 + 2*math.Pi*22,
			outer: 11, void: 7,
		},
		{
			// The L hole erodes to [6, 14]×[6, 8] ∪ [6, 8]×[6, 14] with its
			// reflex corner rounded by a radius-1 arc about (9, 9), the
			// material's convex corner there: area 28 + (1 − π/4). So the
			// volume is (480 + π − 29 + π/4)·12 − (400 − 64)·10.
			name: "L-shaped hole", build: lHoledPrism,
			volume: 2052 + 15*math.Pi,
			curved: 2*math.Pi*12 + math.Pi/2*12,
			// The outer shell's eroded hole holds six walls and the arc wall.
			outer: 17, void: 12,
		},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			bar := tc.build(t)
			doc := bar.Document()
			closed, err := decad.Shell(t.Context(), bar, nil, units.Millimeters(1),
				decad.WithNoOpenings(), decad.WithShellSense(decad.Outward))
			require.NoError(t, err)
			require.True(t, closed.IsSolid())
			requireManifold(t, closed)
			requireVolumeNear(t, closed, tc.volume)

			mesh, err := closed.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			requireWatertight(t, mesh)
			require.True(t, mesh.VolumeVerified())
			require.InDelta(t, tc.volume, meshVolume(mesh), tc.curved*0.05)

			lumps := closed.Lumps()
			require.Len(t, lumps, 1)
			shells := lumps[0].Shells()
			require.Len(t, shells, 2)
			require.False(t, shells[0].IsVoid(), `the outer shell comes first`)
			require.True(t, shells[1].IsVoid(), `the drilled bar's own volume is the void shell`)
			require.Len(t, shells[0].Faces(), tc.outer)
			require.Len(t, shells[1].Faces(), tc.void)
			rings := 0
			for _, f := range shells[1].Faces() {
				if _, ok := f.Surface().(decad.Plane); ok && len(f.Loops()) == 2 {
					rings++
				}
			}
			require.Equal(t, 2, rings, `the cavity's floor and ceiling are each the bar's holed section`)

			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)

			data := string(writeSTEP(t, closed))
			require.Equal(t, 1, strings.Count(data, "BREP_WITH_VOIDS("))
			require.Equal(t, 0, strings.Count(data, "MANIFOLD_SOLID_BREP("))
			read, err := readSTEPBytes(t, []byte(data))
			require.NoError(t, err)
			require.True(t, read.Report().Closed)
			require.Equal(t, 2, read.Report().Shells, `the outer skin and the ring cavity`)
			held, err := read.Volume()
			require.NoError(t, err)
			require.InDelta(t, tc.volume, held.Value.Base(), tc.curved*0.1)
		})
	}
}

// TestShellClosedDrilledBarRefusals covers the thicknesses that collapse the
// ring, the axial limit both kept caps set, and the outward thicknesses that
// fill the hole in. Every refusal leaves the document as it was.
func TestShellClosedDrilledBarRefusals(t *testing.T) {
	t.Parallel()
	tests := []struct {
		name  string
		build func(*testing.T) *decad.Body
		th    float64
		opts  []decad.ShellOption
		is    error
		want  string
	}{
		{
			// The lining's radius 3 + 4 passes the inset wall at 10 − 4: the
			// cavity's hole crosses its outer loop and the ring splits, which
			// the offset audit refuses (S11).
			name: "the lining meets the inset wall", build: axialDrilledBar, th: 4,
			is: decad.ErrUnsupported, want: "crosses itself",
		},
		{
			// 5 exceeds the section's inradius (13/(1+√2/2) − 3, about 4.6).
			name: "t past the inradius (S10)", build: axialDrilledBar, th: 5,
			is: decad.ErrDegenerate, want: "inradius",
		},
		{
			// Across the bar the lining's top at 10 + 3 + 4 passes the inset
			// top at 20 − 4.
			name: "the cross lining meets the inset top", build: crossBar, th: 4,
			is: decad.ErrUnsupported, want: "crosses itself",
		},
		{
			// Outward the radius-3 hole erodes to radius 3 − t: at t = 3 it
			// fills in, a feature the offset drops (S11a).
			name: "outward fills the hole (S11a)", build: axialDrilledBar, th: 3,
			opts: []decad.ShellOption{decad.WithShellSense(decad.Outward)},
			is:   decad.ErrUnsupported, want: "drops a section feature",
		},
		{
			name: "outward past the hole's radius (S11a)", build: axialDrilledBar, th: 4,
			opts: []decad.ShellOption{decad.WithShellSense(decad.Outward)},
			is:   decad.ErrUnsupported, want: "drops a section feature",
		},
		{
			name: "outward fills the cross hole (S11a)", build: crossBar, th: 3,
			opts: []decad.ShellOption{decad.WithShellSense(decad.Outward)},
			is:   decad.ErrUnsupported, want: "drops a section feature",
		},
		{
			// The 1-wide waist erodes away at t = 1, splitting the hole in
			// two: the offset walls of the waist cross (S11b).
			name: "outward splits a dumbbell hole (S11b)", build: dumbbellHoledPrism, th: 1,
			opts: []decad.ShellOption{decad.WithShellSense(decad.Outward)},
			is:   decad.ErrUnsupported, want: "the rewrite crosses itself",
		},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			bar := tc.build(t)
			doc := bar.Document()
			_, err := decad.Shell(t.Context(), bar, nil, units.Millimeters(tc.th), append(tc.opts, decad.WithNoOpenings())...)
			require.ErrorIs(t, err, tc.is)
			if tc.want != "" {
				require.ErrorContains(t, err, tc.want)
			}
			require.Equal(t, []*decad.Body{bar}, doc.Bodies())
		})
	}
	t.Run("two kept caps eat the sweep (SX11)", func(t *testing.T) {
		t.Parallel()
		// A 40×40 bar 4 tall with a radius-3 hole: 2 mm leaves no cavity
		// between the caps, though the section survives it.
		doc := decad.New()
		box := boxBodyAtZ(t, doc, -20, -20, 20, 20, 0, 4)
		rod := circleBodyAtZ(t, doc, 0, 3, -1, 6)
		bar, err := decad.Cut(t.Context(), box, rod)
		require.NoError(t, err)
		_, err = decad.Shell(t.Context(), bar, nil, units.Millimeters(2), decad.WithNoOpenings())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "SX11")
		require.Equal(t, []*decad.Body{bar}, doc.Bodies())
	})
}

// TestShellClosedDrilledBarThroughCut cuts the axial drilled bar's closed
// shell with a radius-0.3 rod along z at x = 9.5, wholly inside the outer wall
// band, so the result keeps every face of the shell and loses only the rod's
// π·0.3²·10. The stacked through-cut arms cut one region per slab, and the
// closed shell's wall slab holds the outer band beside the hole's lining, so
// they leave the pair to the mesh path, whose verified volume bound keeps the
// lining: one lump, an outer shell around the ring cavity.
func TestShellClosedDrilledBarThroughCut(t *testing.T) {
	t.Parallel()
	bar := axialDrilledBar(t)
	doc := bar.Document()
	closed, err := decad.Shell(t.Context(), bar, nil, units.Millimeters(1), decad.WithNoOpenings())
	require.NoError(t, err)
	rod := circleBodyAtZ(t, doc, 9.5, 0.3, -1, 12)
	cut, err := decad.Cut(t.Context(), closed, rod)
	require.NoError(t, err)
	want := (400-9*math.Pi)*10 - (18*18-16*math.Pi)*8 - 0.09*math.Pi*10
	vol, err := cut.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), vol.Bound.Base()+1e-9,
		`volume %v must lie within its bound %v of the closed form %v`, volumeMM(t, vol), vol.Bound.Base(), want)
	require.Less(t, vol.Bound.Base(), 7*math.Pi*8, `the bound must not cover the lining's own volume`)
	require.True(t, tookMeshPath(cut))
	require.Len(t, cut.Lumps(), 1)
	shells := cut.Shells()
	require.Len(t, shells, 2)
	require.False(t, shells[0].IsVoid())
	require.True(t, shells[1].IsVoid(), `the ring cavity stays closed`)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

// TestShellClosedBoxThroughCut is the hole-free counterpart: the 20×20×10
// box's closed shell keeps one region per slab, so the same rod at x = 9.5,
// inside the wall, takes the exact stacked through cut, which copies the
// closed shell's record and keeps the cavity as its one void shell.
func TestShellClosedBoxThroughCut(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 10)
	closed, err := decad.Shell(t.Context(), box, nil, units.Millimeters(1), decad.WithNoOpenings())
	require.NoError(t, err)
	rod := circleBodyAtZ(t, doc, 9.5, 0.3, -1, 12)
	cut, err := decad.Cut(t.Context(), closed, rod)
	require.NoError(t, err)
	require.False(t, tookMeshPath(cut), `the stacked through cut builds the result exactly`)
	requireVolumeNear(t, cut, 400*10-18*18*8-0.09*math.Pi*10)
	require.Len(t, cut.Lumps(), 1)
	shells := cut.Shells()
	require.Len(t, shells, 2)
	require.False(t, shells[0].IsVoid())
	require.True(t, shells[1].IsVoid(), `the cavity stays closed`)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}
