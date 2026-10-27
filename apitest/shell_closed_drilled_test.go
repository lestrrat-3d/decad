package apitest_test

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

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

// TestShellClosedDrilledBarRefusals covers the thicknesses that collapse the
// ring, the axial limit both kept caps set, and the senses and receivers the
// closed shell of a holed prism does not take. Every refusal leaves the
// document as it was.
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
			name: "outward on a holed prism (SX8)", build: axialDrilledBar, th: 1,
			opts: []decad.ShellOption{decad.WithShellSense(decad.Outward)},
			is:   decad.ErrUnsupported, want: "modify-reach SX8",
		},
		{
			name: "outward on a brep (SG2)", build: crossBar, th: 1,
			opts: []decad.ShellOption{decad.WithShellSense(decad.Outward)},
			is:   decad.ErrUnsupported, want: "modify-general SG2",
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
