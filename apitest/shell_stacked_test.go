package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// twoHolePlate extrudes the 100×60 plate by shellBoxHeight with a 10×20
// rectangular hole at (20, 20)-(30, 40) and a radius-8 round hole at
// (70, 30), in that ProfileRecord order.
func twoHolePlate(t *testing.T) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateRectangle(20, 20, 30, 40)
	s.CreateCircle(s.CreatePoint(70, 30), 8)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 2 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	body, err := decad.New().Extrude(s, prof, decad.Distance{D: units.Millimeters(shellBoxHeight), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// requireVolumeNear asserts the body's volume lies within its own published
// bound of want, widened by a few ulps of want for the closed form's own
// float evaluation.
func requireVolumeNear(t *testing.T, b *decad.Body, want float64) {
	t.Helper()
	vol, err := b.Volume()
	require.NoError(t, err)
	got := volumeMM(t, vol)
	require.LessOrEqual(t, math.Abs(got-want), vol.Bound.Base()+1e-12*want,
		`volume %v must lie within its bound %v of the closed form %v`, got, vol.Bound.Base(), want)
}

// TestShellBothCapsHoledBands is modify-reach BX8: removing both caps of a
// section with k = 2 holes leaves 1 + k disjoint wall bands — the band inside
// the outer loop, then one band lining each hole in ProfileRecord order — each
// its own lump with its own two rims, under one stacked slab.
func TestShellBothCapsHoledBands(t *testing.T) {
	t.Parallel()
	h := shellBoxHeight
	tests := []struct {
		th    float64
		sense decad.ShellSense
		// bands are each band's closed-form section area, outer band first.
		bands []float64
		faces []int
	}{
		{
			// Inward 3 mm: the outer loop insets to a sharp 94×54; the
			// rectangular hole grows to a 16×26 rectangle with radius-3 corners;
			// the round hole grows to radius 11.
			th: 3, sense: decad.Inward,
			bands: []float64{100*60 - 94*54, 16*26 - (4-math.Pi)*9 - 10*20, math.Pi * (11*11 - 8*8)},
			faces: []int{4 + 4 + 2, 8 + 4 + 2, 1 + 1 + 2},
		},
		{
			// Outward 2 mm: the outer loop grows to a 104×64 rectangle with
			// radius-2 corners; the rectangular hole shrinks to a sharp 6×16;
			// the round hole shrinks to radius 6.
			th: 2, sense: decad.Outward,
			bands: []float64{104*64 - (4-math.Pi)*4 - 100*60, 10*20 - 6*16, math.Pi * (8*8 - 6*6)},
			faces: []int{8 + 4 + 2, 4 + 4 + 2, 1 + 1 + 2},
		},
	}
	for _, tt := range tests {
		t.Run(tt.sense.String(), func(t *testing.T) {
			t.Parallel()
			box := twoHolePlate(t)
			body, err := box.Shell(t.Context(), bothCaps(), units.Millimeters(tt.th), decad.WithShellSense(tt.sense))
			require.NoError(t, err)
			require.True(t, body.IsSolid())
			requireManifold(t, body)

			rimRoles := map[string]struct{}{decad.CapStart(body).Role: {}, decad.CapEnd(body).Role: {}}
			lumps := body.Lumps()
			require.Len(t, lumps, 3, `1 + k disjoint bands`)
			total := 0.0
			for m, lump := range lumps {
				shells := lump.Shells()
				require.Len(t, shells, 1)
				require.False(t, shells[0].IsVoid())
				faces := shells[0].Faces()
				require.Len(t, faces, tt.faces[m], `band %d's walls and its two rims`, m)
				// Outer band first, then the hole linings in record order: every
				// wall of lump m names region m.
				var rims int
				for _, f := range faces {
					role := f.Origins()[0].Role
					if _, rim := rimRoles[role]; rim {
						rims++
						continue
					}
					require.Contains(t, role, fmt.Sprintf("slab(0).region(%d).side(", m))
				}
				require.Equal(t, 2, rims, `band %d carries its own two rims`, m)
				total += tt.bands[m]
			}
			requireVolumeNear(t, body, total*h)

			// Every band's rims answer CapStart / CapEnd, one per lump.
			for _, cap := range []decad.FeatureRef{decad.CapStart(body), decad.CapEnd(body)} {
				rims, err := decad.Faces(decad.FaceCreatedBy(cap)).SelectFaces(body)
				require.NoError(t, err)
				require.Len(t, rims, 3)
			}

			mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			requireWatertight(t, mesh)
			require.True(t, mesh.VolumeVerified())

			rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
			require.NoError(t, err)
			placed, err := body.Placed(t.Context(), rotation)
			require.NoError(t, err)
			require.Len(t, placed.Lumps(), 3)
			require.Len(t, placed.Faces(), len(body.Faces()))
			requireVolumeNear(t, placed, total*h)
			placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.05))
			require.NoError(t, err)
			requireWatertight(t, placedMesh)
		})
	}
}

// TestShellBothCapsHoledBandsVerify reads the staged wall survey on a BX8
// body: asked, it is Suspect with the unsupported-payload diagnostic, never
// absent and never a fabricated reading (modify-reach Table DX, DX9).
func TestShellBothCapsHoledBandsVerify(t *testing.T) {
	t.Parallel()
	box := twoHolePlate(t)
	doc := box.Document()
	body, err := box.Shell(t.Context(), bothCaps(), units.Millimeters(3))
	require.NoError(t, err)
	require.Len(t, body.Lumps(), 3)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	wallReport, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, wallReport.Status)
	_, staged := findDiagnostic(wallReport.Diagnostics, decad.DiagUnsupportedSurveyPayload)
	require.True(t, staged)
}

// TestShellCupTwoHolesThroughStack is a k = 2 cup built through the stacked
// record (modify-reach §9.1): one floor under 1 + k wall bands, one lump, the
// cup's own roles, and the closed-form volume A_P·h − A_Q·(h − t) within the
// published bound.
func TestShellCupTwoHolesThroughStack(t *testing.T) {
	t.Parallel()
	const th = 3.0
	h := shellBoxHeight
	box := twoHolePlate(t)
	body, err := box.Shell(t.Context(), topCap(box), units.Millimeters(th))
	require.NoError(t, err)
	requireManifold(t, body)
	require.Len(t, body.Lumps(), 1, `every band hangs off the one floor`)
	require.Len(t, body.Shells(), 1)

	aP := 100.0*60 - 10*20 - math.Pi*8*8
	aQ := 94.0*54 - (16*26 - (4-math.Pi)*9) - math.Pi*11*11
	requireVolumeNear(t, body, aP*h-aQ*(h-th))

	roles := map[string]int{}
	for _, f := range body.Faces() {
		role := f.Origins()[0].Role
		switch {
		case len(role) > 5 && role[:5] == "side(":
			roles["side"]++
		case len(role) > 10 && role[:10] == "shellSide(":
			roles["shellSide"]++
		default:
			roles[role]++
		}
	}
	require.Equal(t, map[string]int{
		"side": 4 + 4 + 1, "shellSide": 4 + 8 + 1,
		decad.CapStart(body).Role: 1, "shellCap": 1, "rim(0)": 1, "rim(1)": 1, "rim(2)": 1,
	}, roles)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05))
	require.NoError(t, err)
	requireWatertight(t, mesh)
}
