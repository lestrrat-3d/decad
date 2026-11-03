package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin agent-docs/interference-design.md §3.2's coaxial
// separator: a shaft whose radius equals its bore's. The cylinder faces
// coincide, so no mesh decides the pair, but the shared cylinder separates
// the interiors.

// coaxialSection extrudes the circles of radius radii about (cx, cy) on the
// XY plane by length; one radius is a disc, two an annulus.
func coaxialSection(t *testing.T, doc *decad.Document, cx, cy, length float64, radii ...float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	for _, r := range radii {
		c := s.CreatePoint(cx, cy)
		s.Fix(c)
		s.CreateCircle(c, r)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(length), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// placedAlongZ moves body by dz.
func placedAlongZ(t *testing.T, body *decad.Body, dz float64) *decad.Body {
	t.Helper()
	move, err := r3.Translation(r3.Vec{Z: dz})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, move)
	require.NoError(t, err)
	return placed
}

// keyedShaft is a Ø8 shaft along z from 0 to 40 with a keyway pocket cut by
// a placed box, so the Cut takes the mesh path and the shaft is a held mesh.
func keyedShaft(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	shaft := coaxialSection(t, doc, 0, 0, 40, 4)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(2.8, -1, 5, 1)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	key, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	keyed, err := decad.Cut(t.Context(), shaft, placedAlongZ(t, key, 10.001))
	require.NoError(t, err)
	require.Equal(t, "decad.facetedPayload", decadtest.PayloadClass(keyed))
	return keyed
}

// pairFindings counts the report's interference rows and pair diagnostics
// that name both a and b.
func pairFindings(report *decad.Report, a, b *decad.Body) (interferences, diagnostics int) {
	named := func(x, y *decad.Body) bool { return x == a && y == b || x == b && y == a }
	for _, in := range report.Interferences {
		if named(in.A, in.B) {
			interferences++
		}
	}
	for _, d := range report.Diagnostics {
		if d.Pair != nil && named(d.Pair.A, d.Pair.B) {
			diagnostics++
		}
	}
	return interferences, diagnostics
}

// TestVerifyCoaxialSeparator holds the separator to exact matches: a held or
// prism shaft in a bore of its radius is decided, and a smaller bore or an
// axis 0.01 mm off is not.
//
// Leg shown to fail: never matching the outside side leaves both coincident
// cases with pair diagnostics.
func TestVerifyCoaxialSeparator(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		held      bool
		cx, bore  float64
		separated bool
	}{
		{"held shaft in a bore of its radius", true, 0, 4, true},
		{"prism shaft in a bore of its radius", false, 0, 4, true},
		{"bore smaller than the shaft", true, 0, 3.9, false},
		{"bore off the shaft's axis", true, 0.01, 4, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			var shaft *decad.Body
			if tc.held {
				shaft = keyedShaft(t, doc)
			} else {
				shaft = coaxialSection(t, doc, 0, 0, 40, 4)
			}
			bearing := placedAlongZ(t, coaxialSection(t, doc, tc.cx, 0, 8, 11, tc.bore), 6)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, shaft, bearing)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the separator decides the pair")
				return
			}
			require.Positive(t, interferences+diagnostics, "an overlapping or unproven pair is never called separated")
		})
	}
}
