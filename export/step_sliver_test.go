package export_test

import (
	"bytes"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// sliverPrism extrudes the polygon (u, v) and circles (u, v, r) drawn on the frame at origin with axes u and v,
// over length along the frame's normal.
func sliverPrism(t *testing.T, doc *decad.Document, origin, u, v r3.Vec, poly [][2]float64, circles [][3]float64, length float64) *decad.Body {
	t.Helper()
	f, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	pts := make([]*sketch.Point, len(poly))
	for i, p := range poly {
		pts[i] = s.CreatePoint(p[0], p[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	for _, c := range circles {
		centre := s.CreatePoint(c[0], c[1])
		s.Fix(centre)
		s.AddConstraint(sketch.NewRadius(s.CreateCircle(centre, c[2]), c[2]))
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	want := len(circles)
	if poly == nil {
		want = 0
	}
	var chosen *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == want {
			chosen = p
		}
	}
	require.NotNil(t, chosen)
	body, err := decad.Extrude(t.Context(), doc, s, chosen, decad.Distance{D: units.Millimeters(length), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func requireSTEP(t *testing.T, body *decad.Body, tol float64) {
	t.Helper()
	var buf bytes.Buffer
	require.NoError(t, export.STEP(t.Context(), &buf, body, units.Millimeters(tol),
		export.WithSTEPName("sliver"), export.WithSTEPAuthor("test"), export.WithSTEPOrganization("test")))
}

// TestSTEPSplitBoreRoundingSliver splits a bored plate on Y=0. The mesh Boolean holds two copies of a bore
// vertex on the split face that differ in their last bits, joined by a triangle whose float cross product is
// zero; STEP must still write the closed mesh.
func TestSTEPSplitBoreRoundingSliver(t *testing.T) {
	doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
	x, y, z := r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}
	outline := [][2]float64{{140, -50}, {150, -47.5}, {159, -48.5}, {178, -50.5}, {186, -50}, {202, -43.2}, {215, -38.5},
		{227, -36.3}, {245, -35}, {292, -35}, {292, -18}, {306, -18}, {306, 0}, {156, 0}, {140, -12}}
	body := sliverPrism(t, doc, r3.Vec{Y: 20.3}, x, z, outline, nil, 40.6) // normal -Y, Y -20.3..20.3
	bore := sliverPrism(t, doc, r3.Vec{Y: 21.3}, x, z, nil, [][3]float64{{205, -9, 2.1}}, 42.6)
	body, err := decad.Cut(t.Context(), body, bore)
	require.NoError(t, err)
	right, err := decad.Copy(t.Context(), body)
	require.NoError(t, err)
	box := sliverPrism(t, doc, r3.Vec{Z: -51.5}, x, y, [][2]float64{{139, 0}, {307, 0}, {307, 21.3}, {139, 21.3}}, nil, 52.5)
	right, err = decad.Cut(t.Context(), right, box)
	require.NoError(t, err)
	requireSTEP(t, right, 0.1)
}

// TestSTEPGapShiftSliver cuts a through window and then a blind pocket that abuts it along a shared wall plane
// and shares its floor and ceiling planes. The mesh Boolean refuses the coplanar contact and retries with the
// pocket shifted by a few micrometres, which leaves slivers that thin; STEP must still write the closed mesh.
func TestSTEPGapShiftSliver(t *testing.T) {
	doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
	x, z := r3.Vec{X: 1}, r3.Vec{Z: 1}
	window := sliverPrism(t, doc, r3.Vec{Y: 24.3}, x, z,
		[][2]float64{{277, -16.35}, {304.117, -16.35}, {311.9, -10.9}, {315.9, -10.9}, {315.9, -6.7}, {277, -6.7},
			{277, -10.9}, {307.5, -10.9}, {309, -11.9}, {309, -12.15}, {277, -12.15}}, nil, 5)
	pocket := sliverPrism(t, doc, r3.Vec{Y: 24.3}, x, z,
		[][2]float64{{277, -16.35}, {200, -16.35}, {200, -12.15}, {230, -12.15}, {240, -6.7}, {277, -6.7},
			{277, -10.9}, {250, -10.9}, {248, -12.15}, {277, -12.15}}, nil, 3)
	full := sliverPrism(t, doc, r3.Vec{Y: 23.3}, x, z,
		[][2]float64{{165, -18}, {332, -18}, {332, 1.8}, {165, 1.8}}, nil, 3)
	body, err := decad.Cut(t.Context(), full, window)
	require.NoError(t, err)
	body, err = decad.Cut(t.Context(), body, pocket)
	require.NoError(t, err)
	t.Logf("diagnostics: %v", body.OperationDiagnostics())
	requireSTEP(t, body, 0.1)
}

// TestSTLGapShiftSliver writes the gap-shift sliver mesh as STL; every facet needs a unit normal even where the
// float cross product of its vertices is too short to normalise.
func TestSTLGapShiftSliver(t *testing.T) {
	doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
	x, z := r3.Vec{X: 1}, r3.Vec{Z: 1}
	window := sliverPrism(t, doc, r3.Vec{Y: 24.3}, x, z,
		[][2]float64{{277, -16.35}, {304.117, -16.35}, {311.9, -10.9}, {315.9, -10.9}, {315.9, -6.7}, {277, -6.7},
			{277, -10.9}, {307.5, -10.9}, {309, -11.9}, {309, -12.15}, {277, -12.15}}, nil, 5)
	pocket := sliverPrism(t, doc, r3.Vec{Y: 24.3}, x, z,
		[][2]float64{{277, -16.35}, {200, -16.35}, {200, -12.15}, {230, -12.15}, {240, -6.7}, {277, -6.7},
			{277, -10.9}, {250, -10.9}, {248, -12.15}, {277, -12.15}}, nil, 3)
	full := sliverPrism(t, doc, r3.Vec{Y: 23.3}, x, z,
		[][2]float64{{165, -18}, {332, -18}, {332, 1.8}, {165, 1.8}}, nil, 3)
	body, err := decad.Cut(t.Context(), full, window)
	require.NoError(t, err)
	body, err = decad.Cut(t.Context(), body, pocket)
	require.NoError(t, err)
	var buf bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &buf, body, units.Millimeters(0.1)))
	require.NotContains(t, buf.String(), "NaN")
}
