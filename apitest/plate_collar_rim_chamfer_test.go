package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// annulusAt extrudes the ring rIn..rOut up from the plane z = z0 by h.
func annulusAt(t *testing.T, doc *decad.Document, rIn, rOut, z0, h float64) *decad.Body {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{Z: z0}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, rIn)
	s.CreateCircle(c, rOut)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, pr := range s.Profiles() {
		if !pr.Valid || len(pr.Holes) != 1 {
			continue
		}
		b, err := decad.Extrude(t.Context(), doc, s, pr, decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
		require.NoError(t, err)
		return b
	}
	require.FailNow(t, "no annulus profile")
	return nil
}

func plateCollar(t *testing.T, collarZ0 float64) (*decad.Body, error) {
	t.Helper()
	doc := decad.New()
	plate := annulusAt(t, doc, 4.55, 32.65, 8.095, 1.2)
	collar := annulusAt(t, doc, 4.55, 5.9, collarZ0, 18.9-collarZ0)
	return decad.Union(t.Context(), plate, collar)
}

// TestPlateCollarRimChamfer chamfers the bore's rim on the bed face of a plate joined to a collar that starts inside
// it. The blend strip's seam sliver, whose two ring points differ by rounding, must wind with the rest of the strip.
func TestPlateCollarRimChamfer(t *testing.T) {
	t.Parallel()
	body, err := plateCollar(t, 8.695)
	require.NoError(t, err)
	before, err := body.Volume()
	require.NoError(t, err)
	rim := boreRim(t, body)
	out, err := decad.Chamfer(t.Context(), body, rim.Exactly(1), units.Millimeters(0.5))
	require.NoError(t, err)
	require.True(t, out.IsSolid())
	after, err := out.Volume()
	require.NoError(t, err)
	// The 0.5 x 0.5 triangle removed from the rim sweeps its centroid, 0.5/3 off the bore wall, round the axis.
	want := 2 * math.Pi * (4.55 + 0.5/3) * 0.125
	require.InDelta(t, want, before.Value.Base()-after.Value.Base(),
		before.Bound.Base()+after.Bound.Base()+0.2)
}

// boreRim selects the one circular edge of body that rings the hole's rim on the bed face. EndpointAt matches the
// stored position exactly, so the position is read back from the edge.
func boreRim(t *testing.T, body *decad.Body) *decad.EdgeQuery {
	t.Helper()
	for _, e := range body.Edges() {
		p := e.Start().Position().Value
		if math.Abs(p.Z-8.095) < 1e-9 && math.Abs(math.Hypot(p.X, p.Y)-4.55) < 1e-9 {
			return decad.Edges(decad.EndpointAt(p), decad.Circular())
		}
	}
	require.FailNow(t, "no bore rim edge")
	return nil
}
