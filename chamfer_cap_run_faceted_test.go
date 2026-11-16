package decad_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// prismFrom extrudes the closed outline that draw sketches in the plane z = z0 by height, in document doc.
func prismFrom(t *testing.T, doc *decad.Document, z0, height float64, draw func(s *sketch.Sketch)) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	draw(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var pr *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid {
			pr = p
		}
	}
	require.NotNil(t, pr)
	b, err := decad.Extrude(t.Context(), doc, s, pr, decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	return b
}

// facetedBody is the keyhole's plate as a mesh Boolean result: a 20 x 20 x 5 plate cut by a slot prism and by the bore's
// prism (a disc, or a D with a flat), in the order that slotFirst gives.
func (k keyhole) facetedBody(t *testing.T, slotFirst bool) *decad.Body {
	t.Helper()
	body, _ := k.facetedBodyIn(t, slotFirst)
	return body
}

// facetedBodyIn is facetedBody with the document it lives in, for tools that must share it.
func (k keyhole) facetedBodyIn(t *testing.T, slotFirst bool) (*decad.Body, *decad.Document) {
	t.Helper()
	doc := decad.New()
	plate := prismFrom(t, doc, 0, 5, func(s *sketch.Sketch) {
		r := s.CreateRectangle(-10, -10, 10, 10)
		s.Fix(r.A)
		s.Fix(r.B)
		s.Fix(r.C)
		s.Fix(r.D)
	})
	slot := prismFrom(t, doc, -1, 7, func(s *sketch.Sketch) {
		r := s.CreateRectangle(-k.w, -11, k.w, 0)
		s.Fix(r.A)
		s.Fix(r.B)
		s.Fix(r.C)
		s.Fix(r.D)
	})
	bore := prismFrom(t, doc, -1, 7, func(s *sketch.Sketch) {
		c := s.CreatePoint(0, 0)
		s.Fix(c)
		if k.flat == 0 {
			s.CreateCircle(c, k.r)
			return
		}
		qx := math.Sqrt(k.r*k.r - k.flat*k.flat)
		q2, q1 := s.CreatePoint(qx, k.flat), s.CreatePoint(-qx, k.flat)
		s.CreateArc(c, q1, q2)
		s.CreateLine(q2, q1)
		s.Fix(q1)
		s.Fix(q2)
	})
	tools := []*decad.Body{slot, bore}
	if !slotFirst {
		tools = []*decad.Body{bore, slot}
	}
	out := plate
	for _, tool := range tools {
		var err error
		out, err = decad.Cut(t.Context(), out, tool)
		require.NoError(t, err)
	}
	return out, doc
}

// rimOf selects the top face's edges of the bore's rim on body by their stored end points, which a Boolean places a few
// ulps off the exact ones: its arcs, and the flat of a D.
func (k keyhole) rimOf(t *testing.T, body *decad.Body) *decad.EdgeQuery {
	t.Helper()
	var q *decad.EdgeQuery
	for _, e := range body.Edges() {
		s, en := e.Start().Position().Value, e.End().Position().Value
		if s.Z != 5 || en.Z != 5 || math.Hypot(s.X, s.Y) > k.r+1e-6 || math.Hypot(en.X, en.Y) > k.r+1e-6 {
			continue
		}
		var pred decad.EdgePredicate
		switch e.Curve().(type) {
		case decad.Arc3, decad.Circle3:
			pred = decad.Circular()
		default:
			if s.Y != k.flat || en.Y != k.flat {
				continue
			}
			pred = decad.ParallelTo(r3.Vec{X: 1})
		}
		if q == nil {
			q = decad.Edges(decad.EndpointAt(s), pred)
		} else {
			q = q.Or(decad.EndpointAt(s), pred)
		}
	}
	require.NotNil(t, q)
	return q
}

// box is the prism of the axis-aligned box between lo and hi, in doc.
func box(t *testing.T, doc *decad.Document, lo, hi [3]float64) *decad.Body {
	t.Helper()
	return prismFrom(t, doc, lo[2], hi[2]-lo[2], func(s *sketch.Sketch) {
		r := s.CreateRectangle(lo[0], lo[1], hi[0], hi[1])
		s.Fix(r.A)
		s.Fix(r.B)
		s.Fix(r.C)
		s.Fix(r.D)
	})
}

// TestChamferKeyholeRimRunOnMeshBoolean chamfers the rim run of a bore that opens into a slot, on a plate that two Cuts
// built, in either order (the slot, then the bore, or the bore, then the slot), in the family of bores, slots and
// setbacks the prism takes.
func TestChamferKeyholeRimRunOnMeshBoolean(t *testing.T) {
	t.Parallel()
	for _, k := range []keyhole{{4, 0.5, 0}, {4, 0.5, 3}, {2.5, 0.6, 0}, {2.5, 0.6, 1.5}, {4, 1, 3}} {
		for _, d := range []float64{0.3, 0.5} {
			for _, slotFirst := range []bool{true, false} {
				t.Run(fmt.Sprintf("r %v w %v flat %v d %v slot first %v", k.r, k.w, k.flat, d, slotFirst), func(t *testing.T) {
					t.Parallel()
					body := k.facetedBody(t, slotFirst)
					out, err := decad.Chamfer(t.Context(), body, k.rimOf(t, body), units.Millimeters(d))
					require.NoError(t, err)
					require.True(t, out.IsSolid())
					m, err := out.Volume()
					require.NoError(t, err)
					bound, err := m.Bound.In(units.CubicMillimeter)
					require.NoError(t, err)
					require.InDelta(t, k.removed(d), volumeOf(t, body)-volumeOf(t, out), bound+1e-6)
					k.checkSections(t, out, 1, d)
					verified, err := out.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
					require.NoError(t, err)
					require.True(t, verified.BoundaryVerified())
				})
			}
		}
	}
}

// A Cut after the chamfer takes the chamfered body: a box across the slot below the bore removes the plate's material
// in the box and leaves the bevel as it was.
func TestChamferKeyholeRimRunThenCut(t *testing.T) {
	t.Parallel()
	const d = 0.5
	k := keyhole{4, 0.5, 3}
	body, doc := k.facetedBodyIn(t, true)
	out, err := decad.Chamfer(t.Context(), body, k.rimOf(t, body), units.Millimeters(d))
	require.NoError(t, err)
	cut, err := decad.Cut(t.Context(), out, box(t, doc, [3]float64{-3, -6, -1}, [3]float64{3, -5, 6}))
	require.NoError(t, err)
	mc, err := cut.Volume()
	require.NoError(t, err)
	mo, err := out.Volume()
	require.NoError(t, err)
	bound := mc.Bound.Base() + mo.Bound.Base()
	// The box holds the plate's 5 mm height of a 6 mm by 1 mm area less the 2w slot.
	require.InDelta(t, volumeOf(t, out)-5*(6-2*k.w), volumeOf(t, cut), bound+1e-6)
	k.checkSections(t, cut, 1, d)
}

// A feature of another face that stands in the cutter's reach is material the run's face does not show, so the chamfer
// is refused and the body stays live: a pin standing in the slot beside the bore, within the cutter's end, and a cavity
// from the plate's back whose wall lies within the bevel's band, where the cut would match the CAD result but the route
// does not read it.
func TestChamferKeyholeRimRunRefusesAnotherFaceInReach(t *testing.T) {
	t.Parallel()
	k := keyhole{4, 0.5, 0}
	for name, tool := range map[string]struct {
		lo, hi [3]float64
		union  bool
	}{
		"a pin in the slot":   {[3]float64{0.3, -4.6, 3.5}, [3]float64{0.45, -4.1, 6}, true},
		"a blind cavity wall": {[3]float64{3, -3, -1}, [3]float64{3.4, -2.6, 4.6}, false},
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			body, doc := k.facetedBodyIn(t, true)
			var err error
			if tool.union {
				body, err = decad.Union(t.Context(), body, box(t, doc, tool.lo, tool.hi))
			} else {
				body, err = decad.Cut(t.Context(), body, box(t, doc, tool.lo, tool.hi))
			}
			require.NoError(t, err)
			_, err = decad.Chamfer(t.Context(), body, k.rimOf(t, body), units.Millimeters(0.5))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Equal(t, []*decad.Body{body}, doc.Bodies())
		})
	}
}
