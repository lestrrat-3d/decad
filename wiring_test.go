package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestWiring exercises the seams decad is built on: a solved 2D sketch profile
// with provenance, staleness detection and per-fragment trim certification
// (BoundaryEdge.TStart/TEnd/TExact), lifted into world space through the
// r3 frame of the plane it was drawn on and placed by an r3 rigid transform,
// with every quantity a units.Value — the same type, from the same module,
// that sketch's own dimensions carry. It asserts nothing about decad itself —
// the kernel API is not implemented yet. It exists to keep the dependencies
// honest and to prove the layering compiles. Delete it once real decad code
// imports them.
func TestWiring(t *testing.T) {
	w := sketch.NewWorld()

	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err, `w.CreateSketch should succeed`)

	// A 100 x 60 rectangle, grounded at the origin corner.
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)

	_, err = s.Solve(t.Context())
	require.NoError(t, err, `s.Solve should succeed`)

	// sketch answers the 2D question: does this close into an extrudable region?
	profiles := s.Profiles()
	require.Len(t, profiles, 1, `rectangle should yield exactly one profile`)
	prof := profiles[0]
	require.True(t, prof.Valid, `profile should be extrudable`)
	require.InDelta(t, 6000.0, prof.Area, 1e-9, `profile area should be 100 x 60`)

	// sketch answers provenance and staleness too: a profile back-references
	// the sketch it was built from and reports whether that sketch has changed.
	require.Same(t, s, prof.Sketch(), `profile should back-reference its sketch`)
	require.False(t, prof.IsStale(), `fresh profile should not be stale`)

	// A dimension's driving value is a units.Value — the same type decad's
	// quantities use, so nothing converts at the seam.
	p1 := s.CreatePoint(200, 0)
	p2 := s.CreatePoint(260, 0)
	dim := sketch.NewDistance(p1, p2, 60)
	s.AddConstraint(dim)
	_, err = s.Solve(t.Context())
	require.NoError(t, err, `s.Solve should succeed after the edit`)
	require.Equal(t, units.Millimeters(60), dim.Target(), `dimension target should be 60 mm`)
	require.Equal(t, units.Length, dim.Kind(), `a distance measures Length`)

	// That edit made the earlier profile snapshot stale, and sketch says so.
	require.True(t, prof.IsStale(), `profile built before the edit should read stale`)
	require.False(t, s.Profiles()[0].IsStale(), `rebuilt profile should be current`)

	// sketch answers the trim question too: a Partial fragment carries its
	// parameter range on the entity (TStart/TEnd, natural direction, never
	// wrapping) and whether that range is exact. A circle crossed by a
	// rectangle edge is cut by the closed-form kernel, so its fragments are
	// certified (TExact true); an ellipse's cuts are sampled, and sketch says
	// so (TExact false).
	s2, err := w.CreateSketch(w.XY())
	require.NoError(t, err, `w.CreateSketch should succeed`)
	s2.CreateRectangle(0, 0, 100, 60)
	s2.CreateCircle(s2.CreatePoint(90, 30), 20)
	s2.CreateEllipse(s2.CreatePoint(10, 30), 25, 15, 0)
	_, err = s2.Solve(t.Context())
	require.NoError(t, err, `s2.Solve should succeed`)
	var circleFrags, ellipseFrags int
	for _, p := range s2.Profiles() {
		loops := append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...)
		for _, loop := range loops {
			for _, e := range loop {
				if !e.Partial {
					continue
				}
				require.Less(t, e.TStart, e.TEnd, `a fragment's range is natural-direction and never wraps`)
				require.GreaterOrEqual(t, e.TStart, 0.0, `a fragment's range stays in [0, 1]`)
				require.LessOrEqual(t, e.TEnd, 1.0, `a fragment's range stays in [0, 1]`)
				switch e.Entity.(type) {
				case *sketch.Circle:
					circleFrags++
					require.True(t, e.TExact, `a circle cut by a line is certified exact`)
				case *sketch.Ellipse:
					ellipseFrags++
					require.False(t, e.TExact, `an ellipse's cut is sampled, never certified`)
				}
			}
		}
	}
	require.Positive(t, circleFrags, `the crossed circle should be split into fragments`)
	require.Positive(t, ellipseFrags, `the crossed ellipse should be split into fragments`)

	// units values round-trip as text, bit for bit.
	vol := units.CubicMillimeters(24)
	text, err := vol.MarshalText()
	require.NoError(t, err, `a volume should have a text form`)
	require.Equal(t, "24 mm^3", string(text), `a volume marshals with its symbol`)
	var parsed units.Value
	require.NoError(t, parsed.UnmarshalText(text), `the text form should parse back`)
	require.Equal(t, vol, parsed, `the text round trip should be exact`)

	// r3 answers the 3D question: where does that region sit in world space?
	f, err := w.XY().Frame()
	require.NoError(t, err, `XY datum should have a valid frame`)

	corner := f.ToWorldUV(100, 60)
	require.Equal(t, r3.NewVec(100, 60, 0), corner, `XY-local (100, 60) is world (100, 60, 0)`)

	// ...and where a rigid placement moves it: a quarter turn about Z, then 5 up.
	spin, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), units.Degrees(90))
	require.NoError(t, err, `rotation should build`)
	lift, err := r3.Translation(r3.NewVec(0, 0, 5))
	require.NoError(t, err, `translation should build`)
	place, err := spin.Then(lift)
	require.NoError(t, err, `composition should stay an isometry`)
	moved := place.Apply(corner)
	require.InDelta(t, -60, moved.X, 1e-9, `placed corner X`)
	require.InDelta(t, 100, moved.Y, 1e-9, `placed corner Y`)
	require.InDelta(t, 5, moved.Z, 1e-9, `placed corner Z`)
	back, err := place.Inverse()
	require.NoError(t, err, `a rigid transform inverts`)
	unmoved := back.Apply(moved)
	require.InDelta(t, corner.X, unmoved.X, 1e-9, `inverse round trip X`)
	require.InDelta(t, corner.Y, unmoved.Y, 1e-9, `inverse round trip Y`)
	require.InDelta(t, corner.Z, unmoved.Z, 1e-9, `inverse round trip Z`)
}
