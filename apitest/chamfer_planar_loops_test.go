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

// roundedTrianglePlate is a 3 mm plate whose outline is three lines joined
// tangentially by three arcs: the convex hull of three circles of radius
// boss/2 whose centers lie on a circle of radius orbit.
func roundedTrianglePlate(t *testing.T, doc *decad.Document, world *sketch.World, orbit, radius float64) *decad.Body {
	t.Helper()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	const n = 3
	centers := make([]*sketch.Point, n)
	for k := range n {
		phi := 2 * math.Pi * float64(k) / n
		centers[k] = s.CreatePoint(orbit*math.Cos(phi), orbit*math.Sin(phi))
		s.Fix(centers[k])
	}
	start := make([]*sketch.Point, n)
	end := make([]*sketch.Point, n)
	for k := range n {
		phi := 2 * math.Pi * float64(k) / n
		normal := phi + math.Pi/n
		dx, dy := radius*math.Cos(normal), radius*math.Sin(normal)
		next := 2 * math.Pi * float64(k+1) / n
		start[k] = s.CreatePoint(orbit*math.Cos(phi)+dx, orbit*math.Sin(phi)+dy)
		end[k] = s.CreatePoint(orbit*math.Cos(next)+dx, orbit*math.Sin(next)+dy)
		s.Fix(start[k])
		s.Fix(end[k])
	}
	for k := range n {
		s.CreateLine(start[k], end[k])
		s.CreateArc(centers[(k+1)%n], end[k], start[(k+1)%n])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	plate, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(3), Dir: decad.Along})
	require.NoError(t, err)
	return plate
}

// planeLoopEdges selects every edge of the planar faces at height z whose
// loops pass keep, matching edges by start point and kind like a consumer
// that reads the edges off the face.
func planeLoopEdges(t *testing.T, b *decad.Body, z float64, keep func(*decad.Loop, []*decad.Edge) bool) (*decad.EdgeQuery, int) {
	t.Helper()
	return planeLoopEdgesUpTo(t, b, z, keep, -1)
}

// planeLoopEdgesUpTo is planeLoopEdges that stops after limit edges when limit is not negative.
func planeLoopEdgesUpTo(t *testing.T, b *decad.Body, z float64, keep func(*decad.Loop, []*decad.Edge) bool, limit int) (*decad.EdgeQuery, int) {
	t.Helper()
	var q *decad.EdgeQuery
	count := 0
	for _, f := range b.Faces() {
		pl, ok := f.Surface().(decad.Plane)
		if !ok || math.Abs(math.Abs(pl.Frame.N().Z)-1) > 1e-9 || math.Abs(pl.Frame.Origin().Z-z) > 1e-6 {
			continue
		}
		for _, loop := range f.Loops() {
			edges := loop.Edges()
			if !keep(loop, edges) {
				continue
			}
			for _, e := range edges {
				if limit >= 0 && count == limit {
					break
				}
				start := e.Start().Position().Value
				var pred []decad.EdgePredicate
				switch e.Curve().(type) {
				case decad.Circle3, decad.Arc3:
					pred = []decad.EdgePredicate{decad.EndpointAt(start), decad.Circular()}
				case decad.Line3:
					pred = []decad.EdgePredicate{decad.EndpointAt(start), decad.ParallelTo(e.End().Position().Value.Sub(start))}
				default:
					t.Fatalf("unexpected curve %T", e.Curve())
				}
				if q == nil {
					q = decad.Edges(pred...)
				} else {
					q = q.Or(pred...)
				}
				count++
			}
		}
	}
	require.NotNil(t, q)
	return q, count
}

const (
	plateOrbit  = 21.0
	plateRadius = 6.75
)

// roundedPlateOutline is the length of the rounded triangle's outline: three lines between the circles and three arcs of a
// third of a turn around them.
func roundedPlateOutline() float64 {
	return 3*2*plateOrbit*math.Sin(math.Pi/3) + 2*math.Pi*plateRadius
}

func TestChamferTangentLineArcOutline(t *testing.T) {
	t.Parallel()
	for _, z := range []float64{0, 3} {
		doc, world := decad.New(), sketch.NewWorld()
		plate := roundedTrianglePlate(t, doc, world, plateOrbit, plateRadius)
		q, n := planeLoopEdges(t, plate, z, func(l *decad.Loop, _ []*decad.Edge) bool { return l.IsOuter() })
		require.Equal(t, 6, n)
		out, err := decad.Chamfer(t.Context(), plate, q.Exactly(n), units.Millimeters(0.5))
		require.NoError(t, err)
		requireChamferRemoved(t, plate, out, roundedPlateOutline())
	}
}

func pinBody(t *testing.T, doc *decad.Document, world *sketch.World, x, z, diameter, length float64) *decad.Body {
	t.Helper()
	plane, err := world.CreateOffsetPlane(world.XY(), z)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(x, 0)
	s.Fix(c)
	s.CreateCircle(c, diameter/2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(length), Dir: decad.Along})
	require.NoError(t, err)
	return b
}

// TestChamferTangentOutlineAfterPinUnion chamfers the outline of a plate that pins were joined to. The join is a mesh
// Boolean, so the outline's walls are retained tags rather than a recorded prism, and the tangent junctions between the
// lines and arcs hold only to the rounding of those tags.
func TestChamferTangentOutlineAfterPinUnion(t *testing.T) {
	t.Parallel()
	doc, world := decad.New(), sketch.NewWorld()
	plate := roundedTrianglePlate(t, doc, world, plateOrbit, plateRadius)
	pin := pinBody(t, doc, world, plateOrbit, -9, 7.5, 9+1.5)
	pins, err := decad.PatternCopies(t.Context(), pin, decad.CircularPattern{Center: r3.NewVec(0, 0, 0), Axis: r3.NewVec(0, 0, 1), Count: 3})
	require.NoError(t, err)
	carrier, err := decad.Union(t.Context(), plate, append([]*decad.Body{pin}, pins...)...)
	require.NoError(t, err)
	for _, z := range []float64{0, 3} {
		q, n := planeLoopEdges(t, carrier, z, func(l *decad.Loop, _ []*decad.Edge) bool { return l.IsOuter() })
		require.Equal(t, 6, n)
		out, err := decad.Chamfer(t.Context(), carrier, q.Exactly(n), units.Millimeters(0.5))
		require.NoError(t, err)
		requireChamferRemoved(t, carrier, out, roundedPlateOutline())
		carrier = out
	}
}

// windowedFrame is a 3 mm disc of radius 44 with a bore and three through
// windows, each a loop of two lines and two arcs about the disc's axis.
func windowedFrame(t *testing.T, doc *decad.Document, world *sketch.World) *decad.Body {
	t.Helper()
	const half, hubR, rimR = 4.75, 17.25, 37.0
	hubX, rimX := math.Sqrt(hubR*hubR-half*half), math.Sqrt(rimR*rimR-half*half)
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	s.CreateCircle(s.Origin(), 44)
	s.CreateCircle(s.Origin(), 9)
	at := func(k int, x, y float64) *sketch.Point {
		sin, cos := math.Sincos(math.Pi/6 + 2*math.Pi*float64(k)/3)
		p := s.CreatePoint(x*cos-y*sin, x*sin+y*cos)
		s.Fix(p)
		return p
	}
	for k := range 3 {
		next := (k + 1) % 3
		a, b := at(k, hubX, half), at(k, rimX, half)
		c, d := at(next, rimX, -half), at(next, hubX, -half)
		s.CreateLine(a, b)
		s.CreateArc(s.Origin(), b, c)
		s.CreateLine(c, d)
		s.CreateArc(s.Origin(), a, d)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 4 {
			profile = p
		}
	}
	require.NotNil(t, profile)
	frame, err := decad.Extrude(t.Context(), doc, s, profile, decad.Distance{D: units.Millimeters(3), Dir: decad.Along})
	require.NoError(t, err)
	return frame
}

// windowLoopsLength is the length of the three windows' loops: per window two lines and the hub and rim arcs.
func windowLoopsLength() float64 {
	const half, hubR, rimR = 4.75, 17.25, 37.0
	line := math.Sqrt(rimR*rimR-half*half) - math.Sqrt(hubR*hubR-half*half)
	arc := func(r float64) float64 { return r * (2*math.Pi/3 - 2*math.Asin(half/r)) }
	return 3 * (2*line + arc(hubR) + arc(rimR))
}

func TestChamferWindowLoops(t *testing.T) {
	t.Parallel()
	doc, world := decad.New(), sketch.NewWorld()
	cover := windowedFrame(t, doc, world)
	q, n := planeLoopEdges(t, cover, 0, func(l *decad.Loop, e []*decad.Edge) bool { return !l.IsOuter() && len(e) > 1 })
	require.Equal(t, 12, n)
	out, err := decad.Chamfer(t.Context(), cover, q.Exactly(n), units.Millimeters(0.5))
	require.NoError(t, err)
	requireChamferRemoved(t, cover, out, windowLoopsLength())
}

// TestChamferWindowLoopsAfterBossUnion chamfers the window loops of both faces of a frame that a ring boss was joined
// to. The join is a mesh Boolean, so each window edge ends at a wall that is a retained cylinder, a corner the
// per-edge routes do not build.
func TestChamferWindowLoopsAfterBossUnion(t *testing.T) {
	t.Parallel()
	doc, world := decad.New(), sketch.NewWorld()
	frame := windowedFrame(t, doc, world)
	plane, err := world.CreateOffsetPlane(world.XY(), 3)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	s.CreateCircle(s.Origin(), 15)
	s.CreateCircle(s.Origin(), 11.1)
	require.NoError(t, solveSketch(t, s))
	var ring *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			ring = p
		}
	}
	require.NotNil(t, ring)
	boss, err := decad.Extrude(t.Context(), doc, s, ring, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	cover, err := decad.Union(t.Context(), frame, boss)
	require.NoError(t, err)
	// The bevel on the cap face z=0 takes the rim and the bore with the windows, as one selection.
	for _, step := range []struct {
		z    float64
		keep func(*decad.Loop, []*decad.Edge) bool
		run  float64
	}{
		{0, func(*decad.Loop, []*decad.Edge) bool { return true }, windowLoopsLength() + 2*math.Pi*(44-0.5/3) + 2*math.Pi*(9+0.5/3)},
		{3, func(l *decad.Loop, e []*decad.Edge) bool { return !l.IsOuter() && len(e) > 1 }, windowLoopsLength()},
	} {
		q, n := planeLoopEdges(t, cover, step.z, step.keep)
		out, err := decad.Chamfer(t.Context(), cover, q.Exactly(n), units.Millimeters(0.5))
		require.NoError(t, err, "z=%g", step.z)
		requireChamferRemoved(t, cover, out, step.run)
		// The cutters are held exactly to within a few microns, so cutting the windows leaves the body's bound where
		// the join of the boss left it.
		before, err := cover.Bounds()
		require.NoError(t, err)
		after, err := out.Bounds()
		require.NoError(t, err)
		require.LessOrEqual(t, after.Bound.Base(), before.Bound.Base()*1.001+1e-9, "z=%g", step.z)
		cover = out
	}
}

func solveSketch(t *testing.T, s *sketch.Sketch) error {
	t.Helper()
	_, err := s.Solve(t.Context())
	return err
}

// requireChamferRemoved checks that out is a solid that passes the full mesh verification and that it removed about
// run*c*c/2 of volume from before, where run is the chamfered edge length. A corner adds or removes a term of order c^3,
// so the removed volume may differ from that by 2 percent. The setback is 0.5 mm.
func requireChamferRemoved(t *testing.T, before, out *decad.Body, run float64) {
	const c, tol = 0.5, 0.02
	t.Helper()
	require.True(t, out.IsSolid())
	v0, err := before.Volume()
	require.NoError(t, err)
	v1, err := out.Volume()
	require.NoError(t, err)
	removed := v0.Value.Base() - v1.Value.Base()
	want := run * c * c / 2
	require.InEpsilon(t, want, removed, tol, "removed %.4f, want about %.4f", removed, want)
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
}

// windowedCoverWithBoss is the frame with a ring boss joined to its z=3 face, a mesh Boolean result.
func windowedCoverWithBoss(t *testing.T) *decad.Body {
	t.Helper()
	doc, world := decad.New(), sketch.NewWorld()
	frame := windowedFrame(t, doc, world)
	plane, err := world.CreateOffsetPlane(world.XY(), 3)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	s.CreateCircle(s.Origin(), 15)
	s.CreateCircle(s.Origin(), 11.1)
	require.NoError(t, solveSketch(t, s))
	var ring *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			ring = p
		}
	}
	require.NotNil(t, ring)
	boss, err := decad.Extrude(t.Context(), doc, s, ring, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	cover, err := decad.Union(t.Context(), frame, boss)
	require.NoError(t, err)
	return cover
}

// TestChamferWindowLoopsRefusals keeps the refusals of a loop chamfer that cannot be cut: a setback that reaches the far
// end of the 3 mm walls, and a selection that holds only part of a loop.
func TestChamferWindowLoopsRefusals(t *testing.T) {
	t.Parallel()
	t.Run("setback reaches the far end of the walls", func(t *testing.T) {
		t.Parallel()
		cover := windowedCoverWithBoss(t)
		q, n := planeLoopEdges(t, cover, 0, func(l *decad.Loop, e []*decad.Edge) bool { return !l.IsOuter() && len(e) > 1 })
		_, err := decad.Chamfer(t.Context(), cover, q.Exactly(n), units.Millimeters(3))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
	t.Run("part of a loop", func(t *testing.T) {
		t.Parallel()
		cover := windowedCoverWithBoss(t)
		q, n := planeLoopEdgesUpTo(t, cover, 0, func(l *decad.Loop, e []*decad.Edge) bool { return !l.IsOuter() && len(e) > 1 }, 11)
		require.Equal(t, 11, n)
		_, err := decad.Chamfer(t.Context(), cover, q.Exactly(n), units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}
