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

func loftPocketProfile(t *testing.T, station, flare float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	const r = 2.5
	a := r * math.Sqrt(3) / 2
	frame, err := r3.NewFrame(r3.NewVec(station, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	coords := [][2]float64{{-a - flare, 6 - flare}, {a + flare, 6 - flare},
		{a + flare, 8 + flare}, {-a - flare, 8 + flare}}
	points := make([]*sketch.Point, 4)
	lines := make([]*sketch.Line, 4)
	for i, p := range coords {
		points[i] = s.CreatePoint(p[0], p[1])
	}
	for i := range points {
		j := (i + 1) % len(points)
		lines[i] = s.CreateLine(points[i], points[j])
		if coords[i][1] == coords[j][1] {
			s.AddConstraint(sketch.NewHorizontal(lines[i]), sketch.NewDistancePointLine(s.Origin(), lines[i], math.Abs(coords[i][1])))
		} else {
			s.AddConstraint(sketch.NewVertical(lines[i]), sketch.NewDistancePointLine(s.Origin(), lines[i], math.Abs(coords[i][0])))
		}
	}
	for i := range points {
		_, err = sketch.CreateFillet(s, lines[(i+3)%4], lines[i], points[i], 0.5)
		require.NoError(t, err)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

func loftPocketBody(t *testing.T, options ...decad.DocumentOption) (*decad.Document, *decad.Body) {
	t.Helper()
	const r = 2.5
	a := r * math.Sqrt(3) / 2
	doc := decad.New(append([]decad.DocumentOption{decad.WithChainTolerance(units.Millimeters(0.01))}, options...)...)
	body := slabCylinder(t, doc, r3.NewVec(0, 0, -10), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), 11, 20)
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 6), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	points := []*sketch.Point{s.CreatePoint(-r, 0), s.CreatePoint(-r/2, -a), s.CreatePoint(15, -a),
		s.CreatePoint(15, a), s.CreatePoint(-r/2, a)}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	slot, err := decad.Extrude(doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(2), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	body, err = decad.Cut(body, slot).Do(t.Context())
	require.NoError(t, err)
	matchingEdges := 0
	for _, edge := range body.Edges() {
		if edge.Start() == nil || edge.End() == nil {
			continue
		}
		start, end := edge.Start().Position(), edge.End().Position()
		if math.Abs(start.Value.X+r/2) > 1e-9 || math.Abs(end.Value.X+r/2) > 1e-9 ||
			math.Abs(math.Abs(start.Value.Y)-a) > 1e-9 || math.Abs(math.Abs(end.Value.Y)-a) > 1e-9 {
			continue
		}
		require.IsType(t, decad.Line3{}, edge.Curve())
		require.Zero(t, start.Bound.Base())
		require.Zero(t, end.Bound.Base())
		matchingEdges++
	}
	require.Equal(t, 2, matchingEdges)
	return doc, body
}

func TestLoftCutAtPentagonalPocketJunction(t *testing.T) {
	const r = 2.5
	a := r * math.Sqrt(3) / 2
	doc, body := loftPocketBody(t)
	s0, p0 := loftPocketProfile(t, -r/2, 0)
	s1, p1 := loftPocketProfile(t, -r/2+0.5, 0.5)
	taper, err := decad.Loft(doc, s0, p0, s1, p1).Do(t.Context())
	require.NoError(t, err)
	matchingEdges := 0
	for _, edge := range taper.Edges() {
		if edge.Start() == nil || edge.End() == nil {
			continue
		}
		start, end := edge.Start().Position(), edge.End().Position()
		if math.Abs(start.Value.X+r/2) > 1e-9 || math.Abs(end.Value.X+r/2) > 1e-9 ||
			math.Abs(math.Abs(start.Value.Y)-a) > 1e-9 || math.Abs(math.Abs(end.Value.Y)-a) > 1e-9 {
			continue
		}
		require.IsType(t, decad.Line3{}, edge.Curve())
		require.Less(t, start.Bound.Base(), 1e-9)
		require.Less(t, end.Bound.Base(), 1e-9)
		matchingEdges++
	}
	require.Equal(t, 2, matchingEdges)
	removed := r3.NewVec(-0.8, 2.35, 7)
	retained := r3.NewVec(-1.5, 2.35, 7)
	for _, point := range []r3.Vec{removed, retained} {
		got, err := body.LocatePoint(t.Context(), point, units.Millimeters(0.01))
		require.NoError(t, err)
		require.Equal(t, decad.PointInside, got, "%v", point)
	}
	for point, want := range map[r3.Vec]decad.PointLocation{removed: decad.PointInside, retained: decad.PointOutside} {
		got, err := taper.LocatePoint(t.Context(), point, units.Millimeters(0.01))
		require.NoError(t, err)
		require.Equal(t, want, got, "%v", point)
	}
	sectionPlane := outlinePlane(t, r3.NewVec(0, 0, 7), r3.NewVec(0, 0, 1))
	before, err := body.SectionOutline(t.Context(), sectionPlane, units.Millimeters(0.1))
	require.NoError(t, err)
	volumeBefore, err := body.Volume()
	require.NoError(t, err)
	body, err = decad.Cut(body, taper).Do(t.Context())
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	volumeAfter, err := body.Volume()
	require.NoError(t, err)
	// The taper lies outside the slot it widens: its section grows from the slot's 2a-by-2 rectangle to
	// (2a+1)-by-3 over 0.5 mm, less four 0.5 mm corner roundings of (4-pi)/4 * 0.5^2 * 4.
	roundings := (4 - math.Pi) * 0.25
	removed3 := 0.5*a + 0.5 + 4.0/3*0.125 - 0.5*roundings
	t.Logf("volume before %v after %v expected removed %v", volumeBefore.Value.Base(), volumeAfter.Value.Base(), removed3)
	require.InDelta(t, removed3, volumeBefore.Value.Base()-volumeAfter.Value.Base(), 0.05)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	after, err := body.SectionOutline(t.Context(), sectionPlane, units.Millimeters(0.1))
	require.NoError(t, err)
	// The taper adds a 0.5-by-0.5 triangle to each side of this section.
	require.InDelta(t, 0.25, before.PolygonArea().Base()-after.PolygonArea().Base(), 0.02)
}

func loftPocketTaper(t *testing.T, doc *decad.Document, flare float64) *decad.Body {
	t.Helper()
	const r = 2.5
	s0, p0 := loftPocketProfile(t, -r/2, 0)
	s1, p1 := loftPocketProfile(t, -r/2+0.5, flare)
	taper, err := decad.Loft(doc, s0, p0, s1, p1).Do(t.Context())
	require.NoError(t, err)
	return taper
}

func sectionContainsPoint(outline decad.SectionOutline, point r3.Vec) bool {
	inside := false
	for _, loop := range outline.Loops {
		for i, a := range loop {
			b := loop[(i+1)%len(loop)]
			if (a.Y > point.Y) != (b.Y > point.Y) && point.X < (b.X-a.X)*(point.Y-a.Y)/(b.Y-a.Y)+a.X {
				inside = !inside
			}
		}
	}
	return inside
}

// A taper that narrows into the slot lies in the pocket's void and touches the target only along the junction
// edges. The proximity gate has no evidence for that contact, so the cut keeps its original refusal.
func TestLoftCutNarrowingTaperAtPocketJunctionStaysRefused(t *testing.T) {
	doc, body := loftPocketBody(t)
	taper := loftPocketTaper(t, doc, -0.3)
	_, err := decad.Cut(body, taper).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
