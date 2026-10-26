package pointquery

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type pt = sectionrecord.Point2

func line(a, b pt) sectionrecord.LineSeg {
	return sectionrecord.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1}
}

// reversedLine walks a → b over the line entity drawn b → a.
func reversedLine(a, b pt) sectionrecord.LineSeg {
	return sectionrecord.LineSeg{Start: b, End: a, TStart: 1, TEnd: 0}
}

func squareLoop() sectionrecord.LoopRecord {
	a, b, c, d := pt{U: 0, V: 0}, pt{U: 10, V: 0}, pt{U: 10, V: 10}, pt{U: 0, V: 10}
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{line(a, b), line(b, c), line(c, d), line(d, a)}}
}

func wholeCircle(center pt, r float64, ccw bool) sectionrecord.CircleSeg {
	seg := sectionrecord.CircleSeg{Center: center, Radius: units.Millimeters(r), CCW: ccw, TStart: 0, TEnd: 1}
	if !ccw {
		seg.TStart, seg.TEnd = 1, 0
	}
	return seg
}

// pieLoop is the quarter disk of radius 20 about the origin: a line out
// along u, the whole arc to (0, 20), and a line back.
func pieLoop(arc sectionrecord.ArcSeg) sectionrecord.LoopRecord {
	o := pt{U: 0, V: 0}
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{line(o, arc.Start), arc, line(arc.End, o)}}
}

func quarterArc() sectionrecord.ArcSeg {
	return sectionrecord.ArcSeg{Center: pt{U: 0, V: 0}, Start: pt{U: 20, V: 0}, End: pt{U: 0, V: 20}, TStart: 0, TEnd: 1}
}

func payloadOf(t *testing.T, profile momentinput.Profile) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return prismrecord.Payload{Profile: profile, Frame: frame, Z0: 0, Z1: 10, Xform: r3.Identity()}
}

type prismCase struct {
	name string
	at   r3.Vec
	want Location
}

func requirePrismLocations(t *testing.T, pp prismrecord.Payload, cases []prismCase) {
	t.Helper()
	for _, tc := range cases {
		got, decided, err := LocatePrism(t.Context(), tc.at, pp)
		require.NoError(t, err, tc.name)
		require.Equal(t, tc.want != Undecided, decided, tc.name)
		require.Equal(t, tc.want, got, tc.name)
	}
}

func TestLocatePrismSquare(t *testing.T) {
	t.Parallel()
	cases := []prismCase{
		{name: "material", at: r3.NewVec(5, 5, 5), want: Inside},
		{name: "beside", at: r3.NewVec(11, 5, 5), want: Outside},
		{name: "side", at: r3.NewVec(10, 5, 5), want: OnBoundary},
		{name: "corner", at: r3.NewVec(0, 0, 5), want: OnBoundary},
		{name: "cap", at: r3.NewVec(5, 5, 10), want: OnBoundary},
		{name: "cap level beside", at: r3.NewVec(11, 5, 10), want: Outside},
		{name: "below", at: r3.NewVec(5, 5, -1e-9), want: Outside},
		{name: "1e-9 inside a side", at: r3.NewVec(1e-9, 5, 5), want: Inside},
	}
	requirePrismLocations(t, payloadOf(t, momentinput.Profile{Outer: squareLoop()}), cases)

	// The same square walked clockwise through reversed records answers the
	// same: a reversed line denotes its entity at its own recorded t.
	a, b, c, d := pt{U: 0, V: 0}, pt{U: 10, V: 0}, pt{U: 10, V: 10}, pt{U: 0, V: 10}
	reversed := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		reversedLine(a, d), reversedLine(d, c), reversedLine(c, b), reversedLine(b, a),
	}}
	requirePrismLocations(t, payloadOf(t, momentinput.Profile{Outer: reversed}), cases)
}

func TestLocatePrismHoles(t *testing.T) {
	t.Parallel()
	pp := payloadOf(t, momentinput.Profile{Outer: squareLoop(), Holes: []sectionrecord.LoopRecord{
		{Segments: []sectionrecord.CurveSegment{wholeCircle(pt{U: 5, V: 5}, 2, false)}},
	}})
	requirePrismLocations(t, pp, []prismCase{
		{name: "bore", at: r3.NewVec(5, 5, 5), want: Outside},
		{name: "bore wall", at: r3.NewVec(7, 5, 5), want: OnBoundary},
		{name: "bore rim", at: r3.NewVec(5, 3, 0), want: OnBoundary},
		{name: "material", at: r3.NewVec(8, 8, 5), want: Inside},
	})

	// Two holes holding one point, and a hole point outside the outer loop,
	// are not a sketch-arranged region; the route does not decide them.
	overlapping := payloadOf(t, momentinput.Profile{Outer: squareLoop(), Holes: []sectionrecord.LoopRecord{
		{Segments: []sectionrecord.CurveSegment{wholeCircle(pt{U: 5, V: 5}, 2, false)}},
		{Segments: []sectionrecord.CurveSegment{wholeCircle(pt{U: 6, V: 5}, 1.5, false)}},
	}})
	requirePrismLocations(t, overlapping, []prismCase{{name: "two holes", at: r3.NewVec(5.5, 5, 5), want: Undecided}})
	escaping := payloadOf(t, momentinput.Profile{Outer: squareLoop(), Holes: []sectionrecord.LoopRecord{
		{Segments: []sectionrecord.CurveSegment{wholeCircle(pt{U: 10, V: 5}, 2, false)}},
	}})
	requirePrismLocations(t, escaping, []prismCase{{name: "hole outside the outer loop", at: r3.NewVec(11, 5, 5), want: Undecided}})
	// A point on the outer loop inside a hole is not a sketch-arranged
	// region's point either.
	crossing := payloadOf(t, momentinput.Profile{Outer: squareLoop(), Holes: []sectionrecord.LoopRecord{
		{Segments: []sectionrecord.CurveSegment{wholeCircle(pt{U: 5, V: 1}, 2, false)}},
	}})
	requirePrismLocations(t, crossing, []prismCase{{name: "outer boundary inside a hole", at: r3.NewVec(5, 0, 5), want: Undecided}})
}

func TestLocatePrismArc(t *testing.T) {
	t.Parallel()
	requirePrismLocations(t, payloadOf(t, momentinput.Profile{Outer: pieLoop(quarterArc())}), []prismCase{
		{name: "segment", at: r3.NewVec(12, 12, 5), want: Inside},
		{name: "beyond the arc", at: r3.NewVec(14.2, 14.2, 5), want: Outside},
		{name: "on the arc", at: r3.NewVec(12, 16, 5), want: OnBoundary},
		{name: "circle beyond the arc", at: r3.NewVec(-12, 16, 5), want: Outside},
		{name: "chord", at: r3.NewVec(10, 10, 5), want: Undecided},
	})

	// The same pie walked clockwise: the arc runs back from End to Start.
	arc := quarterArc()
	arc.TStart, arc.TEnd = 1, 0
	o := pt{U: 0, V: 0}
	clockwise := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		reversedLine(o, arc.End), arc, reversedLine(arc.Start, o),
	}}
	requirePrismLocations(t, payloadOf(t, momentinput.Profile{Outer: clockwise}), []prismCase{
		{name: "segment", at: r3.NewVec(12, 12, 5), want: Inside},
		{name: "beyond the arc", at: r3.NewVec(14.2, 14.2, 5), want: Outside},
	})
}

// TestLocatePrismLeavesOpenRecords covers every gate that hands a record or
// point back to the mesh.
func TestLocatePrismLeavesOpenRecords(t *testing.T) {
	t.Parallel()
	square := momentinput.Profile{Outer: squareLoop()}
	at := r3.NewVec(5, 5, 5)
	gates := map[string]func(*prismrecord.Payload){
		"section displacement":     func(pp *prismrecord.Payload) { pp.SectionDelta = 1e-12 },
		"start level displacement": func(pp *prismrecord.Payload) { pp.Z0Delta = 1e-12 },
		"end level displacement":   func(pp *prismrecord.Payload) { pp.Z1Delta = 1e-12 },
		"surface result":           func(pp *prismrecord.Payload) { pp.SurfaceResult = true },
		"empty interval":           func(pp *prismrecord.Payload) { pp.Z1 = pp.Z0 },
		"singular placement":       func(pp *prismrecord.Payload) { pp.Xform = r3.Transform{} },
	}
	for name, gate := range gates {
		pp := payloadOf(t, square)
		gate(&pp)
		_, decided, err := LocatePrism(t.Context(), at, pp)
		require.NoError(t, err, name)
		require.False(t, decided, name)
	}

	cutArc := quarterArc()
	cutArc.TEnd = 0.5
	offRadius := quarterArc()
	offRadius.End = pt{U: 0, V: 20.000000000000004}
	cutCircle := wholeCircle(pt{U: 5, V: 5}, 2, true)
	cutCircle.TEnd = 0.5
	inchCircle := wholeCircle(pt{U: 5, V: 5}, 2, true)
	inchCircle.Radius = units.Inches(2)
	o := pt{U: 0, V: 0}
	loops := map[string]sectionrecord.LoopRecord{
		"arc fragment":       pieLoop(cutArc),
		"arc end off radius": pieLoop(offRadius),
		"circle fragment":    {Segments: []sectionrecord.CurveSegment{cutCircle}},
		"circle in inches":   {Segments: []sectionrecord.CurveSegment{inchCircle}},
		"ellipse": {Segments: []sectionrecord.CurveSegment{sectionrecord.EllipseSeg{
			Center: pt{U: 5, V: 5}, Rx: units.Millimeters(3), Ry: units.Millimeters(2), Rotation: units.Degrees(0),
			CCW: true, TStart: 0, TEnd: 1,
		}}},
		"lone line":       {Segments: []sectionrecord.CurveSegment{line(o, pt{U: 10, V: 0})}},
		"arc start = end": pieLoop(sectionrecord.ArcSeg{Center: o, Start: pt{U: 20, V: 0}, End: pt{U: 20, V: 0}, TStart: 0, TEnd: 1}),
		"zero radius":     {Segments: []sectionrecord.CurveSegment{wholeCircle(pt{U: 5, V: 5}, 0, true)}},
	}
	for name, loop := range loops {
		_, decided, err := LocatePrism(t.Context(), r3.NewVec(5, 5.5, 5), payloadOf(t, momentinput.Profile{Outer: loop}))
		require.NoError(t, err, name)
		require.False(t, decided, name)
	}
}

// TestLocatePrismJunctionCorner reads two line fragments whose recorded ends
// miss each other by a few ulps. The evaluator closes the junction through
// the exact crossing of their supports, so that crossing is boundary.
func TestLocatePrismJunctionCorner(t *testing.T) {
	t.Parallel()
	// Each fragment's recorded t places its ends a few ulps short of the
	// corners, so only the closing path through the exact corner reaches
	// them; a straight chord would cut each corner off.
	t0 := math.Nextafter(1.0/12, 1)
	t1 := math.Nextafter(11.0/12, 0)
	bottom := sectionrecord.LineSeg{Start: pt{U: -1, V: 0}, End: pt{U: 11, V: 0}, TStart: t0, TEnd: t1}
	right := sectionrecord.LineSeg{Start: pt{U: 10, V: -1}, End: pt{U: 10, V: 11}, TStart: t0, TEnd: t1}
	top := sectionrecord.LineSeg{Start: pt{U: 11, V: 10}, End: pt{U: -1, V: 10}, TStart: t0, TEnd: t1}
	left := sectionrecord.LineSeg{Start: pt{U: 0, V: 11}, End: pt{U: 0, V: -1}, TStart: t0, TEnd: t1}
	ends := func(s sectionrecord.LineSeg) [2]float64 {
		a, b := momentinput.ExactLineEnds(s)
		u0, _ := a.U.Float64()
		u1, _ := b.U.Float64()
		return [2]float64{u0, u1}
	}
	got := ends(bottom)
	require.Greater(t, got[0], 0.0, `the fragment starts past the corner`)
	require.Less(t, got[1], 10.0, `the fragment ends short of the corner`)
	pp := payloadOf(t, momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		bottom, right, top, left,
	}}})
	requirePrismLocations(t, pp, []prismCase{
		{name: "corner", at: r3.NewVec(10, 0, 5), want: OnBoundary},
		{name: "material", at: r3.NewVec(5, 5, 5), want: Inside},
		{name: "1e-9 past the corner", at: r3.NewVec(10+1e-9, 0, 5), want: Outside},
	})
}

// TestLocatePrismConcaveArc reads a 20 mm square whose top side carries a
// radius-4 notch: a half-circle arc dipping into the material, walked back
// from End to Start. The ring's chord runs along the top side, so a point in
// the notch is inside the ring and only the arc's −1 correction places it
// outside.
func TestLocatePrismConcaveArc(t *testing.T) {
	t.Parallel()
	notch := sectionrecord.ArcSeg{Center: pt{U: 10, V: 20}, Start: pt{U: 6, V: 20}, End: pt{U: 14, V: 20}, TStart: 1, TEnd: 0}
	a, b, c, d := pt{U: 0, V: 0}, pt{U: 20, V: 0}, pt{U: 20, V: 20}, pt{U: 0, V: 20}
	loop := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(a, b), line(b, c), line(c, notch.End), notch, line(notch.Start, d), line(d, a),
	}}
	requirePrismLocations(t, payloadOf(t, momentinput.Profile{Outer: loop}), []prismCase{
		{name: "in the notch", at: r3.NewVec(10, 18, 5), want: Outside},
		{name: "notch floor", at: r3.NewVec(10, 16, 5), want: OnBoundary},
		{name: "under the notch", at: r3.NewVec(10, 15, 5), want: Inside},
		{name: "beside the notch", at: r3.NewVec(3, 18, 5), want: Inside},
	})
}
