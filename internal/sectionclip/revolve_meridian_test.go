package sectionclip_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionclip"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func meridianLoop(points ...[2]float64) sectionrecord.LoopRecord {
	loop := sectionrecord.LoopRecord{}
	for i, p := range points {
		q := points[(i+1)%len(points)]
		loop.Segments = append(loop.Segments, sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: p[0], V: p[1]}, End: sectionrecord.Point2{U: q[0], V: q[1]},
			TStart: 0, TEnd: 1,
		})
	}
	return loop
}

func meridianCircle(u, v, radius float64, ccw bool) sectionrecord.CircleSeg {
	t0, t1 := 0.0, 1.0
	if !ccw {
		t0, t1 = 1, 0
	}
	return sectionrecord.CircleSeg{
		Center: sectionrecord.Point2{U: u, V: v}, Radius: units.Millimeters(radius), CCW: ccw, TStart: t0, TEnd: t1,
	}
}

func meridianFrame(t *testing.T, origin, u, v r3.Vec) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	return frame
}

// meridianPayload revolves a world-XY meridian region at u >= 0 a full turn
// about world Y; the oriented axis runs along -Y so the region has rho = u.
func meridianPayload(t *testing.T, outer sectionrecord.LoopRecord, holes ...sectionrecord.LoopRecord) revolveplan.Payload {
	t.Helper()
	return revolveplan.Payload{
		Profile: momentinput.Profile{Outer: outer, Holes: holes},
		Frame:   meridianFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1}),
		Ax:      revolveaxis.Frame{DU: 0, DV: -1, SnapTol: 1e-9},
		Full:    true,
		Xform:   r3.Identity(),
	}
}

// steppedMeridian is r=10 for v 0..20 and r=6 for v 20..40.
func steppedMeridian(t *testing.T) revolveplan.Payload {
	t.Helper()
	return meridianPayload(t, meridianLoop([2]float64{0, 0}, [2]float64{10, 0}, [2]float64{10, 20},
		[2]float64{6, 20}, [2]float64{6, 40}, [2]float64{0, 40}))
}

func axialPlane(t *testing.T) r3.Frame {
	t.Helper()
	return meridianFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
}

func normalPlane(t *testing.T, v float64) r3.Frame {
	t.Helper()
	return meridianFrame(t, r3.Vec{Y: v}, r3.Vec{Z: 1}, r3.Vec{X: 1})
}

func circleRadius(t *testing.T, segment sectionrecord.CurveSegment) float64 {
	t.Helper()
	circle, ok := segment.(sectionrecord.CircleSeg)
	require.True(t, ok)
	radius, err := circle.Radius.In(units.Millimeter)
	require.NoError(t, err)
	return radius
}

func TestRevolveMeridianSectionRecords(t *testing.T) {
	rp := steppedMeridian(t)
	records, ok := sectionclip.RevolveAxialSection(rp, axialPlane(t))
	require.True(t, ok)
	require.Len(t, records, 1)
	// Five off-axis lines and their five mirrors; the axis run is gone.
	require.Len(t, records[0].Outer.Segments, 10)
	require.Empty(t, records[0].Holes)

	records, ok = sectionclip.RevolveNormalSection(rp, normalPlane(t, 30))
	require.True(t, ok)
	require.Len(t, records, 1)
	require.Equal(t, 6.0, circleRadius(t, records[0].Outer.Segments[0]))

	// A quarter fillet of radius 5 about (5, 15) crosses v = 18 at u = 9
	// only; the circle's other crossing, u = 1, is off the arc.
	fillet := meridianPayload(t, sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: sectionrecord.Point2{}, End: sectionrecord.Point2{U: 10}, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 10}, End: sectionrecord.Point2{U: 10, V: 15}, TEnd: 1},
		sectionrecord.ArcSeg{
			Center: sectionrecord.Point2{U: 5, V: 15}, Start: sectionrecord.Point2{U: 10, V: 15},
			End: sectionrecord.Point2{U: 5, V: 20}, TEnd: 1,
		},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 5, V: 20}, End: sectionrecord.Point2{V: 20}, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{V: 20}, End: sectionrecord.Point2{}, TEnd: 1},
	}})
	records, ok = sectionclip.RevolveNormalSection(fillet, normalPlane(t, 18))
	require.True(t, ok)
	require.Len(t, records, 1)
	require.Equal(t, 9.0, circleRadius(t, records[0].Outer.Segments[0]))
	_, ok = sectionclip.RevolveAxialSection(fillet, axialPlane(t))
	require.True(t, ok)
}

func TestRevolveMeridianSectionSourceGates(t *testing.T) {
	// A turn about the revolve axis keeps the axis a signed world axis.
	rotated, err := r3.Rotation(r3.Vec{Y: 1}, units.Degrees(30))
	require.NoError(t, err)
	for _, tc := range []struct {
		name   string
		mutate func(*revolveplan.Payload)
	}{
		{name: "partial sweep", mutate: func(rp *revolveplan.Payload) { rp.Full = false }},
		{name: "sheet", mutate: func(rp *revolveplan.Payload) { rp.SurfaceResult = true }},
		{name: "section displacement", mutate: func(rp *revolveplan.Payload) { rp.SectionDelta = 1e-9 }},
		{name: "axis anchor bound", mutate: func(rp *revolveplan.Payload) { rp.Ax.AUBound = 1e-12 }},
		{name: "axis direction bound", mutate: func(rp *revolveplan.Payload) { rp.Ax.DVBound = 1e-12 }},
		{name: "axis anchor V bound", mutate: func(rp *revolveplan.Payload) { rp.Ax.AVBound = 1e-12 }},
		{name: "axis direction U bound", mutate: func(rp *revolveplan.Payload) { rp.Ax.DUBound = 1e-12 }},
		// A negative NaN is the non-finite value whose raw bits read as a
		// huge negative number, the reading that would pass the later gates.
		{name: "nonfinite snap tolerance", mutate: func(rp *revolveplan.Payload) {
			rp.Ax.SnapTol = math.Copysign(math.NaN(), -1)
			rp.Profile = momentinput.Profile{Outer: meridianLoop([2]float64{1e-10, 0}, [2]float64{10, 0},
				[2]float64{10, 20}, [2]float64{1e-10, 20})}
		}},
		{name: "nonfinite axis anchor", mutate: func(rp *revolveplan.Payload) { rp.Ax.AU = math.Copysign(math.NaN(), -1) }},
		// The anchor's coordinate along the axis cancels out of every mapped
		// point, the level and the centre, so only the finiteness gate
		// refuses it.
		{name: "nonfinite axis anchor along the axis", mutate: func(rp *revolveplan.Payload) { rp.Ax.AV = math.NaN() }},
		{name: "negative snap tolerance", mutate: func(rp *revolveplan.Payload) { rp.Ax.SnapTol = -1 }},
		{name: "nonfinite point", mutate: func(rp *revolveplan.Payload) {
			line := rp.Profile.Outer.Segments[5].(sectionrecord.LineSeg)
			line.Start.V = math.NaN()
			rp.Profile.Outer.Segments[5] = line
		}},
		{name: "zero-length line", mutate: func(rp *revolveplan.Payload) {
			line := rp.Profile.Outer.Segments[1].(sectionrecord.LineSeg)
			line.End = line.Start
			rp.Profile.Outer.Segments[1] = line
		}},
		{name: "partial circle", mutate: func(rp *revolveplan.Payload) {
			circle := meridianCircle(5, 12, 2, false)
			circle.TStart = 0.5
			rp.Profile.Holes = []sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{circle}}}
		}},
		{name: "radial admission", mutate: func(rp *revolveplan.Payload) { rp.Ax.RadialAdmitAllow = 1e-12 }},
		{name: "snap allowance", mutate: func(rp *revolveplan.Payload) { rp.Ax.Snap.Area = 1e-12 }},
		// No binary64 direction off the world axes has unit length; this one
		// keeps every tube coordinate exact.
		{name: "tilted axis", mutate: func(rp *revolveplan.Payload) {
			rp.Ax.DU, rp.Ax.DV = 0.75, -0.5
			rp.Profile = momentinput.Profile{Outer: meridianLoop([2]float64{4, 0}, [2]float64{10, 0},
				[2]float64{10, 20}, [2]float64{4, 20})}
		}},
		{name: "tilted placement", mutate: func(rp *revolveplan.Payload) { rp.Xform = rotated }},
		{name: "tilted source frame", mutate: func(rp *revolveplan.Payload) {
			// V, and so the axis, stays a world axis; U does not.
			rp.Frame = meridianFrame(t, r3.Vec{}, r3.Vec{X: 0.6, Z: 0.8}, r3.Vec{Y: 1})
		}},
		{name: "partial line", mutate: func(rp *revolveplan.Payload) {
			line := rp.Profile.Outer.Segments[1].(sectionrecord.LineSeg)
			line.TStart = 0.25
			rp.Profile.Outer.Segments[1] = line
		}},
		{name: "point within snap", mutate: func(rp *revolveplan.Payload) {
			rp.Profile = momentinput.Profile{Outer: meridianLoop([2]float64{1e-10, 0}, [2]float64{10, 0},
				[2]float64{10, 20}, [2]float64{1e-10, 20})}
		}},
		{name: "point across axis", mutate: func(rp *revolveplan.Payload) {
			// The level-10 cut never meets the vertex at u = -1.
			rp.Profile = momentinput.Profile{Outer: meridianLoop([2]float64{0, 0}, [2]float64{10, 0},
				[2]float64{10, 20}, [2]float64{6, 20}, [2]float64{6, 40}, [2]float64{-1, 40}, [2]float64{0, 30})}
		}},
		{name: "arc ends off one circle", mutate: func(rp *revolveplan.Payload) {
			rp.Profile.Outer.Segments[2] = sectionrecord.ArcSeg{
				Center: sectionrecord.Point2{U: 8, V: 20}, Start: sectionrecord.Point2{U: 10, V: 20},
				End: sectionrecord.Point2{U: 6, V: 20.5}, TEnd: 1,
			}
		}},
		{name: "spline", mutate: func(rp *revolveplan.Payload) {
			rp.Profile.Outer.Segments[2] = sectionrecord.SplineSeg{Control: []sectionrecord.Point2{
				{U: 10, V: 20}, {U: 9, V: 20}, {U: 7, V: 20}, {U: 6, V: 20},
			}, TEnd: 1}
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			rp := steppedMeridian(t)
			_, ok := sectionclip.RevolveAxialSection(rp, axialPlane(t))
			require.True(t, ok)
			_, ok = sectionclip.RevolveNormalSection(rp, normalPlane(t, 10))
			require.True(t, ok)
			tc.mutate(&rp)
			_, ok = sectionclip.RevolveAxialSection(rp, axialPlane(t))
			require.False(t, ok)
			_, ok = sectionclip.RevolveNormalSection(rp, normalPlane(t, 10))
			require.False(t, ok)
		})
	}
}

func TestRevolveMeridianAxialSectionGates(t *testing.T) {
	lowerHalf := meridianPayload(t, sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{
			Center: sectionrecord.Point2{V: 10}, Start: sectionrecord.Point2{V: 15},
			End: sectionrecord.Point2{V: 5}, TEnd: 1,
		},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{V: 5}, End: sectionrecord.Point2{V: 15}, TEnd: 1},
	}})
	// A half turn about (3, 10) from (6, 10) to (0, 10) arrives on the axis
	// tangent to it, at the circle's point nearest the axis.
	tangentEnd := meridianPayload(t, sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: sectionrecord.Point2{}, End: sectionrecord.Point2{U: 6}, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 6}, End: sectionrecord.Point2{U: 6, V: 10}, TEnd: 1},
		sectionrecord.ArcSeg{
			Center: sectionrecord.Point2{U: 3, V: 10}, Start: sectionrecord.Point2{U: 6, V: 10},
			End: sectionrecord.Point2{V: 10}, TEnd: 1,
		},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{V: 10}, End: sectionrecord.Point2{}, TEnd: 1},
	}})
	for _, tc := range []struct {
		name  string
		rp    revolveplan.Payload
		plane r3.Frame
	}{
		{name: "plane off the axis", rp: steppedMeridian(t),
			plane: meridianFrame(t, r3.Vec{Z: 1}, r3.Vec{X: 1}, r3.Vec{Y: 1})},
		{name: "plane across the axis", rp: steppedMeridian(t), plane: normalPlane(t, 0)},
		{name: "noncardinal plane", rp: steppedMeridian(t),
			plane: meridianFrame(t, r3.Vec{}, r3.Vec{X: 0.6, Z: 0.8}, r3.Vec{Y: 1})},
		{name: "inexact plane coordinate", rp: steppedMeridian(t),
			plane: meridianFrame(t, r3.Vec{X: 1 << 60}, r3.Vec{X: 1}, r3.Vec{Y: 1})},
		{name: "pinch vertex", rp: meridianPayload(t, meridianLoop([2]float64{0, 5}, [2]float64{5, 0},
			[2]float64{5, 10})), plane: axialPlane(t)},
		{name: "second contact on the chain", rp: meridianPayload(t, meridianLoop([2]float64{0, 0},
			[2]float64{5, 0}, [2]float64{5, 10}, [2]float64{0, 15}, [2]float64{5, 20}, [2]float64{5, 30},
			[2]float64{0, 30})), plane: axialPlane(t)},
		{name: "two axis runs", rp: meridianPayload(t, meridianLoop([2]float64{0, 0}, [2]float64{5, 0},
			[2]float64{5, 30}, [2]float64{0, 30}, [2]float64{0, 20}, [2]float64{3, 20}, [2]float64{3, 10},
			[2]float64{0, 10})), plane: axialPlane(t)},
		{name: "hole on the axis", rp: meridianPayload(t, meridianLoop([2]float64{0, 0}, [2]float64{10, 0},
			[2]float64{10, 20}, [2]float64{0, 20}), meridianLoop([2]float64{0, 8}, [2]float64{2, 12},
			[2]float64{4, 8})), plane: axialPlane(t)},
		{name: "circle reaching the axis", rp: meridianPayload(t, sectionrecord.LoopRecord{
			Segments: []sectionrecord.CurveSegment{meridianCircle(2, 10, 3, true)},
		}), plane: axialPlane(t)},
		{name: "arc through the far side", rp: lowerHalf, plane: axialPlane(t)},
		{name: "arc tangent to the axis at its end", rp: tangentEnd, plane: axialPlane(t)},
		{name: "hole circle reaching the axis", rp: meridianPayload(t, meridianLoop([2]float64{0, 0},
			[2]float64{10, 0}, [2]float64{10, 20}, [2]float64{0, 20}), sectionrecord.LoopRecord{
			Segments: []sectionrecord.CurveSegment{meridianCircle(2, 10, 3, false)},
		}), plane: axialPlane(t)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, ok := sectionclip.RevolveAxialSection(tc.rp, tc.plane)
			require.False(t, ok)
		})
	}
	// Each gate refuses on its own: the same meridians admit where the
	// refused contact is absent.
	_, ok := sectionclip.RevolveAxialSection(meridianPayload(t, meridianLoop([2]float64{1, 5}, [2]float64{5, 0},
		[2]float64{5, 10})), axialPlane(t))
	require.True(t, ok)
	records, ok := sectionclip.RevolveAxialSection(meridianPayload(t, sectionrecord.LoopRecord{
		Segments: []sectionrecord.CurveSegment{meridianCircle(4, 10, 3, true)},
	}), axialPlane(t))
	require.True(t, ok)
	require.Len(t, records, 2)
}

func TestRevolveMeridianNormalSectionGates(t *testing.T) {
	slanted := meridianPayload(t, meridianLoop([2]float64{0, 5}, [2]float64{3, 0}, [2]float64{3, 10}))
	ring := meridianPayload(t, sectionrecord.LoopRecord{
		Segments: []sectionrecord.CurveSegment{meridianCircle(10, 10, 5, true)},
	})
	kissing := meridianPayload(t, sectionrecord.LoopRecord{
		Segments: []sectionrecord.CurveSegment{meridianCircle(3, 10, 3, true)},
	})
	touching := meridianPayload(t, meridianLoop([2]float64{0, 0}, [2]float64{10, 0}, [2]float64{10, 20},
		[2]float64{0, 20}), sectionrecord.LoopRecord{
		Segments: []sectionrecord.CurveSegment{meridianCircle(6, 10, 4, false)},
	})
	crossing := meridianPayload(t, sectionrecord.LoopRecord{
		Segments: []sectionrecord.CurveSegment{meridianCircle(2, 10, 3, true)},
	})
	shifted := steppedMeridian(t)
	move, err := r3.Translation(r3.Vec{X: 0.1})
	require.NoError(t, err)
	shifted.Xform = move
	for _, tc := range []struct {
		name  string
		rp    revolveplan.Payload
		plane r3.Frame
	}{
		{name: "vertex level", rp: steppedMeridian(t), plane: normalPlane(t, 20)},
		{name: "cap level", rp: steppedMeridian(t), plane: normalPlane(t, 0)},
		{name: "exterior", rp: steppedMeridian(t), plane: normalPlane(t, 50)},
		{name: "plane holding the axis", rp: steppedMeridian(t),
			plane: meridianFrame(t, r3.Vec{Y: 10}, r3.Vec{X: 1}, r3.Vec{Y: 1})},
		{name: "open loop", rp: meridianPayload(t, sectionrecord.LoopRecord{
			Segments: meridianLoop([2]float64{0, 0}, [2]float64{10, 0}, [2]float64{10, 20},
				[2]float64{0, 20}).Segments[:3],
		}), plane: normalPlane(t, 10)},
		// At a = 2^-60 the line from rho 1 to rho 2 crosses at 1 + 2^-60.
		{name: "crossing radius beyond binary64", rp: meridianPayload(t, meridianLoop([2]float64{1, 0},
			[2]float64{2, -1}, [2]float64{2, 1})), plane: normalPlane(t, -0x1p-60)},
		{name: "inexact line crossing", rp: slanted, plane: normalPlane(t, 1)},
		{name: "tangent circle", rp: ring, plane: normalPlane(t, 15)},
		{name: "inexact square root", rp: ring, plane: normalPlane(t, 12)},
		{name: "crossing on the axis off an axis line", rp: kissing, plane: normalPlane(t, 10)},
		{name: "two loops at one radius", rp: touching, plane: normalPlane(t, 10)},
		{name: "circle across the axis", rp: crossing, plane: normalPlane(t, 10)},
		{name: "inexact centre", rp: shifted,
			plane: meridianFrame(t, r3.Vec{X: 1 << 20, Y: 10}, r3.Vec{Z: 1}, r3.Vec{X: 1})},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, ok := sectionclip.RevolveNormalSection(tc.rp, tc.plane)
			require.False(t, ok)
		})
	}
	// The same sources admit at a nearby level or plane origin.
	for _, tc := range []struct {
		name  string
		rp    revolveplan.Payload
		plane r3.Frame
		radii [2]float64
	}{
		{name: "ring", rp: ring, plane: normalPlane(t, 13), radii: [2]float64{6, 14}},
		{name: "slanted", rp: slanted, plane: normalPlane(t, 2.5), radii: [2]float64{1.5, 3}},
		{name: "shifted", rp: shifted, plane: normalPlane(t, 10), radii: [2]float64{0, 10}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			records, ok := sectionclip.RevolveNormalSection(tc.rp, tc.plane)
			require.True(t, ok)
			require.Len(t, records, 1)
			require.Equal(t, tc.radii[1], circleRadius(t, records[0].Outer.Segments[0]))
			if tc.radii[0] == 0 {
				require.Empty(t, records[0].Holes)
				return
			}
			require.Len(t, records[0].Holes, 1)
			require.Equal(t, tc.radii[0], circleRadius(t, records[0].Holes[0].Segments[0]))
		})
	}
}
