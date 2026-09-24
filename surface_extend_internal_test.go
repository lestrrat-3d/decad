package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file checks, as geometry, the one claim fullExtendSegment's own comment
// derives: the scene entity buildPrismScene creates from the full-domain
// recreation carries the RECEIVER'S OWN parameterisation, so a cut parameter
// sketch reports on that entity indexes the receiver's record directly and
// resolveExtend stores it without a map.
//
// It is an internal fixture because two of the three admitted kinds cannot be
// driven through Body.Extend at all today. Only a CircleSeg is affected in
// practice: a circle fragment always separates the disk's interior from the
// exterior, so it is never a bridge in sketch's arrangement and never reaches
// Chains() — and ExtrudeChain is the one producer of an undisplaced
// chainPayload, Trim's always carrying the nonzero sectionDelta S7 refuses. The
// arm stays because a receiver's non-extended segments may be circles and a
// future producer may reach it; it is proven here rather than left unexercised.
// The LineSeg and ArcSeg arms have public fixtures of their own in
// surface_extend_test.go, and this file checks both recorded senses of all
// three in one place.

// extendTestSquare is the far-away tool operand the scene needs as its second
// operand. Nothing here reads it; it exists so buildPrismScene builds a scene
// at all.
func extendTestSquare() ProfileRecord {
	pts := []Point2{{U: 1000, V: 1000}, {U: 1010, V: 1000}, {U: 1010, V: 1010}, {U: 1000, V: 1010}}
	segs := make([]CurveSegment, len(pts))
	for i := range pts {
		segs[i] = LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	return ProfileRecord{Outer: LoopRecord{Segments: segs}}
}

// recordPointAt evaluates a recorded entity's OWN natural parameterisation at
// t, from the record's defining fields alone and ignoring its recorded range —
// the parameterisation walkOf reads (segment_walk.go) and the one a stored
// TStart/TEnd indexes.
func recordPointAt(t *testing.T, seg CurveSegment, param float64) (float64, float64) {
	t.Helper()
	switch s := seg.(type) {
	case LineSeg:
		return s.Start.U + param*(s.End.U-s.Start.U), s.Start.V + param*(s.End.V-s.Start.V)
	case CircleSeg:
		r, err := s.Radius.In(units.Millimeter)
		require.NoError(t, err)
		ang := 2 * math.Pi * param
		return s.Center.U + r*math.Cos(ang), s.Center.V + r*math.Sin(ang)
	case ArcSeg:
		r := math.Hypot(s.Start.U-s.Center.U, s.Start.V-s.Center.V)
		a0 := math.Atan2(s.Start.V-s.Center.V, s.Start.U-s.Center.U)
		a1 := math.Atan2(s.End.V-s.Center.V, s.End.U-s.Center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		ang := a0 + param*sweep
		return s.Center.U + r*math.Cos(ang), s.Center.V + r*math.Sin(ang)
	default:
		t.Fatalf("no recorded parameterisation for %T", seg)
		return 0, 0
	}
}

// scenePointAt evaluates a sketch entity's own parameterisation at t, on
// sketch's published conventions (geom/sample.go): a line runs its Start to its
// End, a circle runs a full turn counter-clockwise from the +u axis, and an arc
// runs its counter-clockwise sweep from its Start.
func scenePointAt(t *testing.T, e sketch.Entity, param float64) (float64, float64) {
	t.Helper()
	switch ent := e.(type) {
	case *sketch.Line:
		g := ent.Geometry()
		return g.Start.X + param*(g.End.X-g.Start.X), g.Start.Y + param*(g.End.Y-g.Start.Y)
	case *sketch.Circle:
		g := ent.Geometry()
		ang := 2 * math.Pi * param
		return g.Center.X + g.Radius*math.Cos(ang), g.Center.Y + g.Radius*math.Sin(ang)
	case *sketch.Arc:
		g := ent.Geometry()
		ang := g.StartAngle() + param*g.Sweep()
		return g.Center.X + g.Radius()*math.Cos(ang), g.Center.Y + g.Radius()*math.Sin(ang)
	default:
		t.Fatalf("no scene parameterisation for %T", e)
		return 0, 0
	}
}

// extendSceneCarrier runs fullExtendSegment and buildPrismScene exactly as
// resolveExtend does, and returns the one receiver-side entity the scene holds.
func extendSceneCarrier(t *testing.T, seg CurveSegment) sketch.Entity {
	t.Helper()
	full, err := fullExtendSegment(seg)
	require.NoError(t, err)
	view := prismPayload{profile: ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{full}}}}
	tool := prismPayload{profile: extendTestSquare()}
	reexpress, err := newPrismReexpression(view, tool)
	require.NoError(t, err)
	require.True(t, reexpress.identity)
	_, tags, _, err := buildPrismScene(newWorkBudget(t.Context()), view, tool, reexpress)
	require.NoError(t, err)
	var carrier sketch.Entity
	found := 0
	for entity, tag := range tags {
		if tag.isB {
			continue
		}
		carrier, found = entity, found+1
	}
	require.Equal(t, 1, found, "the Extend scene holds exactly one receiver entity")
	return carrier
}

func TestExtendFullDomainSceneSharesTheRecordParameterisation(t *testing.T) {
	t.Parallel()
	// Each pair is one recorded entity in both senses: the forward record and
	// the reversed one sketch publishes when a chain walks against the entity's
	// authored direction (seam.go's recordEdge swaps the range order for a
	// Reversed fragment). Both must index the same scene entity identically.
	for _, tc := range []struct {
		name string
		seg  CurveSegment
	}{
		{"line forward", LineSeg{Start: Point2{U: 100, V: 0.1}, End: Point2{U: 0, V: 0.1}, TStart: 0, TEnd: 0.6}},
		{"line reversed", LineSeg{Start: Point2{U: 100, V: 0.1}, End: Point2{U: 0, V: 0.1}, TStart: 0.6, TEnd: 0}},
		{"circle ccw", CircleSeg{Center: Point2{U: 3, V: -7}, Radius: units.Millimeters(50), CCW: true, TStart: 0.25, TEnd: 0.75}},
		{"circle cw", CircleSeg{Center: Point2{U: 3, V: -7}, Radius: units.Millimeters(50), CCW: false, TStart: 0.75, TEnd: 0.25}},
		{"arc forward", ArcSeg{Center: Point2{U: 0, V: 0}, Start: Point2{U: 100, V: 0}, End: Point2{U: -100, V: 0}, TStart: 0, TEnd: 0.5}},
		{"arc reversed", ArcSeg{Center: Point2{U: 0, V: 0}, Start: Point2{U: 100, V: 0}, End: Point2{U: -100, V: 0}, TStart: 0.5, TEnd: 0}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			carrier := extendSceneCarrier(t, tc.seg)
			worst := 0.0
			for _, param := range []float64{0, 0.125, 0.25, 0.5, 0.6, 0.75, 0.875, 1} {
				wantU, wantV := recordPointAt(t, tc.seg, param)
				gotU, gotV := scenePointAt(t, carrier, param)
				worst = math.Max(worst, math.Max(math.Abs(gotU-wantU), math.Abs(gotV-wantV)))
				require.InDelta(t, wantU, gotU, 1e-12, "u at t = %v", param)
				require.InDelta(t, wantV, gotV, 1e-12, "v at t = %v", param)
			}
			t.Logf("%s: worst coordinate disagreement over the shared domain %g mm", tc.name, worst)
		})
	}
}

// TestExtendSetBoundWidensOnlyTheNamedEnd covers extendSetBound's three arms on
// computed geometry: the widened segment must reach the new parameter's own
// point and leave the other end's recorded coordinate exactly where it was.
func TestExtendSetBoundWidensOnlyTheNamedEnd(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		seg     CurveSegment
		atStart bool
		bound   float64
	}{
		{"line reversed at start", LineSeg{Start: Point2{U: 100, V: 0.1}, End: Point2{U: 0, V: 0.1}, TStart: 0.6, TEnd: 0}, true, 0.7},
		{"line forward at end", LineSeg{Start: Point2{U: 0, V: 0.1}, End: Point2{U: 100, V: 0.1}, TStart: 0, TEnd: 0.4}, false, 0.7},
		{"circle at end", CircleSeg{Center: Point2{U: 3, V: -7}, Radius: units.Millimeters(50), CCW: true, TStart: 0.25, TEnd: 0.75}, false, 0.875},
		{"arc reversed at start", ArcSeg{Center: Point2{U: 0, V: 0}, Start: Point2{U: 100, V: 0}, End: Point2{U: -100, V: 0}, TStart: 0.5, TEnd: 0}, true, 0.75},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			t0, t1, err := trimSegmentParamRange(tc.seg)
			require.NoError(t, err)
			kept := t0
			if tc.atStart {
				kept = t1
			}
			widened := extendSetBound(tc.seg, tc.atStart, tc.bound)
			w0, w1, err := trimSegmentParamRange(widened)
			require.NoError(t, err)
			if tc.atStart {
				require.Equal(t, tc.bound, w0)
				require.Equal(t, t1, w1, "the unnamed end keeps its own recorded parameter")
			} else {
				require.Equal(t, t0, w0, "the unnamed end keeps its own recorded parameter")
				require.Equal(t, tc.bound, w1)
			}
			// The named end now stands on the carrier at the new parameter, and
			// the other end has not moved a float.
			wantU, wantV := recordPointAt(t, tc.seg, tc.bound)
			walk, err := walkOf(widened, newFreeformWork())
			require.NoError(t, err)
			movedU, movedV := walk.endU, walk.endV
			stillU, stillV := walk.startU, walk.startV
			if tc.atStart {
				movedU, movedV, stillU, stillV = stillU, stillV, movedU, movedV
			}
			require.InDelta(t, wantU, movedU, 1e-9, "the widened end stands at the new parameter's own point")
			require.InDelta(t, wantV, movedV, 1e-9, "the widened end stands at the new parameter's own point")
			keptU, keptV := recordPointAt(t, tc.seg, kept)
			require.InDelta(t, keptU, stillU, 1e-9, "the unnamed end did not move")
			require.InDelta(t, keptV, stillV, 1e-9, "the unnamed end did not move")
			t.Logf("%s: widened end (%g, %g), unnamed end (%g, %g)", tc.name, movedU, movedV, stillU, stillV)
		})
	}
}
