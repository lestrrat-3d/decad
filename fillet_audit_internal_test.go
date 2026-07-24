package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestNestingAuditCancellationReachesSectionBBox(t *testing.T) {
	loop := LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 10, V: 0}, TEnd: 1},
		LineSeg{Start: Point2{U: 10, V: 0}, End: Point2{U: 10, V: 10}, TEnd: 1},
		LineSeg{Start: Point2{U: 10, V: 10}, End: Point2{U: 0, V: 10}, TEnd: 1},
		LineSeg{Start: Point2{U: 0, V: 10}, End: Point2{U: 0, V: 0}, TEnd: 1},
	}}
	segs, err := buildSegEntries([]LoopRecord{loop})
	require.NoError(t, err)

	calls := 0
	budget := &workBudget{
		stepFn: func() error {
			calls++
			if calls == len(segs)+1 {
				return context.Canceled
			}
			return nil
		},
		errFn: func() error { return nil },
	}
	err = nestingAuditBudget(budget, segs, 2)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, len(segs)+1, calls,
		`the nesting audit must spend the shared budget in its section bounding-box scan`)
}

// TestContactFloorUsesTrueSectionBBox confirms the §5 noise floor δ = ε·D reads
// the TRUE section (u, v) bounding box — an arc's own extrema, not its endpoint
// chord. An arc bulges outside its endpoint box, so an endpoint-only D
// understates the section diameter the doc claims.
func TestContactFloorUsesTrueSectionBBox(t *testing.T) {
	t.Run("a semicircle bulges outside its endpoint chord", func(t *testing.T) {
		// A CCW semicircle from (10,0) to (−10,0) about the origin bulges to
		// (0,10). Its endpoint box is 20 wide, 0 tall (diagonal 20); the TRUE box
		// is 20 wide, 10 tall (diagonal √500).
		w, err := walkOf(ArcSeg{
			Center: Point2{U: 0, V: 0},
			Start:  Point2{U: 10, V: 0},
			End:    Point2{U: -10, V: 0},
			TEnd:   1,
		})
		require.NoError(t, err)
		segs := []segEntry{{loop: 0, idx: 0, n: 1, w: w}}

		got := contactFloor(segs)
		wantTrue := contactEps * math.Hypot(20, 10)     // true bbox diagonal √500
		wantEndpoints := contactEps * math.Hypot(20, 0) // the old endpoint box
		require.InEpsilon(t, wantTrue, got, 1e-12, `contactFloor must read the true section bbox`)
		require.Greater(t, got, wantEndpoints, `the true D strictly exceeds the endpoint box when an arc bulges`)
	})

	t.Run("a full circle's endpoints collapse to a point", func(t *testing.T) {
		// A whole circle of radius 5 is recorded as one closed segment whose start
		// == end, so its endpoint box is a single point (diagonal 0); the TRUE box
		// is 10×10 (diagonal √200).
		w, err := walkOf(CircleSeg{
			Center: Point2{U: 0, V: 0},
			Radius: units.Millimeters(5),
			CCW:    true,
			TEnd:   1,
		})
		require.NoError(t, err)
		segs := []segEntry{{loop: 0, idx: 0, n: 1, w: w}}

		got := contactFloor(segs)
		want := contactEps * math.Hypot(10, 10) // true bbox diagonal √200
		require.InEpsilon(t, want, got, 1e-12,
			`a full circle's true D is its diameter's diagonal, not the collapsed endpoint`)
	})
}

// TestAuditRewriteSingleCornerOverrunIsUnsupported pins the §5 audit's Table S
// S6 verdict for a SINGLE corner whose own setback runs past the far end of its
// adjacent walls. Such an overrun also flips the rewritten loop's signed area,
// so S8 (orientation, asked first) could misfile it as ErrDegenerate; Table S
// assigns the overrun to S6, so the audit must return ErrUnsupported — and NOT
// ErrDegenerate — even though the loop genuinely inverted. The two-corner case
// (both ends of one wall) is exercised end-to-end by TestChamferOverLargeSetbackRefused.
func TestAuditRewriteSingleCornerOverrunIsUnsupported(t *testing.T) {
	// A CCW 100×60 rectangle: positive signed area, four right-angle corners,
	// corner 0 at the origin (arriving down the left wall, leaving along the
	// bottom wall). Both walls at corner 0 are shorter than the setback below.
	rect := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 100, V: 0}, TEnd: 1},
		LineSeg{Start: Point2{U: 100, V: 0}, End: Point2{U: 100, V: 60}, TEnd: 1},
		LineSeg{Start: Point2{U: 100, V: 60}, End: Point2{U: 0, V: 60}, TEnd: 1},
		LineSeg{Start: Point2{U: 0, V: 60}, End: Point2{U: 0, V: 0}, TEnd: 1},
	}}}

	// blendOne blends exactly ONE corner of the section (corner 0) at an
	// over-large setback, rewrites the profile, and returns the loops and per-loop
	// blend maps the audit reads.
	blendOne := func(t *testing.T, cb *cornerBlend, loops []cornerLoop) (ProfileRecord, []map[int]*cornerBlend) {
		t.Helper()
		blendAt := []map[int]*cornerBlend{{0: cb}}
		rewritten, _ := rewriteProfile(rect, loops, blendAt)
		return rewritten, blendAt
	}

	origArea, err := loopSignedArea(rect.Outer)
	require.NoError(t, err)

	for _, tc := range []struct {
		name string
		// flips is true when this op's over-large single-corner blend inverts the
		// rewritten loop's signed area — the case S8 (asked first) would grab. The
		// chamfer's chord does; the fillet's arc bulges back and keeps the area
		// positive, so its overrun is caught by S6's own report instead. Either
		// path must reach ErrUnsupported.
		flips bool
		blend func(loops []cornerLoop) (*cornerBlend, error)
	}{
		{"chamfer", true, func(loops []cornerLoop) (*cornerBlend, error) { return computeChamfer(loops[0], 0, 120) }},
		{"fillet", false, func(loops []cornerLoop) (*cornerBlend, error) { return computeFillet(loops[0], 0, 120) }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			loops := sectionCornerLoops(t, rect)
			cb, err := tc.blend(loops)
			require.NoError(t, err, `an over-large blend of a right-angle corner still constructs — the overrun is a §5 audit verdict, not a construction gate`)

			rewritten, blendAt := blendOne(t, cb, loops)

			newArea, err := loopSignedArea(rewritten.Outer)
			require.NoError(t, err)
			require.Equal(t, tc.flips, math.Signbit(origArea) != math.Signbit(newArea),
				`the rewritten loop's signed-area flip must match the case's expectation`)

			err = auditRewrite(rect, rewritten, loops, blendAt)
			require.ErrorIs(t, err, ErrUnsupported,
				`a single corner's overrun is Table S's S6, not the S8 inside-out verdict`)
			require.NotErrorIs(t, err, ErrDegenerate,
				`the overrun must not be misfiled as S8 ErrDegenerate even when the loop flipped`)
		})
	}
}

// sectionCornerLoops resolves a section's loops into coalesced corner walks, the
// same decomposition prismCornerLoops runs, so an internal audit test can drive
// the real computeChamfer/computeFillet path on a hand-built section.
func sectionCornerLoops(t *testing.T, prof ProfileRecord) []cornerLoop {
	t.Helper()
	var out []cornerLoop
	for _, loop := range append([]LoopRecord{prof.Outer}, prof.Holes...) {
		raw := make([]sideWalk, len(loop.Segments))
		for i, seg := range loop.Segments {
			w, err := walkOf(seg)
			require.NoError(t, err)
			raw[i] = sideWalk{segmentWalk: w, segs: []int{i}}
		}
		out = append(out, cornerLoop{walks: coalesceWalks(raw)})
	}
	return out
}

// TestCrossingAuditRejectsBoundaryContact exercises the §5 audit's crossing
// test directly on hand-built loops: it must reject not only an interior
// crossing but any BOUNDARY CONTACT — a tangency or a shared boundary point
// with no interior crossing — since a large fillet can bring the rewritten
// section's loops into contact without ever crossing them, and an interior-only
// test would pass that pinch (S7, ErrUnsupported).
func TestCrossingAuditRejectsBoundaryContact(t *testing.T) {
	v := 20 / math.Sqrt2 // a point on the quarter arc of radius 20 at 45°

	t.Run("inter-loop crossing", func(t *testing.T) {
		horizontal := LoopRecord{Segments: []CurveSegment{
			LineSeg{Start: Point2{U: -10, V: 0}, End: Point2{U: 10, V: 0}, TEnd: 1},
		}}
		vertical := LoopRecord{Segments: []CurveSegment{
			LineSeg{Start: Point2{U: 0, V: -10}, End: Point2{U: 0, V: 10}, TEnd: 1},
		}}
		segs, err := buildSegEntries([]LoopRecord{horizontal, vertical})
		require.NoError(t, err)
		err = crossingAudit(segs)
		require.ErrorIs(t, err, ErrUnsupported, "two loop segments crossing in their interiors are unsupported")
		require.ErrorContains(t, renderAuditCoordinates(err), `rewritten loop 0 segment 0 and loop 1 segment 0 cross`,
			`the refusal names both loop segments in the crossing pair`)
	})

	t.Run("inter-loop shared point: a hole vertex on an outer arc", func(t *testing.T) {
		// loop0 is a quarter disk whose curved wall is an arc of radius 20 about
		// the origin; loop1 is a separate triangle one of whose vertices lands
		// exactly on that arc. The interiors never cross — the touch is at the
		// triangle vertex — yet the loops are not disjoint.
		quarterDisk := LoopRecord{Segments: []CurveSegment{
			LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 20, V: 0}, TEnd: 1},
			ArcSeg{Center: Point2{U: 0, V: 0}, Start: Point2{U: 20, V: 0}, End: Point2{U: 0, V: 20}, TEnd: 1},
			LineSeg{Start: Point2{U: 0, V: 20}, End: Point2{U: 0, V: 0}, TEnd: 1},
		}}
		triangleOnArc := LoopRecord{Segments: []CurveSegment{
			LineSeg{Start: Point2{U: v, V: v}, End: Point2{U: 30, V: 30}, TEnd: 1},
			LineSeg{Start: Point2{U: 30, V: 30}, End: Point2{U: 30, V: v}, TEnd: 1},
			LineSeg{Start: Point2{U: 30, V: v}, End: Point2{U: v, V: v}, TEnd: 1},
		}}
		segs, err := buildSegEntries([]LoopRecord{quarterDisk, triangleOnArc})
		require.NoError(t, err)
		err = crossingAudit(segs)
		require.ErrorIs(t, err, ErrUnsupported, "a hole vertex touching an outer arc is boundary contact")
		require.ErrorContains(t, renderAuditCoordinates(err), `rewritten loop 0 segment 1 and loop 1 segment 0 are in contact`,
			`the refusal names both loop segments in the contacting pair`)
	})

	t.Run("same-loop non-adjacent touch: a vertex on a far edge", func(t *testing.T) {
		// A single loop whose segment 2 ends exactly on the interior of segment 0
		// (x = 10 on the x axis). Segments 0 and 2 are non-adjacent, so the audit
		// must see the pinch; the touch is at segment 2's endpoint, not an
		// interior crossing, so the interior-only test misses it.
		pinched := LoopRecord{Segments: []CurveSegment{
			LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 20, V: 0}, TEnd: 1},
			LineSeg{Start: Point2{U: 20, V: 0}, End: Point2{U: 20, V: 20}, TEnd: 1},
			LineSeg{Start: Point2{U: 20, V: 20}, End: Point2{U: 10, V: 0}, TEnd: 1},
			LineSeg{Start: Point2{U: 10, V: 0}, End: Point2{U: 10, V: -10}, TEnd: 1},
			LineSeg{Start: Point2{U: 10, V: -10}, End: Point2{U: 0, V: 0}, TEnd: 1},
		}}
		segs, err := buildSegEntries([]LoopRecord{pinched})
		require.NoError(t, err)
		err = crossingAudit(segs)
		require.ErrorIs(t, err, ErrUnsupported, "a loop touching itself away from its shared vertices is a pinch")
		require.ErrorContains(t, renderAuditCoordinates(err), `rewritten loop 0 segment 0 and loop 0 segment 2 are in contact`,
			`the refusal names both loop segments in the contacting pair`)
	})
}

// TestCrossingAuditAcceptsDisjointLoops confirms the widened test does not
// false-reject: loops that stay properly apart pass, and the adjacent-endpoint
// exemption still lets legitimately consecutive segments share their vertex.
func TestCrossingAuditAcceptsDisjointLoops(t *testing.T) {
	t.Run("two well-separated loops", func(t *testing.T) {
		a := LoopRecord{Segments: []CurveSegment{
			LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 10, V: 0}, TEnd: 1},
			LineSeg{Start: Point2{U: 10, V: 0}, End: Point2{U: 10, V: 10}, TEnd: 1},
			LineSeg{Start: Point2{U: 10, V: 10}, End: Point2{U: 0, V: 10}, TEnd: 1},
			LineSeg{Start: Point2{U: 0, V: 10}, End: Point2{U: 0, V: 0}, TEnd: 1},
		}}
		b := LoopRecord{Segments: []CurveSegment{
			LineSeg{Start: Point2{U: 100, V: 100}, End: Point2{U: 110, V: 100}, TEnd: 1},
			LineSeg{Start: Point2{U: 110, V: 100}, End: Point2{U: 110, V: 110}, TEnd: 1},
			LineSeg{Start: Point2{U: 110, V: 110}, End: Point2{U: 100, V: 100}, TEnd: 1},
		}}
		segs, err := buildSegEntries([]LoopRecord{a, b})
		require.NoError(t, err)
		require.NoError(t, crossingAudit(segs))
	})

	t.Run("adjacent segments legitimately share their endpoint", func(t *testing.T) {
		// Every consecutive pair of this convex quad shares a vertex; the
		// adjacency exemption keeps those from reading as contact.
		quad := LoopRecord{Segments: []CurveSegment{
			LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 10, V: 0}, TEnd: 1},
			LineSeg{Start: Point2{U: 10, V: 0}, End: Point2{U: 10, V: 10}, TEnd: 1},
			LineSeg{Start: Point2{U: 10, V: 10}, End: Point2{U: 0, V: 10}, TEnd: 1},
			LineSeg{Start: Point2{U: 0, V: 10}, End: Point2{U: 0, V: 0}, TEnd: 1},
		}}
		segs, err := buildSegEntries([]LoopRecord{quad})
		require.NoError(t, err)
		require.NoError(t, crossingAudit(segs))
	})
}
