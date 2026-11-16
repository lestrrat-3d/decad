package prismboolean_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismboolean"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func polar(r, a float64) sectionrecord.Point2 {
	return sectionrecord.Point2{U: r * math.Cos(a), V: r * math.Sin(a)}
}

func poly(pts ...sectionrecord.Point2) sectionrecord.LoopRecord {
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i, p := range pts {
		segs[i] = sectionrecord.LineSeg{Start: p, End: pts[(i+1)%len(pts)], TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

// window is an annular sector hole (a clockwise loop of two radial lines, an
// outer and an inner arc) centred on angle phi.
func window(phi, half, rIn, rOut float64) sectionrecord.LoopRecord {
	c := sectionrecord.Point2{}
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Center: c, Start: polar(rOut, phi-half), End: polar(rOut, phi+half), TStart: 1, TEnd: 0},
		sectionrecord.LineSeg{Start: polar(rOut, phi-half), End: polar(rIn, phi-half), TEnd: 1},
		sectionrecord.ArcSeg{Center: c, Start: polar(rIn, phi-half), End: polar(rIn, phi+half), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: polar(rIn, phi+half), End: polar(rOut, phi+half), TEnd: 1},
	}}
}

func discPayload(t *testing.T, radius float64, holes ...sectionrecord.LoopRecord) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	disc := sectionrecord.CircleSeg{Radius: units.Millimeters(radius), CCW: true, TStart: 0, TEnd: 1}
	return prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: 0, Z1: 3,
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{disc}}, Holes: holes}}
}

// tooth is an all-line trapezoid straddling the rim of a disc of radius 20 at
// angle psi, rotated as a pattern instance rounds it: every datum moved by at
// most delta, which the uniform carrier record states.
func toothPayload(t *testing.T, psi, delta float64) prismrecord.Payload {
	t.Helper()
	p := discPayload(t, 1)
	p.Profile = momentinput.Profile{Outer: poly(polar(18, psi-0.10), polar(24, psi-0.05), polar(24, psi+0.05), polar(18, psi+0.10))}
	if delta > 0 {
		cr := prismrecord.UniformCarrierRecord(p.Profile, delta)
		p.SectionDelta, p.Carriers = delta, &cr
	}
	return p
}

func testEnv() topo.Env { return topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}} }

func unionBodies(t *testing.T, ps ...prismrecord.Payload) (prismrecord.Payload, bool, error) {
	t.Helper()
	bodies := make([]*topo.Body, len(ps))
	for i, p := range ps {
		bodies[i] = &topo.Body{Payload: p}
		if p.Carriers != nil {
			// A section the build refuses stays an unbuilt body with no owned record.
			if body, err := prismrecord.BuildWithRecord(t.Context(), testEnv(), p, *p.Carriers); err == nil {
				bodies[i] = body
			}
		}
	}
	body, ok, err := prismboolean.UnionMany(t.Context(), testEnv(), bodies)
	if err != nil || !ok {
		return prismrecord.Payload{}, ok, err
	}
	out, isPrism := body.Payload.(prismrecord.Payload)
	require.True(t, isPrism)
	return out, true, nil
}

func loopArea(t *testing.T, loop sectionrecord.LoopRecord) float64 {
	t.Helper()
	a, err := sectionaudit.LoopSignedArea(proofbound.NewWorkBudget(t.Context()), loop)
	require.NoError(t, err)
	return a
}

// referenceArea is the area of the same outline drawn in place (a disc, the
// teeth at their own angles, the windows removed), by midpoint sampling.
func referenceArea(teeth []float64, windows [][4]float64) float64 {
	const n = 1400
	lo, hi := -26.0, 26.0
	step := (hi - lo) / n
	inTooth := func(x, y, psi float64) bool {
		// rotate into the tooth's frame (psi along +x) and test the trapezoid
		c, s := math.Cos(psi), math.Sin(psi)
		u, v := x*c+y*s, -x*s+y*c
		if u < 18*math.Cos(0.10) || u > 24*math.Cos(0.05) {
			return false
		}
		// the flanks run from (18cos.1, ±18sin.1) to (24cos.05, ±24sin.05)
		x0, y0 := 18*math.Cos(0.10), 18*math.Sin(0.10)
		x1, y1 := 24*math.Cos(0.05), 24*math.Sin(0.05)
		limit := y0 + (u-x0)*(y1-y0)/(x1-x0)
		return math.Abs(v) <= limit
	}
	count := 0
	for i := range n {
		for j := range n {
			x, y := lo+(float64(i)+0.5)*step, lo+(float64(j)+0.5)*step
			r := math.Hypot(x, y)
			in := r <= 20
			for _, psi := range teeth {
				in = in || inTooth(x, y, psi)
			}
			for _, w := range windows {
				a := math.Atan2(y, x)
				d := math.Abs(math.Remainder(a-w[0], 2*math.Pi))
				if r >= w[2] && r <= w[3] && d <= w[1] {
					in = false
				}
			}
			if in {
				count++
			}
		}
	}
	return float64(count) * step * step
}

func TestUnionManyExtendedClassDiscWindowsAndTeeth(t *testing.T) {
	t.Parallel()
	const delta = 3e-13
	w1, w2 := window(0.6, 0.35, 8, 14), window(3.4, 0.30, 9, 13)
	disc := discPayload(t, 20, w1, w2)
	teeth := []float64{0.0, 2.0, 4.1}
	out, ok, err := unionBodies(t, disc, toothPayload(t, teeth[0], 0), toothPayload(t, teeth[1], delta),
		toothPayload(t, teeth[2], delta))
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, out.Profile.Holes, 2, "the windows stay holes")
	// the teeth join the rim: 3 teeth add 3 lines each... the outer loop has
	// 3 disc arcs between teeth, and every tooth contributes 3 lines
	require.Greater(t, len(out.Profile.Outer.Segments), 9)

	area := loopArea(t, out.Profile.Outer)
	for _, h := range out.Profile.Holes {
		area += loopArea(t, h)
	}
	want := referenceArea(teeth, [][4]float64{{0.6, 0.35, 8, 14}, {3.4, 0.30, 9, 13}})
	require.InEpsilon(t, want, area, 2e-3, "the union's area matches the same outline drawn in place")

	// The result is displaced by the teeth's charge and states a record.
	require.Greater(t, out.SectionDelta, 0.0)
	cr, valid, err := out.CarrierRecord(proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, valid)
	for l := range cr.Segment {
		require.LessOrEqual(t, cr.LoopH(l), out.SectionDelta, "CR4: every loop's h_loop is within SectionDelta")
	}
	// The undisplaced windows carry all-zero entries.
	for l := 1; l <= 2; l++ {
		require.True(t, cr.ExactLoop(l), "window loop %d is exact", l)
	}
	require.False(t, cr.ExactLoop(0))
}

func rectLoop(x0, y0, x1, y1 float64) sectionrecord.LoopRecord {
	return poly(sectionrecord.Point2{U: x0, V: y0}, sectionrecord.Point2{U: x1, V: y0},
		sectionrecord.Point2{U: x1, V: y1}, sectionrecord.Point2{U: x0, V: y1})
}

func rectPayload(t *testing.T, loop sectionrecord.LoopRecord, delta float64) prismrecord.Payload {
	t.Helper()
	p := discPayload(t, 1)
	p.Profile = momentinput.Profile{Outer: loop}
	if delta > 0 {
		cr := prismrecord.UniformCarrierRecord(p.Profile, delta)
		p.SectionDelta, p.Carriers = delta, &cr
	}
	return p
}

// A holed rectangular plate united with a block and its translation copies is
// no disc: the extended class builds it with a record, and the area is the
// plate's less its hole plus the blocks' parts above its top edge.
func TestUnionManyHoledPlateWithTranslationCopies(t *testing.T) {
	t.Parallel()
	plate := rectPayload(t, rectLoop(0, 0, 40, 30), 0)
	// a clockwise rectangular hole
	hole := poly(sectionrecord.Point2{U: 5, V: 5}, sectionrecord.Point2{U: 5, V: 10},
		sectionrecord.Point2{U: 10, V: 10}, sectionrecord.Point2{U: 10, V: 5})
	plate.Profile.Holes = []sectionrecord.LoopRecord{hole}
	ps := []prismrecord.Payload{plate}
	for i := range 3 {
		dy := 0.3 * float64(i)
		delta := 0.0
		if i > 0 {
			delta = 2e-13
		}
		ps = append(ps, rectPayload(t, rectLoop(4+12*float64(i), 25+dy, 12+12*float64(i), 35+dy), delta))
	}
	out, ok, err := unionBodies(t, ps...)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, out.Profile.Holes, 1)
	area := loopArea(t, out.Profile.Outer) + loopArea(t, out.Profile.Holes[0])
	require.InDelta(t, 1200-25+8*(15+0.9), area, 1e-9)
	cr, valid, err := out.CarrierRecord(proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, valid)
	require.True(t, cr.ExactLoop(1), "the undisplaced plate's hole is exact")
	require.GreaterOrEqual(t, out.SectionDelta, 2e-13)
}

// ownsRecord reports that the operand's own body carries a valid carrier
// record (U3), so a union miss is not a miss at admission.
func ownsRecord(t *testing.T, p prismrecord.Payload) bool {
	t.Helper()
	body, err := prismrecord.BuildWithRecord(t.Context(), testEnv(), p, *p.Carriers)
	require.NoError(t, err)
	_, valid, err := prismrecord.CarrierRecordOf(proofbound.NewWorkBudget(t.Context()), body)
	require.NoError(t, err)
	return valid
}

// V1 (§5.3): an operand's own recorded vertex that lies on another operand's
// circle meets that circle's entity there, with no cut parameter the
// arrangement computed on the operand's own two segments, so nothing charges
// it and the set misses.
//
// The triangle is counter-clockwise (a clockwise one is refused when it is
// built, so the union would miss at U3), has its vertex on the rim of the disc
// at angle 0.5 and one flank leaving outward and one inward (a triangle wholly
// outside the rim leaves a cell the classification cannot decide, a miss at
// ClassifyRegions before V1). Sketch snaps the vertex onto the circle: the
// float point polar(20, 0.5) is not exactly on it. The same triangle with its
// vertex moved 1 mm in from the rim crosses the rim transversally and builds,
// so the miss belongs to the vertex. The guard-disabled demonstration of V1
// is prismcells' TestReadVerticesOwnVertexOnCircle (the union with V1 off
// still misses, one step later, where the arrangement audit cannot read the
// uncharged vertex).
func TestUnionManyMissesAtAVertexOnTheCircle(t *testing.T) {
	t.Parallel()
	triangle := func(vertexRadius float64) prismrecord.Payload {
		return rectPayload(t, poly(polar(vertexRadius, 0.5), polar(24, 0.7), polar(16, 0.3)), 1e-13)
	}
	on := triangle(20)
	require.True(t, ownsRecord(t, on), "the operand owns a valid record, so the union is not refused at U3")
	_, ok, err := unionBodies(t, discPayload(t, 20), on)
	require.NoError(t, err)
	require.False(t, ok)

	off := triangle(19)
	require.True(t, ownsRecord(t, off))
	out, ok, err := unionBodies(t, discPayload(t, 20), off)
	require.NoError(t, err)
	require.True(t, ok, "the vertex 1 mm inside the rim is a transverse crossing")
	require.Greater(t, out.SectionDelta, 0.0)
}

// A displaced block whose left side lies within the contact floor of an
// undisplaced block's right side misses; the same set with the sides a few
// floors apart builds. Sketch merges the two near-coincident carriers first
// (U5b), so this pins the miss only: V2 is shown by
// TestUnionManyMissesAShallowCrossingNearAnOwnVertex.
func TestUnionManyMissesNearCoincidentCopyLine(t *testing.T) {
	t.Parallel()
	a := rectPayload(t, rectLoop(0, 0, 10000, 10000), 0)
	near := rectPayload(t, rectLoop(10000-2e-5, 2000, 20000, 8000), 1e-13)
	_, ok, err := unionBodies(t, a, near)
	require.NoError(t, err)
	require.False(t, ok)

	apart := rectPayload(t, rectLoop(9000, 2000, 20000, 8000), 1e-13)
	_, ok, err = unionBodies(t, a, apart)
	require.NoError(t, err)
	require.True(t, ok)
}

// V2 (§6.3): a displaced block whose left side leans 1e-3 off the vertical
// crosses an undisplaced square's right side 5e-5 from the square's corner,
// inside the trim length 8F/s0 of 1e-4, so the stability audit refuses the
// union; the same block crossing 1 mm from the corner builds. With V2 switched
// off the near-corner set builds (the guard-disabled demonstration:
// prismcells' TestStabilityGuardIsV2 does the same at the audit's own call). The
// near-coincident sides of TestUnionManyMissesNearCoincidentCopyLine never
// reach V2: sketch merges them first (U5b), so that test pins the miss only.
func TestUnionManyMissesAShallowCrossingNearAnOwnVertex(t *testing.T) {
	t.Parallel()
	block := func(cross float64) prismrecord.Payload {
		x := func(y float64) float64 { return 10 + 1e-3*(y-cross) }
		return rectPayload(t, poly(sectionrecord.Point2{U: x(cross - 2), V: cross - 2},
			sectionrecord.Point2{U: 14, V: cross - 2}, sectionrecord.Point2{U: 14, V: cross + 2},
			sectionrecord.Point2{U: x(cross + 2), V: cross + 2}), 1e-13)
	}
	square := rectPayload(t, rectLoop(0, 0, 10, 10), 0)
	_, ok, err := unionBodies(t, square, block(10-5e-5))
	require.NoError(t, err)
	require.False(t, ok)
	out, ok, err := unionBodies(t, square, block(9))
	require.NoError(t, err)
	require.True(t, ok)
	require.Greater(t, out.SectionDelta, 0.0)
}
