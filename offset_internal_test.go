package decad

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestOffsetPrismUnrepresentableOffset is docs/surface-design.md §15's T207:
// the exact-generation gate refuses an axis-parallel edge whose offset
// coordinate no float represents (R39). The public sketch seam rejects this
// very large translated rectangle before Offset can read it, so the gate is
// driven directly, exactly as thicken_axis_test.go drives its own.
//
// The first assertion is what makes the refusal meaningful: the generated
// coordinate comes back EQUAL to the source coordinate, one whole millimetre
// from the offset the call denotes, while sitting one part in 2^53 away from
// it in relative terms. Only the exact rational comparison
// (rationalFloatError, through thickenPointIsExact) can see that; every
// residual test scaled to the coordinate admits it.
//
// Shown to fail: replace thickenPointIsExact's exact comparison with a
// relative-residual test and both refusals below become successful builds — the
// admitted body publishes an Exact 3160 mm² area at a zero bound over a section
// whose U coordinates never moved at all.
func TestOffsetPrismUnrepresentableOffset(t *testing.T) {
	t.Parallel()
	base := math.Ldexp(1, 53)
	points := []Point2{{U: base, V: 0}, {U: base + 100, V: 0},
		{U: base + 100, V: 60}, {U: base, V: 60}}
	segments := make([]CurveSegment, len(points))
	for i := range points {
		segments[i] = LineSeg{Start: points[i], End: points[(i+1)%len(points)], TStart: 0, TEnd: 1}
	}
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	pp := prismPayload{
		profile: ProfileRecord{Outer: LoopRecord{Segments: segments}},
		frame:   frame, xform: r3.Identity(),
		z0: 0, z1: 10, surfaceResult: true,
	}

	budget := newWorkBudget(t.Context())
	generated, offErr := offsetProfile(budget, pp.profile, +1, 1)
	require.NoError(t, offErr)
	first, ok := generated.Outer.Segments[0].(LineSeg)
	require.True(t, ok)
	require.Equal(t, base, first.Start.U,
		`the generated coordinate rounds back onto the source's, a whole millimetre from the offset it denotes`)

	doc := New()
	result, err := offsetPrism(t.Context(), doc, pp, OffsetNegative, 1, 0)
	require.Nil(t, result)
	require.ErrorIs(t, err, ErrUnsupported)
	require.True(t, strings.Contains(err.Error(), "rounded"), err.Error())
	require.Empty(t, doc.Bodies())

	// The same gate on the growing side, so the refusal is the coordinate's
	// doing rather than one sense's.
	result, err = offsetPrism(t.Context(), doc, pp, OffsetPositive, 1, 0)
	require.Nil(t, result)
	require.ErrorIs(t, err, ErrUnsupported)
	require.True(t, strings.Contains(err.Error(), "rounded"), err.Error())
	require.Empty(t, doc.Bodies())
}

// TestOffsetPrismIntervalCertificationRefusesThePinch pins the whole-interval
// certification on this path: the two walls of §17.2's 4 mm neck meet at
// tau = 2, and thickenAxisIntervalClear refuses every erosion that reaches
// that contact while admitting every one that stops short of it.
//
// On the axis-parallel class this build admits, the certification is a BACKSTOP
// rather than the first gate. Every contact event here is monotone in tau — two
// facing offset walls close at rate 2·tau and two corner circles' radii sum to
// 2·tau — so a contact that begins inside the interval persists to the endpoint,
// where the shared section audit reports it first and more specifically. Remove
// that audit and this certification is what refuses the same neck, with the
// less specific message §16.2's gate order exists to avoid.
func TestOffsetPrismIntervalCertificationRefusesThePinch(t *testing.T) {
	t.Parallel()
	pp := offsetNeckPayload(t)
	budget := newWorkBudget(t.Context())
	loops, err := prismCornerLoopsBudget(budget, pp)
	require.NoError(t, err)
	require.Len(t, loops, 1)
	dirs, err := thickenAxisDirections(loops[0], budget)
	require.NoError(t, err)

	err = thickenAxisIntervalClear(t.Context(), loops[0], dirs, +1, 2.5, budget, nil)
	require.ErrorIs(t, err, ErrUnsupported)
	require.True(t, strings.Contains(err.Error(), "offset interval"), err.Error())

	doc := New()
	result, err := offsetPrism(t.Context(), doc, pp, OffsetNegative, 2.5, 0)
	require.Nil(t, result)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Empty(t, doc.Bodies())

	// A 1.5 mm erosion stops short of the contact: the certification admits it
	// and the section builds, so the refusal above is the pinch's doing and not
	// the shape's.
	require.NoError(t, thickenAxisIntervalClear(t.Context(), loops[0], dirs, +1, 1.5, budget, nil))
	result, err = offsetPrism(t.Context(), doc, pp, OffsetNegative, 1.5, 0)
	require.NoError(t, err)
	require.NotNil(t, result)
}

// TestOffsetPrismResultIsBoundedByTheOffsetLoopAlone is the premise §17.2 drops
// Thicken's source-and-offset nesting audit on: the result holds ONE region,
// bounded by the offset loop alone, and denotes nothing whatever about the
// source section. The committed payload is what states that — its Outer is the
// generated loop bit for bit and it carries no hole, so there is no outer/inner
// pair for a nesting relation to hold between and none is published. Thicken
// needs that audit because its own payload DOES carry the pair.
//
// Shown to fail: assemble the result as an annulus instead — Outer the source
// loop and the reversed offset as its hole, thickenAnnulus's own shape — and
// both assertions below go red, which is the only payload shape the dropped
// audit would have a subject in.
func TestOffsetPrismResultIsBoundedByTheOffsetLoopAlone(t *testing.T) {
	t.Parallel()
	pp := offsetNeckPayload(t)
	budget := newWorkBudget(t.Context())
	section, err := offsetPrismSection(t.Context(), pp, +1, 1.5, budget)
	require.NoError(t, err)

	doc := New()
	result, err := offsetPrism(t.Context(), doc, pp, OffsetNegative, 1.5, 0)
	require.NoError(t, err)
	built, ok := result.payload.(prismPayload)
	require.True(t, ok)
	require.Empty(t, built.profile.Holes,
		`one region: no hole walk, so the payload publishes no nesting relation to audit`)
	require.Equal(t, section.Outer, built.profile.Outer,
		`the result's boundary IS the generated offset loop, and the source loop appears nowhere in it`)
}

// offsetNeckPayload is §17.2's 4 mm-wide neck as a surface-result prism
// payload, the same ordered vertices docs/surface-design.md §15 states. It
// carries the XY frame and the identity placement so the admitted leg really
// builds a body rather than refusing on a zero-value frame.
func offsetNeckPayload(t *testing.T) prismPayload {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	pts := [][2]float64{
		{0, 0}, {10, 0}, {10, 8}, {20, 8}, {20, 0}, {30, 0},
		{30, 20}, {20, 20}, {20, 12}, {10, 12}, {10, 20}, {0, 20},
	}
	segments := make([]CurveSegment, len(pts))
	for i := range pts {
		next := pts[(i+1)%len(pts)]
		segments[i] = LineSeg{
			Start:  Point2{U: pts[i][0], V: pts[i][1]},
			End:    Point2{U: next[0], V: next[1]},
			TStart: 0, TEnd: 1,
		}
	}
	return prismPayload{
		profile: ProfileRecord{Outer: LoopRecord{Segments: segments}},
		frame:   frame, xform: r3.Identity(),
		z0: 0, z1: 10, surfaceResult: true,
	}
}
