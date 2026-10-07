package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// patternedPrism extrudes, into doc, the rectangle [u0, u1]×[v0, v1] by 5,
// or a circle of radius r about (u0, v0) when r > 0.
func patternedPrism(t *testing.T, doc *Document, u0, v0, u1, v1, r float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	if r > 0 {
		c := s.CreatePoint(u0, v0)
		s.Fix(c)
		s.CreateCircle(c, r)
	} else {
		rect := s.CreateRectangle(u0, v0, u1, v1)
		s.Fix(rect.A)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(5), Dir: Along})
	require.NoError(t, err)
	return body
}

func patternedGroup(t *testing.T, b *Body) stackedPrismPayload {
	t.Helper()
	sp, ok := b.payload.(stackedPrismPayload)
	require.True(t, ok)
	require.True(t, sp.isGroup())
	return sp
}

// TestPatternedGroupDisplacement is §8's N-hole row read on the records: six
// Ø4 discs at integer 10 mm steps are a group with a displacement of exactly
// zero, and the plate they cut keeps it zero. The fixture is shown to fail
// with the group tool replaced by six PlacedCopy cuts: each placed tool
// re-expresses into the plate's frame, and the plate publishes a positive
// displacement.
func TestPatternedGroupDisplacement(t *testing.T) {
	t.Parallel()
	t.Run("group tool", func(t *testing.T) {
		t.Parallel()
		doc := New()
		plate := patternedPrism(t, doc, 0, -20, 60, 20, 0)
		disc := patternedPrism(t, doc, 5, 0, 0, 0, 2)
		tool, err := disc.Patterned(t.Context(), LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: units.Millimeters(10), Count: 6})
		require.NoError(t, err)
		sp := patternedGroup(t, tool)
		require.Len(t, sp.slabs[0].regions, 6)
		require.Zero(t, sp.sectionDelta)
		got, err := Cut(t.Context(), plate, tool)
		require.NoError(t, err)
		pp, ok := got.payload.(prismPayload)
		require.True(t, ok)
		require.Len(t, pp.profile.Holes, 6)
		require.Zero(t, pp.sectionDelta)
	})
	t.Run("six PlacedCopy cuts", func(t *testing.T) {
		t.Parallel()
		doc := New()
		part := patternedPrism(t, doc, 0, -20, 60, 20, 0)
		disc := patternedPrism(t, doc, 5, 0, 0, 0, 2)
		for i := range 6 {
			shift, err := r3.Translation(r3.NewVec(10*float64(i), 0, 0))
			require.NoError(t, err)
			inst, err := disc.PlacedCopy(t.Context(), shift)
			require.NoError(t, err)
			part, err = Cut(t.Context(), part, inst)
			require.NoError(t, err)
		}
		pp, ok := part.payload.(prismPayload)
		require.True(t, ok)
		require.Positive(t, pp.sectionDelta, "the contrast: placed tools charge their re-expression")
	})
}

// TestPatternedOverlapIsNotProvenDisjoint pins the premise of §8's
// overlapping row: the disjointness scene over two overlapping instances
// does not prove them disjoint, so Patterned reaches Union.
func TestPatternedOverlapIsNotProvenDisjoint(t *testing.T) {
	t.Parallel()
	a := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 15, V: 0}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 15, V: 0}, End: Point2{U: 15, V: 5}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 15, V: 5}, End: Point2{U: 0, V: 5}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 0, V: 5}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
	rp, err := resolvePattern(LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: units.Millimeters(10), Count: 2})
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	mv, err := rp.instanceMotion(frame, r3.Identity(), 1)
	require.NoError(t, err)
	budget := proofbound.NewWorkBudget(t.Context())
	b, charge, err := moveRegion(budget, a, mv)
	require.NoError(t, err)
	require.Zero(t, charge)
	disjoint, _, err := provePrismRegionsDisjoint(t.Context(), budget, []ProfileRecord{a, b})
	require.NoError(t, err)
	require.False(t, disjoint)
}

// TestPatternedSixPinsGroup pins the circular group's records: six regions,
// a positive displacement from the certified 60° turns, and every centre
// within it of (20 cos 60i°, 20 sin 60i°) — the same check
// TestPatternCopiesChargesTheMotion makes on PatternCopies' instances.
func TestPatternedSixPinsGroup(t *testing.T) {
	t.Parallel()
	doc := New()
	pin := patternedPrism(t, doc, 20, 0, 0, 0, 2)
	group, err := pin.Patterned(t.Context(), CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: 6})
	require.NoError(t, err)
	sp := patternedGroup(t, group)
	require.Len(t, sp.slabs[0].regions, 6)
	require.Positive(t, sp.sectionDelta)
	twin := patternedPrism(t, New(), 20, 0, 0, 0, 2)
	copies, err := twin.PatternCopies(t.Context(), CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: 6})
	require.NoError(t, err)
	for i, c := range copies {
		pp, ok := c.payload.(prismPayload)
		require.True(t, ok)
		require.Equal(t, pp.profile, sp.slabs[0].regions[i+1], "a copy and a group instance are the same record")
		require.LessOrEqual(t, pp.sectionDelta, sp.sectionDelta)
	}
}

// TestPatternedGroupRunsNoBoolean pins §6.1's rule 1 as a construction, not
// only a result: a Union chain over the same disjoint instances would also
// reach a group (general-boolean A5), but rule 1 builds the group in one
// step, so the document advances by exactly one producer identity. The Union
// fallback for overlapping instances reserves one for each instance and each
// intermediate Union.
//
// Shown to fail with rule 1 disabled: the disjoint pattern then takes the
// Union chain and advances by four.
func TestPatternedGroupRunsNoBoolean(t *testing.T) {
	t.Parallel()
	t.Run("disjoint", func(t *testing.T) {
		t.Parallel()
		doc := New()
		peg := patternedPrism(t, doc, 0, 0, 5, 5, 0)
		before := doc.nextProducerID()
		group, err := peg.Patterned(t.Context(), LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: units.Millimeters(10), Count: 3})
		require.NoError(t, err)
		require.Equal(t, before+1, doc.nextProducerID())
		require.Equal(t, before, group.originProducer())
	})
	t.Run("overlapping", func(t *testing.T) {
		t.Parallel()
		doc := New()
		disc := patternedPrism(t, doc, 2, 0, 0, 0, 3)
		before := doc.nextProducerID()
		joined, err := disc.Patterned(t.Context(), CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: 2})
		require.NoError(t, err)
		// One instance and one Union.
		require.Equal(t, before+2, doc.nextProducerID())
		require.Equal(t, before+1, joined.originProducer())
	})
}
