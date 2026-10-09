package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// chamferedFlange builds the drilled filleted flange: a 96×68×16 plate centred
// at the origin, an analytic Cut of an r18 bore, a 12 mm Fillet of its vertical
// edges and a 1 mm cap-loop Chamfer on its end cap. Its outline is four lines
// joined to four fillet arcs, every corner exactly tangent, and its bore is one
// whole clockwise circle.
func chamferedFlange(t *testing.T) (*Body, capBlendPayload) {
	t.Helper()
	w := sketch.NewWorld()
	doc := New()

	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-48, -34, 48, 34)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	plate, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(16), Dir: Along})
	require.NoError(t, err)

	bs, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := bs.CreatePoint(0, 0)
	bs.Fix(o)
	bs.CreateCircle(o, 18)
	_, err = bs.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, bs.Profiles(), 1)
	bore, err := doc.Extrude(bs, bs.Profiles()[0], Symmetric{D: units.Millimeters(32)})
	require.NoError(t, err)

	drilled, err := Cut(t.Context(), plate, bore)
	require.NoError(t, err)
	filleted, err := drilled.Fillet(t.Context(), Edges(ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(12))
	require.NoError(t, err)
	chamfered, err := filleted.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(filleted))), units.Millimeters(1))
	require.NoError(t, err)
	cbp, ok := chamfered.payload.(capBlendPayload)
	require.True(t, ok)
	return chamfered, cbp
}

// TestCapBlendAdmissionAdmitsTheFlange is the predicate's admitting case: every
// one of the outline's eight corners is an exactly tangent line-arc join over
// the rationals, and the bore is the one cornerless whole turn.
func TestCapBlendAdmissionAdmitsTheFlange(t *testing.T) {
	t.Parallel()
	_, cbp := chamferedFlange(t)
	refusal, err := capBlendOccupiedVolumeAdmission(proofbound.NewWorkBudget(t.Context()), cbp)
	require.NoError(t, err)
	require.NoError(t, refusal, `every band of the flange is a whole turn or joins only exactly tangent corners`)

	loops := cbp.loops()
	require.Len(t, loops, 2)
	budget := proofbound.NewWorkBudget(t.Context())
	outer, err := oneLoopCornerLoop(budget, loops[0], freeform.NewFreeformWork())
	require.NoError(t, err)
	n := len(outer.walks)
	require.Equal(t, 8, n)
	for i := range n {
		prev, cur := outer.walks[(i+n-1)%n], outer.walks[i]
		prevSeg := loops[0].Segments[prev.Segs[len(prev.Segs)-1]]
		curSeg := loops[0].Segments[cur.Segs[0]]
		require.True(t, capJoinIsG1(prevSeg, curSeg), `corner %d is an exactly tangent join`, i)
	}

	bore, err := oneLoopCornerLoop(budget, loops[1], freeform.NewFreeformWork())
	require.NoError(t, err)
	require.Len(t, bore.walks, 1)
	require.True(t, bore.walks[0].Closed, `the bore takes the whole-turn branch`)
}

// TestCapBlendAdmissionRefusesAMiteredArc is the predicate's refusing case: a
// quarter disk's arc meets each radius at a genuine miter, where the true foot
// locus is a conic the slice argument does not follow.
func TestCapBlendAdmissionRefusesAMiteredArc(t *testing.T) {
	t.Parallel()
	_, cbp := chamferedSectionBody(t, quarterDiskSection(10), 2)
	refusal, err := capBlendOccupiedVolumeAdmission(proofbound.NewWorkBudget(t.Context()), cbp)
	require.NoError(t, err)
	require.ErrorIs(t, refusal, ErrUnsupported)
	require.ErrorContains(t, refusal, `loop 0`)
	require.ErrorContains(t, refusal, `no proof of the volume`)
}

// TestCapBlendAdmissionRefusesAReflexCorner keeps the L section's reflex corner
// out of this proof: its apex fan's stations read no recorded window.
func TestCapBlendAdmissionRefusesAReflexCorner(t *testing.T) {
	t.Parallel()
	_, cbp := chamferedSectionBody(t, func(s *sketch.Sketch) {
		pts := []*sketch.Point{
			s.CreatePoint(0, 0), s.CreatePoint(40, 0), s.CreatePoint(40, 20),
			s.CreatePoint(20, 20), s.CreatePoint(20, 40), s.CreatePoint(0, 40),
		}
		for i := range pts {
			s.CreateLine(pts[i], pts[(i+1)%len(pts)])
		}
		s.Fix(pts[0])
	}, 3)
	refusal, err := capBlendOccupiedVolumeAdmission(proofbound.NewWorkBudget(t.Context()), cbp)
	require.NoError(t, err)
	require.ErrorIs(t, refusal, ErrUnsupported)
	require.ErrorContains(t, refusal, `reflex`)
	require.ErrorContains(t, refusal, `loop 0`)
	require.ErrorContains(t, refusal, `no proof of the volume`)
}

// TestCapJoinIsG1IsExact pins the tangency test as exact rational arithmetic:
// a line running into a quarter arc at its start is G1, and moving the arc's
// recorded end one ulp off its circle, rotating the line, or reversing it each
// refuses — none of which a tolerance would see.
func TestCapJoinIsG1IsExact(t *testing.T) {
	t.Parallel()
	line := lineSeg{Start: Point2{U: 0, V: -12}, End: Point2{U: 36, V: -12}, TStart: 0, TEnd: 1}
	arc := arcSeg{Center: Point2{U: 36, V: 0}, Start: Point2{U: 36, V: -12}, End: Point2{U: 48, V: 0}, TStart: 0, TEnd: 1}
	require.True(t, capJoinIsG1(line, arc), `the line leaves along the arc's own tangent`)
	require.Empty(t, capBlendSegmentRefusal(arc))

	t.Run("an arc end off its circle", func(t *testing.T) {
		off := arc
		off.End.U = math.Nextafter(off.End.U, math.Inf(1))
		require.NotEmpty(t, capBlendSegmentRefusal(off), `rule 4 sees a one-ulp departure`)
		require.Contains(t, capBlendSegmentRefusal(off), `not on the circle`)
	})

	t.Run("a rotated line", func(t *testing.T) {
		rotated := line
		rotated.Start.V = math.Nextafter(rotated.Start.V, math.Inf(-1))
		require.False(t, capJoinIsG1(rotated, arc), `a nonzero cross product is a miter, however small`)
	})

	t.Run("an antiparallel cusp", func(t *testing.T) {
		back := lineSeg{Start: Point2{U: 72, V: -12}, End: Point2{U: 36, V: -12}, TStart: 0, TEnd: 1}
		require.False(t, capJoinIsG1(back, arc), `a cusp has a zero cross product and a negative dot product`)
	})

	t.Run("a reversed walk", func(t *testing.T) {
		// Walked backwards, the arc runs clockwise from End to Start, and the
		// reversed line leaves Start's point along −x: the join at (36, −12)
		// is still tangent.
		revArc := arc
		revArc.TStart, revArc.TEnd = 1, 0
		revLine := line
		revLine.TStart, revLine.TEnd = 1, 0
		require.True(t, capJoinIsG1(revArc, revLine))
	})

	t.Run("a trimmed range", func(t *testing.T) {
		trimmed := line
		trimmed.TEnd = 0.5
		require.NotEmpty(t, capBlendSegmentRefusal(trimmed))
	})
}
