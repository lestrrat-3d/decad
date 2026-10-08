package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These tests pin docs/clearance-design.md §5's cell pruning: a cell is
// skipped only when its box distance, less the margin, lies STRICTLY above
// the best upper bound in hand, and the pruned walk proves the same minimum
// as a walk that runs every cell.

// pruneRod extrudes a radius-r circle at (cx, cy) 20 mm along +Z: two cap
// planes, one cylinder wall and two rim circles.
func pruneRod(t *testing.T, doc *Document, cx, cy, r float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(cx, cy)
	s.Fix(center)
	s.CreateCircle(center, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(20), Dir: Along})
	require.NoError(t, err)
	return body
}

func TestCellSinkPrunesOnlyStrictlyBeyondTheMargin(t *testing.T) {
	t.Parallel()
	withBest := func(margin float64) *cellSink {
		s := newPruningSink(margin)
		s.Contribs = append(s.Contribs,
			clearance.GapContrib{Lo: 4, Hi: math.Inf(1)},
			clearance.GapContrib{Lo: 5, Hi: 5, Exact: true},
			clearance.GapContrib{Lo: 6, Hi: 7},
		)
		return s
	}
	t.Run("equality never prunes", func(t *testing.T) {
		s := withBest(0)
		require.False(t, s.Pruned(5))
		require.True(t, s.Pruned(math.Nextafter(5, 6)))
		require.Equal(t, 1, s.Skipped)
	})
	t.Run("the margin is charged before comparing", func(t *testing.T) {
		s := withBest(0.5)
		require.False(t, s.Pruned(5.5))
		require.True(t, s.Pruned(5.75))
	})
	t.Run("a non-finite distance never prunes", func(t *testing.T) {
		s := withBest(0)
		require.False(t, s.Pruned(math.Inf(1)))
		require.False(t, s.Pruned(math.NaN()))
		require.Zero(t, s.Skipped)
	})
	t.Run("no finite upper bound prunes nothing", func(t *testing.T) {
		s := newPruningSink(0)
		s.Contribs = append(s.Contribs, clearance.GapContrib{Lo: 4, Hi: math.Inf(1)})
		require.False(t, s.Pruned(1e300))
	})
	t.Run("a zero-value sink never prunes", func(t *testing.T) {
		s := &cellSink{Contribs: []clearance.GapContrib{{Lo: 5, Hi: 5, Exact: true}}}
		require.False(t, s.Pruned(100))
	})
}

// TestEnumeratePrunedRodsMatchTheFullWalk runs two parallel rods of radius 2
// whose axes sit 10 mm apart along (6, 8): the true gap is exactly 6. Every
// cell pairing a rim or cap at z = 0 with one at z = 20 lies at least 20 mm
// apart by box and is pruned. The pruned walk must keep the full walk's least
// upper bound, never lower its least lower bound, and enclose the truth.
func TestEnumeratePrunedRodsMatchTheFullWalk(t *testing.T) {
	t.Parallel()
	doc := New()
	a := pruneRod(t, doc, 0, 0, 2)
	b := pruneRod(t, doc, 6, 8, 2)
	ga, ok := newBodyGeom(a)
	require.True(t, ok)
	gb, ok := newBodyGeom(b)
	require.True(t, ok)
	k := newPairKernel(t.Context(), ga, gb)

	pruned, err := k.enumerate()
	require.NoError(t, err)
	full, err := k.enumerateInto(&cellSink{})
	require.NoError(t, err)

	// The sorted face/edge walk prunes exactly the cells whose box distance
	// less the margin exceeds the full walk's best hi. The vertex tiers run
	// earlier, against the bound in hand then, so each of their cells beyond
	// it may or may not be pruned.
	_, fullHi, _, ok := full.Interval()
	require.True(t, ok)
	cells, err := clearance.FeatureCells(proofbound.NewWorkBudget(t.Context()), ga.faces, gb.faces, ga.edges, gb.edges)
	require.NoError(t, err)
	beyond := 0
	for _, c := range cells {
		if c.LowerBound-k.slack > fullHi {
			beyond++
		}
	}
	vertexBeyond := 0
	for _, side := range [][2]*bodyGeom{{ga, gb}, {gb, ga}} {
		for _, v := range side[0].verts {
			at := [2]r3.Vec{v, v}
			for _, f := range side[1].faces {
				if clearance.ClrBoxDist(at, f.Box)-k.slack > fullHi {
					vertexBeyond++
				}
			}
			for _, e := range side[1].edges {
				if clearance.ClrBoxDist(at, e.Box)-k.slack > fullHi {
					vertexBeyond++
				}
			}
		}
	}
	require.Positive(t, beyond, `far rim and cap cells must be prunable`)
	require.GreaterOrEqual(t, pruned.Skipped, beyond)
	require.LessOrEqual(t, pruned.Skipped, beyond+vertexBeyond)
	require.Zero(t, full.Skipped)
	require.Less(t, len(pruned.Contribs), len(full.Contribs))
	require.Equal(t, full.Overlap, pruned.Overlap)
	require.Equal(t, full.Unsure, pruned.Unsure)
	require.False(t, pruned.Overlap)
	require.False(t, pruned.Unsure)

	lo, hi, _, ok := pruned.Interval()
	require.True(t, ok)
	fullLo, _, _, ok := full.Interval()
	require.True(t, ok)
	require.Equal(t, fullHi, hi)
	require.GreaterOrEqual(t, lo, fullLo)
	require.LessOrEqual(t, lo, 6.0)
	require.GreaterOrEqual(t, hi, 6.0)

	res, err := clearancePair(t.Context(), a, b, false)
	require.NoError(t, err)
	require.Equal(t, pairDisjoint, res.verdict)
	require.LessOrEqual(t, res.lo, 6.0)
	require.GreaterOrEqual(t, res.hi, 6.0)
}

// TestEnumerateNeverPrunesAtTheBestUpperBound builds two perpendicular
// segments whose nearest points are their own endpoints, 5 apart. The
// vertex × vertex distance is the first upper bound, exactly 5, and every
// later cell's box distance is exactly 5 too. With no margin, those cells sit
// AT the best upper bound, not above it, so none may be pruned.
func TestEnumerateNeverPrunesAtTheBestUpperBound(t *testing.T) {
	t.Parallel()
	seg := func(p, q r3.Vec) *clearance.CEdge {
		return &clearance.CEdge{Line: true, A: p, B: q, Box: clearance.BoxOf(p, q)}
	}
	origin, foot := r3.NewVec(0, 0, 0), r3.NewVec(0, 5, 0)
	k := &pairKernel{
		a:   &bodyGeom{verts: []r3.Vec{origin}, edges: []*clearance.CEdge{seg(origin, r3.NewVec(10, 0, 0))}},
		b:   &bodyGeom{verts: []r3.Vec{foot}, edges: []*clearance.CEdge{seg(foot, r3.NewVec(0, 5, 10))}},
		ctx: t.Context(),
		tol: 1e-9,
	}

	pruned, err := k.enumerate()
	require.NoError(t, err)
	full, err := k.enumerateInto(&cellSink{})
	require.NoError(t, err)

	require.Zero(t, pruned.Skipped)
	require.Len(t, full.Contribs, 4)
	require.Equal(t, full.Contribs, pruned.Contribs)
	lo, hi, exact, ok := pruned.Interval()
	require.True(t, ok)
	require.Equal(t, 5.0, lo)
	require.Equal(t, 5.0, hi)
	require.True(t, exact)
}
