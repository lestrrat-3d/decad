package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin general-boolean §3 "A1 as a brep" for blocks of unequal
// heights standing apart on a 60 × 40 × 10 plate. Above the lower block's top
// the taller block's footprint and the lower block's top lie in one interface
// scene as two rectangles that share no edge; each one's membership in the
// other's record is read as outside, so the union stays an exact brep.

// unequalBlock is a block standing on the plate's top at z = 10: its
// footprint and height.
type unequalBlock struct {
	x0, y0, x1, y1, h float64
}

func (b unequalBlock) volume() float64 { return (b.x1 - b.x0) * (b.y1 - b.y0) * b.h }

// unionUnequalBlocks joins the plate and the blocks two at a time, in order.
func unionUnequalBlocks(t *testing.T, doc *decad.Document, blocks []unequalBlock) *decad.Body {
	t.Helper()
	body := boxBody(t, doc, 0, 0, 60, 40, 10)
	for _, b := range blocks {
		var err error
		body, err = decad.Union(t.Context(), body, boxBodyAtZ(t, doc, b.x0, b.y0, b.x1, b.y1, 10, b.h))
		require.NoError(t, err)
	}
	return body
}

// requireExactUnion checks the union is an exact, sound brep holding the
// plate and every block.
func requireExactUnion(t *testing.T, doc *decad.Document, got *decad.Body, blocks []unequalBlock) {
	t.Helper()
	require.False(t, tookMeshPath(got))
	requireEveryEdgeOnTwoFaces(t, got)
	require.Equal(t, 6+5*len(blocks), len(got.Faces()), "the plate's six faces and five per block")
	want := 60.0 * 40 * 10
	for _, b := range blocks {
		want += b.volume()
	}
	volume, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.Equal(t, want, volumeMM(t, volume))
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	reading, err := report.ForBody(got)
	require.NoError(t, err)
	require.Equal(t, decad.Sound, reading.Status)
}

func TestStackedUnionUnequalBlocks(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		blocks []unequalBlock
	}{
		{"taller first", []unequalBlock{{10, 10, 20, 20, 15}, {40, 10, 50, 30, 10}}},
		{"lower first", []unequalBlock{{40, 10, 50, 30, 10}, {10, 10, 20, 20, 15}}},
		{"three heights", []unequalBlock{{5, 5, 15, 15, 5}, {25, 5, 35, 35, 12}, {45, 20, 55, 35, 8}}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			got := unionUnequalBlocks(t, doc, tc.blocks)
			requireExactUnion(t, doc, got, tc.blocks)
		})
	}
}

// TestStackedUnionUnequalBlocksRootFillet rounds the root loop of the lower
// block on the unequal-height union at r = 1: a complete loop of the plate's
// top, which route L fillets on the brep. The four quarter cylinders add
// (1 − π/4)·r² per millimetre over the 60 mm loop and the four horn-torus
// corners (π/2)·r³·(5/6 − π/4) each.
func TestStackedUnionUnequalBlocksRootFillet(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	blocks := []unequalBlock{{10, 10, 20, 20, 15}, {40, 10, 50, 30, 10}}
	body := unionUnequalBlocks(t, doc, blocks)
	require.False(t, tookMeshPath(body))
	var sel *decad.EdgeQuery
	for _, e := range []struct{ at, dir r3.Vec }{
		{r3.NewVec(40, 10, 10), r3.NewVec(1, 0, 0)}, {r3.NewVec(50, 10, 10), r3.NewVec(0, 1, 0)},
		{r3.NewVec(50, 30, 10), r3.NewVec(1, 0, 0)}, {r3.NewVec(40, 30, 10), r3.NewVec(0, 1, 0)},
	} {
		preds := []decad.EdgePredicate{decad.EndpointAt(e.at), decad.ParallelTo(e.dir)}
		if sel == nil {
			sel = decad.Edges(preds...)
		} else {
			sel = sel.Or(preds...)
		}
	}
	got, err := decad.Fillet(t.Context(), body, sel.Exactly(4), units.Millimeters(1))
	require.NoError(t, err)
	requireEveryEdgeOnTwoFaces(t, got)
	before, err := body.Volume()
	require.NoError(t, err)
	after, err := got.Volume()
	require.NoError(t, err)
	want := 60*(1-math.Pi/4) + 4*(math.Pi/2)*(5.0/6-math.Pi/4)
	require.InDelta(t, want, after.Value.Base()-before.Value.Base(), 1e-9)
}
