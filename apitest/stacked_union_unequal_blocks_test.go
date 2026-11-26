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
		body, err = decad.Union(body, boxBodyAtZ(t, doc, b.x0, b.y0, b.x1, b.y1, 10, b.h)).Do(t.Context())
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
	got, err := decad.Fillet(body, sel.Exactly(4), units.Millimeters(1)).Do(t.Context())
	require.NoError(t, err)
	requireEveryEdgeOnTwoFaces(t, got)
	before, err := body.Volume()
	require.NoError(t, err)
	after, err := got.Volume()
	require.NoError(t, err)
	want := 60*(1-math.Pi/4) + 4*(math.Pi/2)*(5.0/6-math.Pi/4)
	require.InDelta(t, want, after.Value.Base()-before.Value.Base(), 1e-9)
}

// TestStackedUnionBlocksApartTakeMeshPath joins two blocks that stand apart
// on no common plate, a 10 mm cube and a block over z 5 to 15 beside it. No
// interface cell lies in both, so the brep build does not read their
// footprints as apart and the union keeps the mesh path, two lumps.
func TestStackedUnionBlocksApartTakeMeshPath(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	got, err := decad.Union(boxBody(t, doc, 0, 0, 10, 10, 10), boxBodyAtZ(t, doc, 12, 3, 15, 7, 5, 10)).Do(t.Context())
	require.NoError(t, err)
	require.True(t, tookMeshPath(got))
	require.Len(t, got.Lumps(), 2)
}

// joinOrRefuse is the union of a and b, or nil with the refusal when the
// union refuses.
func joinOrRefuse(t *testing.T, a, b *decad.Body) (*decad.Body, error) {
	t.Helper()
	got, err := decad.Union(a, b).Do(t.Context())
	if err != nil {
		require.ErrorIs(t, err, decad.ErrUnsupported)
		return nil, err
	}
	return got, nil
}

// TestStackedUnionApartGroupsMustNotTouch puts a block across the tops of two
// 15 mm blocks standing on a plate. Over z = 25 the bridge's footprint and
// one block's top share no edge. Where they stand apart the union is an exact
// brep; where they touch at a corner, or where a bridge's edge is tangent to
// a round boss's top circle, reading them apart would pinch the result at
// that point, so the union keeps the mesh path or refuses.
func TestStackedUnionApartGroupsMustNotTouch(t *testing.T) {
	t.Parallel()
	bridged := func(t *testing.T, doc *decad.Document, x0, y0 float64) *decad.Body {
		t.Helper()
		body := unionUnequalBlocks(t, doc, []unequalBlock{{10, 10, 20, 20, 15}, {x0, y0, x0 + 10, y0 + 10, 15}})
		got, err := decad.Union(body, boxBodyAtZ(t, doc, 15, 15, 30, 25, 25, 5)).Do(t.Context())
		require.NoError(t, err)
		return got
	}
	t.Run("apart", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		got := bridged(t, doc, 35, 25)
		require.False(t, tookMeshPath(got))
		requireEveryEdgeOnTwoFaces(t, got)
		volume, err := got.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, volume.Exactness)
		require.Equal(t, 24000.0+1500+1500+750, volumeMM(t, volume))
		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		reading, err := report.ForBody(got)
		require.NoError(t, err)
		require.Equal(t, decad.Sound, reading.Status)
	})
	t.Run("corner touch", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		// The bridge's corner (30, 25, 25) is the second block's top corner.
		require.True(t, tookMeshPath(bridged(t, doc, 30, 25)))
	})
	for _, tc := range []struct {
		name string
		x1   float64
	}{{"edge tangent to the boss", 30}, {"clear of the boss", 29}} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := boxBody(t, doc, 0, -10, 60, 10, 10)
			body, err := decad.Union(plate, circleBodyAtZ(t, doc, 15, 5, 10, 15)).Do(t.Context())
			require.NoError(t, err)
			body, err = decad.Union(body, circleBodyAtZ(t, doc, 35, 5, 10, 15)).Do(t.Context())
			require.NoError(t, err)
			// The bridge's edge x = 30 touches the second boss's top circle at (30, 0, 25).
			got, err := joinOrRefuse(t, body, boxBodyAtZ(t, doc, 15, -3, tc.x1, 3, 25, 5))
			if tc.x1 == 30 {
				require.True(t, err != nil || tookMeshPath(got), "a tangent contact is not read apart")
				return
			}
			require.NoError(t, err)
			require.False(t, tookMeshPath(got))
			requireEveryEdgeOnTwoFaces(t, got)
			volume, err := got.Volume()
			require.NoError(t, err)
			want := 12000 + 2*25*15*math.Pi + (tc.x1-15)*6*5
			require.InDelta(t, want, volume.Value.Base(), volume.Bound.Base()+1e-9)
		})
	}
}

// TestStackedUnionApartFloatingPiece joins a plate with one 15 mm block and,
// as the other operand, two blocks over z 20 to 30: one stands in the tower's
// top, the other floats clear of everything. The operands meet, so the apart
// reading is tried, and the floating block is a second lump of an exact brep.
// The result is correct and accepted, in either operand order.
func TestStackedUnionApartFloatingPiece(t *testing.T) {
	t.Parallel()
	for _, swap := range []bool{false, true} {
		t.Run(map[bool]string{false: "plate first", true: "blocks first"}[swap], func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			body := unionUnequalBlocks(t, doc, []unequalBlock{{10, 10, 20, 20, 15}})
			blocks, err := decad.Union(boxBodyAtZ(t, doc, 12, 12, 18, 18, 20, 10), boxBodyAtZ(t, doc, 40, 10, 50, 20, 20, 10)).Do(t.Context())
			require.NoError(t, err)
			a, b := body, blocks
			if swap {
				a, b = blocks, body
			}
			got, err := decad.Union(a, b).Do(t.Context())
			require.NoError(t, err)
			require.False(t, tookMeshPath(got))
			require.Len(t, got.Lumps(), 2)
			requireEdgesOnTwoFacesAnyLumps(t, got)
			volume, err := got.Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Exact, volume.Exactness)
			// Plate 24000, tower 1500, the inset block's 6 × 6 × 5 above the tower, the floating block 1000.
			require.Equal(t, 24000.0+1500+36*5+1000, volumeMM(t, volume))
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			reading, err := report.ForBody(got)
			require.NoError(t, err)
			require.Equal(t, decad.Sound, reading.Status)
		})
	}
}

// requireEdgesOnTwoFacesAnyLumps is requireEveryEdgeOnTwoFaces for a body of
// any number of lumps.
func requireEdgesOnTwoFacesAnyLumps(t *testing.T, body *decad.Body) {
	t.Helper()
	for _, e := range body.Edges() {
		require.Equal(t, 2, len(e.Faces()))
	}
}
