package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/stackedrecord"

	"github.com/lestrrat-3d/decad/internal/offset2d"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func internalOffsetBox(t *testing.T, doc *Document, x0, y0, x1, y1, z float64, extent Extent) *Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], extent)
	require.NoError(t, err)
	return body
}

func internalPocket(t *testing.T) (*Document, *Body) {
	t.Helper()
	doc := New()
	plate := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	tool := internalOffsetBox(t, doc, 3, 3, 7, 7, 6, Distance{D: units.Millimeters(4), Dir: Along})
	pocket, err := Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	return doc, pocket
}

func cloneStackedForAudit(sp stackedPrismPayload) stackedPrismPayload {
	sp.slabs = append([]prismSlab(nil), sp.slabs...)
	sp.interfaces = append([]prismSlabInterface(nil), sp.interfaces...)
	for i := range sp.slabs {
		sp.slabs[i].regions = append([]ProfileRecord(nil), sp.slabs[i].regions...)
	}
	return sp
}

func TestStackedPayloadAuditRejectsBrokenRecords(t *testing.T) {
	doc, pocket := internalPocket(t)
	base := pocket.payload.(stackedPrismPayload)
	require.NoError(t, stackedrecord.Falsify(t.Context(), stackedRecordOf(base)))
	other := internalBoxBody(t, doc, 1, 1, 2, 2, 10)
	otherHole, err := offset2d.ReverseLoopRecordContext(t.Context(), other.payload.(prismPayload).profile.Outer)
	require.NoError(t, err)
	cases := []struct {
		name   string
		change func(*stackedPrismPayload)
		want   error
	}{
		{"one slab", func(sp *stackedPrismPayload) { sp.slabs = sp.slabs[:1]; sp.interfaces = nil }, ErrUnsupported},
		{"two regions", func(sp *stackedPrismPayload) {
			sp.slabs[0].regions = append(sp.slabs[0].regions, sp.slabs[0].regions[0])
		}, ErrUnsupported},
		{"empty interval", func(sp *stackedPrismPayload) { sp.slabs[0].z1 = sp.slabs[0].z0 }, ErrDegenerate},
		{"separated levels", func(sp *stackedPrismPayload) { sp.slabs[0].z1 = 5 }, ErrDegenerate},
		{"different outer", func(sp *stackedPrismPayload) {
			sp.slabs[1].regions[0].Outer = sp.slabs[1].regions[0].Holes[0]
		}, ErrUnsupported},
		{"opposed holes", func(sp *stackedPrismPayload) {
			sp.slabs[0].regions[0].Holes = []LoopRecord{otherHole}
		}, ErrUnsupported},
		{"wrong exposed patch", func(sp *stackedPrismPayload) {
			sp.interfaces[0].lowerExposed = nil
		}, ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			sp := cloneStackedForAudit(base)
			tc.change(&sp)
			require.ErrorIs(t, stackedrecord.Falsify(t.Context(), stackedRecordOf(sp)), tc.want)
		})
	}
}

func TestBlindStackedLevelChargesExactOffsetSum(t *testing.T) {
	doc := New()
	plate := internalBoxBody(t, doc, 0, 0, 10, 10, 1)
	tool := internalOffsetBox(t, doc, 3, 3, 7, 7, 0.1,
		Symmetric{D: units.Millimeters(0.3)})
	pocket, err := Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	sp := pocket.payload.(stackedPrismPayload)
	inner := new(big.Rat).Add(proofarith.FloatRat(0.1), proofarith.FloatRat(0.3))
	held, _ := inner.Float64()
	require.Equal(t, held, sp.slabs[0].z1)
	require.Positive(t, sp.slabs[0].z1Delta)
	require.Equal(t, proofarith.RationalFloatError(inner, held), sp.slabs[0].z1Delta)
	require.Equal(t, sp.slabs[0].z1Delta, sp.slabs[1].z0Delta)
}

func TestBlindStackedLevelCarriesToolConversion(t *testing.T) {
	doc := New()
	plate := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	tool := internalOffsetBox(t, doc, 3, 3, 7, 7, 10,
		Distance{D: units.Inches(0.2), Dir: Against})
	toolDelta := tool.payload.(prismPayload).z0Delta
	require.Positive(t, toolDelta)
	pocket, err := Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	sp := pocket.payload.(stackedPrismPayload)
	require.GreaterOrEqual(t, sp.slabs[0].z1Delta, toolDelta)
	volume, err := pocket.Volume()
	require.NoError(t, err)
	require.GreaterOrEqual(t, volume.Bound.Base(), 16*toolDelta)
}

func TestBlindStackedPlacedToolChargesSectionDisplacement(t *testing.T) {
	doc := New()
	plate := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	const shift = 1e9
	tool := internalBoxBody(t, doc, 3-shift, 3, 7-shift, 7, 4)
	move, err := r3.Translation(r3.NewVec(shift, 0, 0))
	require.NoError(t, err)
	tool, err = tool.Placed(t.Context(), move)
	require.NoError(t, err)
	pocket, err := Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	sp := pocket.payload.(stackedPrismPayload)
	require.Positive(t, sp.sectionDelta)
	volume, err := pocket.Volume()
	require.NoError(t, err)
	require.Equal(t, Approximate, volume.Exactness)
	require.NoError(t, stackedrecord.Falsify(t.Context(), stackedRecordOf(sp)))
}

func TestBlindStackedAdmissionLeavesOtherCutsToMesh(t *testing.T) {
	doc := New()
	plate := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	cases := []struct {
		name           string
		x0, y0, x1, y1 float64
		z, height      float64
	}{
		{"touches cap", 3, 3, 7, 7, 10, 2},
		{"enclosed void", 3, 3, 7, 7, 3, 4},
		{"crosses outside wall", 8, 8, 12, 12, 6, 5},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			tool := internalOffsetBox(t, doc, tc.x0, tc.y0, tc.x1, tc.y1, tc.z,
				Distance{D: units.Millimeters(tc.height), Dir: Along})
			_, admitted, err := tryBlindStackedCut(t.Context(), plate, tool)
			require.NoError(t, err)
			require.False(t, admitted)
		})
	}
	_, pocket := internalPocket(t)
	blindAgain := internalOffsetBox(t, pocket.doc, 1, 1, 2, 2, 6,
		Distance{D: units.Millimeters(4), Dir: Along})
	_, admitted, err := tryStackedThroughCut(t.Context(), pocket, blindAgain)
	require.NoError(t, err)
	require.False(t, admitted)
}

// holedCupRecord is a k = 1 cup's stacked record, hand-built from rectangles:
// a 100×60 outer region with a 20×20 hole, under a 90×50 cavity whose hole is
// 30×30. The lining audit compares records only, so the cavity need not be
// the exact offset.
func holedCupRecord(t *testing.T) cupPayload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	hole := func(u0, v0, u1, v1 float64) LoopRecord {
		l, err := offset2d.ReverseLoopRecordContext(t.Context(), rectangleRecord(u0, v0, u1, v1).Outer)
		require.NoError(t, err)
		return l
	}
	outer := ProfileRecord{Outer: rectangleRecord(0, 0, 100, 60).Outer, Holes: []LoopRecord{hole(40, 20, 60, 40)}}
	cavity := ProfileRecord{Outer: rectangleRecord(5, 5, 95, 55).Outer, Holes: []LoopRecord{hole(35, 15, 65, 45)}}
	cp, err := cupView{outer: outer, cavity: cavity, frame: frame, zOuter: 0, zCav: 5, zOpen: 20,
		thickness: 5, sense: Inward, xform: r3.Identity()}.payload(t.Context())
	require.NoError(t, err)
	return cp
}

// TestCupStackedRecord is modify-reach §9.1's cup migration: a cup with k
// holes is one floor slab under one wall slab of 1 + k bands, joined through
// the floor into exactly one lump, and the record audits and re-derives its
// own interface.
func TestCupStackedRecord(t *testing.T) {
	cp := holedCupRecord(t)
	require.Len(t, cp.stack.slabs, 2)
	require.Len(t, cp.stack.slabs[0].regions, 1)
	require.Len(t, cp.stack.slabs[1].regions, 2, `1 + k wall bands`)
	require.NoError(t, stackedrecord.Falsify(t.Context(), stackedRecordOf(cp.stack)))
	derived, err := stackedInterfaces(t.Context(), cp.stack.slabs, cp.stack.interfaces)
	require.NoError(t, err)
	rederived := cp.stack
	rederived.interfaces = derived
	require.NoError(t, stackedrecord.Falsify(t.Context(), stackedRecordOf(rederived)), `the derived spelling passes the same audit`)

	d := New()
	body, err := evalCupContext(t.Context(), d, d.nextProducerID(), cp)
	require.NoError(t, err)
	require.Len(t, body.Lumps(), 1)
	require.Len(t, body.Shells(), 1)
	for _, e := range body.Edges() {
		require.Len(t, e.Faces(), 2)
	}
	// A_P·h − A_Q·(h − t), every term an exact float here.
	volume, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, Exact, volume.Exactness)
	require.Equal(t, (100*60-20*20)*20.0-(90*50-30*30)*15.0, volume.Value.Base())
	view := cp.view()
	require.Equal(t, 0.0, view.zOuter)
	require.Equal(t, 5.0, view.zCav)
	require.Equal(t, 20.0, view.zOpen)
	require.Equal(t, cp.stack.slabs[0].regions[0], view.outer)
	require.Equal(t, cp.stack.interfaces[0].lowerExposed[0], view.cavity)
}

func TestStackedLiningAuditRejectsBrokenRecords(t *testing.T) {
	base := holedCupRecord(t).stack
	other, err := offset2d.ReverseLoopRecordContext(t.Context(), rectangleRecord(1, 1, 2, 2).Outer)
	require.NoError(t, err)
	cases := []struct {
		name   string
		change func(*stackedPrismPayload)
		want   error
	}{
		{"extra narrow region", func(sp *stackedPrismPayload) {
			sp.slabs[1].regions = append(sp.slabs[1].regions, sp.slabs[1].regions[1])
		}, ErrUnsupported},
		{"narrow outer differs", func(sp *stackedPrismPayload) {
			sp.slabs[1].regions[0].Outer = rectangleRecord(1, 1, 99, 59).Outer
		}, ErrUnsupported},
		{"lining misses its wide hole", func(sp *stackedPrismPayload) {
			sp.slabs[1].regions[1].Holes = []LoopRecord{other}
		}, ErrUnsupported},
		{"both sides several regions", func(sp *stackedPrismPayload) {
			sp.slabs[0].regions = append(sp.slabs[0].regions, sp.slabs[0].regions[0])
		}, ErrUnsupported},
		{"exposed patch misses a hole", func(sp *stackedPrismPayload) {
			sp.interfaces[0].lowerExposed = []ProfileRecord{{Outer: sp.interfaces[0].lowerExposed[0].Outer}}
		}, ErrDegenerate},
		{"exposed patch on the narrow side", func(sp *stackedPrismPayload) {
			sp.interfaces[0].upperExposed = sp.interfaces[0].lowerExposed
			sp.interfaces[0].lowerExposed = nil
		}, ErrDegenerate},
		{"exposed hole is not the lining's outer", func(sp *stackedPrismPayload) {
			sp.interfaces[0].lowerExposed = []ProfileRecord{{Outer: sp.interfaces[0].lowerExposed[0].Outer, Holes: []LoopRecord{other}}}
		}, ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			sp := cloneStackedForAudit(base)
			tc.change(&sp)
			require.ErrorIs(t, stackedrecord.Falsify(t.Context(), stackedRecordOf(sp)), tc.want)
		})
	}
}
