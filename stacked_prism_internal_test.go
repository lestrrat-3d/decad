package decad

import (
	"math/big"
	"testing"

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
	require.NoError(t, falsifyStackedPayload(t.Context(), base))
	other := internalBoxBody(t, doc, 1, 1, 2, 2, 10)
	otherHole, err := reverseLoopRecordContext(t.Context(), other.payload.(prismPayload).profile.Outer)
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
			require.ErrorIs(t, falsifyStackedPayload(t.Context(), sp), tc.want)
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
	require.NoError(t, falsifyStackedPayload(t.Context(), sp))
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
