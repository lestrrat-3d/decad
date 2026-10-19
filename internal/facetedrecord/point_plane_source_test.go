package facetedrecord_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// lineSheet builds the chain sheet a one-line chain from (5, -30) to (5, 30)
// on the XY plane sweeps over z ∈ [-30, 30], as chain Extrude records it.
func lineSheet(t *testing.T) *topo.Body {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	line := sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 5, V: -30}, End: sectionrecord.Point2{U: 5, V: 30}, TStart: 0, TEnd: 1}
	env := topo.Env{Doc: &topo.Document{}, Ref: 1, Alloc: topotest.Alloc{}}
	body, err := chainrecord.Build(t.Context(), env, chainrecord.Payload{
		Chains: []sectionrecord.ChainRecord{{Segments: []sectionrecord.CurveSegment{line}}},
		Frame:  frame, Z0: -30, Z1: 30, Xform: r3.Identity(),
	}, freeform.NewFreeformWork())
	require.NoError(t, err)
	return body
}

func TestPointPlaneSheet(t *testing.T) {
	t.Parallel()

	t.Run("ExactLineSheet", func(t *testing.T) {
		t.Parallel()
		sheet := lineSheet(t)
		plane, ok := facetedrecord.PointPlaneSheet(sheet, r3.Vec{})
		require.True(t, ok)
		// The line's normal (a.v − b.v, b.u − a.u) = (−60, 0) has
		// n·(a − apex) = −300, so it is turned to face away from the apex.
		require.Zero(t, plane.Nx.Cmp(big.NewRat(60, 1)))
		require.Zero(t, plane.Ny.Sign())
		require.Zero(t, plane.D.Cmp(big.NewRat(300, 1)))
		require.Equal(t, sectionrecord.Point2{U: 5, V: -30}, plane.A)
		require.Equal(t, sectionrecord.Point2{U: 5, V: 30}, plane.B)
		require.Equal(t, [2]float64{-30, 30}, [2]float64{plane.Z0, plane.Z1})
		faces := topobuild.Faces(sheet)
		require.Len(t, faces, 1)
		surface, ok := faces[0].Surface.(surfacegeom.Plane)
		require.True(t, ok)
		require.Equal(t, surface, plane.Surface)
		require.Equal(t, faces[0].Origins, plane.Origins)
		require.Equal(t, faces[0].NormalBound, plane.NormalBound)
		require.Equal(t, surface.Frame.N().X > 0, plane.FrameTowardFar)
	})

	t.Run("ApexOnThePlaneRefuses", func(t *testing.T) {
		t.Parallel()
		_, ok := facetedrecord.PointPlaneSheet(lineSheet(t), r3.NewVec(5, 0, 0))
		require.False(t, ok)
	})

	t.Run("PlacedSheetRefuses", func(t *testing.T) {
		t.Parallel()
		sheet := lineSheet(t)
		cp, ok := sheet.Payload.(chainrecord.Payload)
		require.True(t, ok)
		cp.Xform, _ = r3.Translation(r3.NewVec(0, 0, 1))
		sheet.Payload = cp
		_, ok = facetedrecord.PointPlaneSheet(sheet, r3.Vec{})
		require.False(t, ok)
	})

	t.Run("DisplacedSectionRefuses", func(t *testing.T) {
		t.Parallel()
		sheet := lineSheet(t)
		cp, ok := sheet.Payload.(chainrecord.Payload)
		require.True(t, ok)
		cp.SectionDelta = 1e-9
		sheet.Payload = cp
		_, ok = facetedrecord.PointPlaneSheet(sheet, r3.Vec{})
		require.False(t, ok)
	})

	t.Run("SolidRefuses", func(t *testing.T) {
		t.Parallel()
		sheet := lineSheet(t)
		sheet.Kind = topo.BodySolid
		_, ok := facetedrecord.PointPlaneSheet(sheet, r3.Vec{})
		require.False(t, ok)
	})
}

func TestPointPlaneSource(t *testing.T) {
	t.Parallel()
	fan := func() *topo.Body {
		return &topo.Body{Kind: topo.BodySolid, Solid: true, Payload: facetedrecord.Payload{
			PointPlaneSourceEligible: true, Xform: r3.Identity(),
			PointSection: &facetedrecord.PointSectionRecord{},
		}}
	}

	t.Run("WholeFanIsItsOwnBase", func(t *testing.T) {
		t.Parallel()
		body := fan()
		record, ok := facetedrecord.PointPlaneSource(body)
		require.True(t, ok)
		fp, isFaceted := body.Payload.(facetedrecord.Payload)
		require.True(t, isFaceted)
		require.Same(t, fp.PointSection, record.Base.PointSection)
		require.Nil(t, record.Lower)
		require.Nil(t, record.Upper)
	})

	t.Run("SlabReadsItsOwnRecord", func(t *testing.T) {
		t.Parallel()
		body := fan()
		fp, isFaceted := body.Payload.(facetedrecord.Payload)
		require.True(t, isFaceted)
		upper := &facetedrecord.PointPlaneLimit{D: big.NewRat(1, 1)}
		fp.PointPlane = &facetedrecord.PointPlaneSliceRecord{Upper: upper}
		body.Payload = fp
		record, ok := facetedrecord.PointPlaneSource(body)
		require.True(t, ok)
		require.Same(t, upper, record.Upper)
	})

	for name, change := range map[string]func(*topo.Body){
		"Ineligible": func(b *topo.Body) {
			fp := b.Payload.(facetedrecord.Payload)
			fp.PointPlaneSourceEligible = false
			b.Payload = fp
		},
		"Placed": func(b *topo.Body) {
			fp := b.Payload.(facetedrecord.Payload)
			fp.Xform, _ = r3.Translation(r3.NewVec(1, 0, 0))
			b.Payload = fp
		},
		"NoPointSection": func(b *topo.Body) {
			fp := b.Payload.(facetedrecord.Payload)
			fp.PointSection = nil
			b.Payload = fp
		},
		"NotSolid": func(b *topo.Body) { b.Solid = false },
		"Sheet":    func(b *topo.Body) { b.Kind = topo.BodySheet },
	} {
		t.Run(name+"Refuses", func(t *testing.T) {
			t.Parallel()
			body := fan()
			change(body)
			_, ok := facetedrecord.PointPlaneSource(body)
			require.False(t, ok)
		})
	}
}
