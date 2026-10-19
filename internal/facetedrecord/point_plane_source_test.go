package facetedrecord_test

import (
	"math"
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
	"github.com/lestrrat-3d/units"
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

	t.Run("ReversedLineParameterIsAdmitted", func(t *testing.T) {
		t.Parallel()
		sheet := lineSheet(t)
		withLine(t, sheet, func(l *sectionrecord.LineSeg) { l.TStart, l.TEnd = 1, 0 })
		_, ok := facetedrecord.PointPlaneSheet(sheet, r3.Vec{})
		require.True(t, ok)
	})

	// Each row breaks one condition of the exact sheet class and must refuse.
	// Shown to fail first (go test -overlay over point_plane_source.go): with
	// its own guard removed, each row below went red, except five guards a
	// later guard always covers. A payload that is no chain payload reads as
	// a zero one, whose placement is no identity; a segment that is no line
	// reads as a zero LineSeg, whose parameter range is neither whole range; a
	// face that is no plane reads as a zero Plane, whose zero normal leaves
	// the side test nothing to clear; a point line has a zero normal, so its
	// apex offset is zero. No input reaches the finite frame-normal guard: a
	// face's plane frame is built through r3.NewFrame, which refuses a
	// non-finite axis. The rows for the first four still pin the refusal.
	frameAt := func(t *testing.T, origin, u, v r3.Vec) r3.Frame {
		t.Helper()
		frame, err := r3.NewFrame(origin, u, v)
		require.NoError(t, err)
		return frame
	}
	for _, tc := range []struct {
		name    string
		apex    r3.Vec
		breakIt func(*testing.T, *topo.Body)
	}{
		{name: "NotAChainSheet", breakIt: func(_ *testing.T, b *topo.Body) { b.Payload = facetedrecord.Payload{} }},
		{name: "FrameOffOrigin", breakIt: func(t *testing.T, b *topo.Body) {
			withPayload(t, b, func(cp *chainrecord.Payload) {
				cp.Frame = frameAt(t, r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
			})
		}},
		{name: "FrameUOffX", breakIt: func(t *testing.T, b *topo.Body) {
			withPayload(t, b, func(cp *chainrecord.Payload) {
				cp.Frame = frameAt(t, r3.Vec{}, r3.NewVec(-1, 0, 0), r3.NewVec(0, 1, 0))
			})
		}},
		{name: "FrameVOffY", breakIt: func(t *testing.T, b *topo.Body) {
			withPayload(t, b, func(cp *chainrecord.Payload) {
				cp.Frame = frameAt(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, -1, 0))
			})
		}},
		{name: "StartLevelDisplaced", breakIt: func(t *testing.T, b *topo.Body) {
			withPayload(t, b, func(cp *chainrecord.Payload) { cp.Z0Delta = 1 })
		}},
		{name: "EndLevelDisplaced", breakIt: func(t *testing.T, b *topo.Body) {
			withPayload(t, b, func(cp *chainrecord.Payload) { cp.Z1Delta = 1 })
		}},
		{name: "TwoChains", breakIt: func(t *testing.T, b *topo.Body) {
			withPayload(t, b, func(cp *chainrecord.Payload) { cp.Chains = append(cp.Chains, cp.Chains[0]) })
		}},
		{name: "TwoSegments", breakIt: func(t *testing.T, b *topo.Body) {
			withPayload(t, b, func(cp *chainrecord.Payload) {
				segs := cp.Chains[0].Segments
				cp.Chains = []sectionrecord.ChainRecord{{Segments: append(append([]sectionrecord.CurveSegment(nil), segs...), segs[0])}}
			})
		}},
		{name: "TwoFaces", breakIt: func(_ *testing.T, b *topo.Body) {
			shell := b.Lumps[0].Shells[0]
			shell.Faces = append(shell.Faces, &topo.Face{})
		}},
		{name: "ArcSegment", breakIt: func(t *testing.T, b *topo.Body) {
			withPayload(t, b, func(cp *chainrecord.Payload) {
				cp.Chains = []sectionrecord.ChainRecord{{Segments: []sectionrecord.CurveSegment{sectionrecord.ArcSeg{
					Center: sectionrecord.Point2{U: -25}, Start: sectionrecord.Point2{U: 5, V: -30},
					End: sectionrecord.Point2{U: 5, V: 30}, TStart: 0, TEnd: 1}}}}
			})
		}},
		{name: "TrimmedLine", breakIt: func(t *testing.T, b *topo.Body) {
			withLine(t, b, func(l *sectionrecord.LineSeg) { l.TEnd = 0.5 })
		}},
		{name: "TrimmedReversedLine", breakIt: func(t *testing.T, b *topo.Body) {
			withLine(t, b, func(l *sectionrecord.LineSeg) { l.TStart, l.TEnd = 1, 0.5 })
		}},
		{name: "DisplacedVertex", breakIt: func(_ *testing.T, b *topo.Body) {
			topobuild.Vertices(b)[0].Bound = units.Millimeters(1)
		}},
		{name: "CurvedFace", breakIt: func(_ *testing.T, b *topo.Body) {
			topobuild.Faces(b)[0].Surface = surfacegeom.Cylinder{}
		}},
		{name: "NonFiniteApex", apex: r3.NewVec(math.NaN(), 0, 0)},
		{name: "NonFiniteLineStart", breakIt: func(t *testing.T, b *topo.Body) {
			withLine(t, b, func(l *sectionrecord.LineSeg) { l.Start.U = math.Inf(1) })
		}},
		{name: "NonFiniteLineEnd", breakIt: func(t *testing.T, b *topo.Body) {
			withLine(t, b, func(l *sectionrecord.LineSeg) { l.End.V = math.Inf(1) })
		}},
		{name: "NonFiniteLevel", breakIt: func(t *testing.T, b *topo.Body) {
			withPayload(t, b, func(cp *chainrecord.Payload) { cp.Z1 = math.Inf(1) })
		}},
		{name: "PointLine", breakIt: func(t *testing.T, b *topo.Body) {
			withLine(t, b, func(l *sectionrecord.LineSeg) { l.End = l.Start })
		}},
		{name: "NonFiniteNormalBound", breakIt: func(_ *testing.T, b *topo.Body) {
			topobuild.Faces(b)[0].NormalBound = math.NaN()
		}},
		{name: "NegativeNormalBound", breakIt: func(_ *testing.T, b *topo.Body) {
			topobuild.Faces(b)[0].NormalBound = -1
		}},
		{name: "NormalBoundSwallowsTheSide", breakIt: func(_ *testing.T, b *topo.Body) {
			topobuild.Faces(b)[0].NormalBound = 1
		}},
	} {
		t.Run(tc.name+"Refuses", func(t *testing.T) {
			t.Parallel()
			sheet := lineSheet(t)
			if tc.breakIt != nil {
				tc.breakIt(t, sheet)
			}
			_, ok := facetedrecord.PointPlaneSheet(sheet, tc.apex)
			require.False(t, ok)
		})
	}
}

// withPayload rewrites b's chain payload in place.
func withPayload(t *testing.T, b *topo.Body, change func(*chainrecord.Payload)) {
	t.Helper()
	cp, ok := b.Payload.(chainrecord.Payload)
	require.True(t, ok)
	change(&cp)
	b.Payload = cp
}

// withLine rewrites the one recorded line segment of b's chain payload.
func withLine(t *testing.T, b *topo.Body, change func(*sectionrecord.LineSeg)) {
	t.Helper()
	withPayload(t, b, func(cp *chainrecord.Payload) {
		line, ok := cp.Chains[0].Segments[0].(sectionrecord.LineSeg)
		require.True(t, ok)
		change(&line)
		cp.Chains = []sectionrecord.ChainRecord{{Segments: []sectionrecord.CurveSegment{line}}}
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
