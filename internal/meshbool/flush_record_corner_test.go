package meshbool_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessdispatch"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestFlushRecordCorner holds the end correspondence of agent-docs/flush-contact-design.md §4.2 for an analytic prism:
// the held vertex at a sketch fillet's tangent point with a straight side stands for the record point within its
// vertex bound. The prism lies on a turned frame through a point no float sum reaches, so the held vertex differs from
// the record point by rounding and a zero bound would not cover it.
func TestFlushRecordCorner(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{X: 0.1, Y: 0.2, Z: 0.3}, r3.Vec{X: 1, Y: 2}, r3.Vec{X: -2, Y: 1, Z: 0.5})
	require.NoError(t, err)
	pt := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	line := func(a, b sectionrecord.Point2) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{Start: a, End: b, TEnd: 1}
	}
	// The rectangle [0, 5] × [0, 3] with its corner (5, 3) rounded by a fillet of radius 1 about (4, 2), tangent to
	// the side u = 5 at (5, 2) and to the side v = 3 at (4, 3).
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(pt(0, 0), pt(5, 0)),
		line(pt(5, 0), pt(5, 2)),
		sectionrecord.ArcSeg{Center: pt(4, 2), Start: pt(5, 2), End: pt(4, 3), TEnd: 1},
		line(pt(4, 3), pt(0, 3)),
		line(pt(0, 3), pt(0, 0)),
	}}}
	pp := prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z1: 2, Profile: profile}
	body, err := prismrecord.Build(t.Context(), topo.Env{Doc: &topo.Document{}, Ref: 1, Alloc: topotest.Alloc{}}, pp,
		freeform.NewFreeformWork())
	require.NoError(t, err)
	m, err := tessdispatch.Body(t.Context(), body, 0.01, tessdispatch.All, tessdispatch.Options{Alloc: topotest.Alloc{}})
	require.NoError(t, err)
	require.Nil(t, m.VertexBound, "an analytic prism publishes no per-vertex record")
	beta, err := topobuild.VertexBounds(m)
	require.NoError(t, err)
	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	o, u, v := frame.Origin(), frame.U(), frame.V()
	for _, tangent := range []sectionrecord.Point2{pt(5, 2), pt(4, 3)} {
		// The record point O + u·U + v·V on the start cap, exactly.
		var exact [3]*big.Rat
		for k, c := range [3][3]float64{{o.X, u.X, v.X}, {o.Y, u.Y, v.Y}, {o.Z, u.Z, v.Z}} {
			exact[k] = new(big.Rat).Add(rat(c[0]), new(big.Rat).Add(
				new(big.Rat).Mul(rat(tangent.U), rat(c[1])), new(big.Rat).Mul(rat(tangent.V), rat(c[2]))))
		}
		held := frame.ToWorldUV(tangent.U, tangent.V)
		best, bestDist := -1, math.Inf(1)
		for i, p := range m.Vertices {
			if d := p.Sub(held).Len(); d < bestDist {
				best, bestDist = i, d
			}
		}
		require.Less(t, bestDist, 1e-9, "a held vertex stands at the tangent point %v", tangent)
		p := m.Vertices[best]
		dist2 := new(big.Rat)
		for k, c := range [3]float64{p.X, p.Y, p.Z} {
			d := new(big.Rat).Sub(rat(c), exact[k])
			dist2.Add(dist2, new(big.Rat).Mul(d, d))
		}
		require.Positive(t, dist2.Sign(), "the held vertex is not the record point")
		b := rat(beta[best])
		require.GreaterOrEqual(t, b.Mul(b, b).Cmp(dist2), 0, "β covers the record point at %v", tangent)
	}
}
