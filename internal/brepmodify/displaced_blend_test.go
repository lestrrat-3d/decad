package brepmodify_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The displaced-receiver fixtures of agent-docs/brep-modify-design.md §4.4:
// the class-B D-flat record (r = 10, L = 40 shaft along x, flat at z = 7),
// as the crossing reach builds it and with every displaced crossing moved by
// eps. Both records denote the same exact D-flat: the cap and flat lines lie
// on their exact planes, the arcs about the exact centre, and every crossing
// lies within the record's displacement of the true one.

// dFlatRecord is the class-B Cut of the shaft by the box above z = 7.
func dFlatRecord(t *testing.T) breprecord.Payload {
	t.Helper()
	yz, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	circle := sectionrecord.CircleSeg{Radius: units.Millimeters(10), CCW: true, TStart: 0, TEnd: 1}
	shaft := prismrecord.Payload{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}},
		Frame:   yz, Z1: 40, Xform: r3.Identity(),
	}
	xy, err := r3.NewFrame(r3.Vec{Z: 7}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	corners := []sectionrecord.Point2{{U: -5, V: -20}, {U: 45, V: -20}, {U: 45, V: 20}, {U: -5, V: 20}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%4], TStart: 0, TEnd: 1})
	}
	tool := prismrecord.Payload{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}},
		Frame:   xy, Z1: 20, Xform: r3.Identity(),
	}
	out, ok, err := classbgeom.OfPayloads(t.Context(), meshbool.OpCut, shaft, tool, r3.Identity(), 0)
	require.NoError(t, err)
	require.True(t, ok, `the pair takes class B`)
	bp, ok := out.(breprecord.Payload)
	require.True(t, ok)
	require.NotZero(t, bp.SectionDelta())
	return bp
}

// perturbed is bp with every recorded crossing coordinate ±yf moved outward
// by eps, and every displaced face's displacement set to 2·eps, which covers
// the true crossing (±√51, 7) still. eps = 0 leaves the record as it is.
func perturbed(t *testing.T, bp breprecord.Payload, eps float64) breprecord.Payload {
	t.Helper()
	if eps == 0 {
		return bp
	}
	var yf float64
	for _, f := range bp.Faces {
		if a, ok := f.Wall.(sectionrecord.ArcSeg); ok {
			yf = math.Abs(a.Start.U)
		}
	}
	require.NotZero(t, yf)
	move := func(x float64) float64 {
		switch x {
		case yf:
			return yf + eps
		case -yf:
			return -yf - eps
		}
		return x
	}
	point := func(p sectionrecord.Point2) sectionrecord.Point2 {
		return sectionrecord.Point2{U: move(p.U), V: move(p.V)}
	}
	segment := func(seg sectionrecord.CurveSegment) sectionrecord.CurveSegment {
		switch s := seg.(type) {
		case sectionrecord.LineSeg:
			s.Start, s.End = point(s.Start), point(s.End)
			return s
		case sectionrecord.ArcSeg:
			s.Start, s.End = point(s.Start), point(s.End)
			return s
		}
		return seg
	}
	out := bp
	out.Faces = make([]breprecord.Face, len(bp.Faces))
	for i, f := range bp.Faces {
		if f.Delta != 0 {
			f.Delta = 2 * eps
		}
		if f.Wall != nil {
			f.Wall = segment(f.Wall)
		}
		if f.Region != nil {
			region := momentinput.Profile{Outer: sectionrecord.LoopRecord{}}
			for _, seg := range f.Region.Outer.Segments {
				region.Outer.Segments = append(region.Outer.Segments, segment(seg))
			}
			require.Empty(t, f.Region.Holes)
			f.Region = &region
		}
		out.Faces[i] = f
	}
	return out
}

func displacedEnv() topo.Env { return topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}} }

// flatEdgesOf is the body's two long straight edges at z = 7.
func flatEdgesOf(t *testing.T, b *topo.Body) []*topo.Edge {
	t.Helper()
	var out []*topo.Edge
	for _, e := range topobuild.Edges(b) {
		if _, ok := e.Curve.(surfacegeom.Line3); !ok || e.Start == nil || e.End == nil {
			continue
		}
		a, c := e.Start.Position, e.End.Position
		if math.Abs(a.Z-7) < 1e-3 && math.Abs(c.Z-7) < 1e-3 && math.Abs(a.X-c.X) > 20 {
			out = append(out, e)
		}
	}
	require.Len(t, out, 2)
	return out
}

// blendRecord builds bp and fillets (or chamfers) both flat edges at size.
func blendRecord(t *testing.T, bp breprecord.Payload, fillet bool, size float64) (*topo.Body, error) {
	t.Helper()
	env := displacedEnv()
	b, err := breprecord.Build(t.Context(), env, bp)
	require.NoError(t, err)
	edges := flatEdgesOf(t, b)
	env.Ref++
	var res brepmodify.Result
	if fillet {
		res, err = brepmodify.Fillet(t.Context(), env, b, brepmodify.FilletCall{Sel: "flat", Edges: edges, R: size})
	} else {
		res, err = brepmodify.Chamfer(t.Context(), env, b, brepmodify.ChamferCall{Sel: "flat", Edges: edges, D: size})
	}
	return res.Body, err
}

// ratOf is f as an exact rational.
func ratOf(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }

// volumeOf is a body's volume and its bound in mm³.
func volumeOf(t *testing.T, b *topo.Body) (float64, float64) {
	t.Helper()
	v, err := b.Volume.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	bound, err := b.Volume.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	return v, bound
}

// requireSameBody checks that two measurements of what is the same exact body
// agree within their two bounds.
func requireSameBody(t *testing.T, got, want *topo.Body) {
	t.Helper()
	v, b := volumeOf(t, got)
	wv, wb := volumeOf(t, want)
	require.LessOrEqualf(t, math.Abs(v-wv), b+wb, `volumes %.17g and %.17g differ by %.3e beyond their bounds %.3e and %.3e`, v, wv, v-wv, b, wb)
}

// TestDisplacedBlendReadsCarriers is Claim D and Claim C over a record whose
// crossings sit eps = 1e-4 off the real record's. The real record's blends
// are the exact D-flat's (apitest's TestRevolvedShaftClassBBlend encloses
// their closed forms), so the perturbed record's must agree within both
// bounds.
//
// Shown-to-fail: with the carrier override dropped, the fillet offsets the
// recorded radius (about 10 + 7.1e-5) and its volume moves by about 0.14 mm³;
// with the chamfer built at SectionDelta 0 (no PD7 charge) its bound no
// longer covers its feet moving by about 1e-4, and with the charge cut to δ
// it states less than the arc foot's transport R·δ/(R − δ).
func TestDisplacedBlendReadsCarriers(t *testing.T) {
	t.Parallel()
	record := dFlatRecord(t)
	for _, tc := range []struct {
		name   string
		fillet bool
		size   float64
	}{{"fillet", true, 1}, {"chamfer", false, 1}} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			want, err := blendRecord(t, record, tc.fillet, tc.size)
			require.NoError(t, err)
			got, err := blendRecord(t, perturbed(t, record, 1e-4), tc.fillet, tc.size)
			require.NoError(t, err)
			pp, ok := got.Payload.(prismrecord.Payload)
			require.True(t, ok, `the blend is a prism`)
			if tc.fillet {
				require.Zero(t, pp.SectionDelta, `a fillet from exact carriers keeps no displacement`)
			} else {
				// PD7: δ_out is R·δ/(R − δ) for the record's δ = 2e-4 on the
				// R = 10 wall, rounded up, which exceeds δ itself.
				delta := 2 * 1e-4
				transport := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(10, 1), ratOf(delta)),
					new(big.Rat).Sub(big.NewRat(10, 1), ratOf(delta)))
				require.Greater(t, transport.Cmp(ratOf(delta)), 0)
				require.GreaterOrEqual(t, ratOf(pp.SectionDelta).Cmp(transport), 0,
					`the chamfer charges the arc foot's transport R·δ/(R − δ), not δ alone`)
			}
			requireSameBody(t, got, want)
		})
	}
}

// TestDisplacedBlendRecordGates is PD2 and PD3 on the real record: a wall
// with no recorded carrier, a carrier off the wall's radius, and a carrier
// centre one ulp off the arc's each refuse SB1 with the row. Shown-to-fail:
// without PD2 the carrier-less wall fillets as a zero circle; without PD3's
// annulus or centre leg its case builds (TestDisplacedCarrierCircle holds the
// half-radius leg, which no buildable record reaches).
func TestDisplacedBlendRecordGates(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		edit func(*breprecord.Face)
		row  string
	}{
		{"no carrier", func(f *breprecord.Face) { f.WallCarrier.Radius = 0 }, `brep-modify PD2`},
		{"carrier radius", func(f *breprecord.Face) { f.WallCarrier.Radius = 10.5 }, `brep-modify PD3`},
		{"carrier centre", func(f *breprecord.Face) {
			f.WallCarrier.Center.U = math.Nextafter(f.WallCarrier.Center.U, 1)
		}, `brep-modify PD3`},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			bp := dFlatRecord(t)
			for i := range bp.Faces {
				if !bp.Faces[i].Planar() {
					tc.edit(&bp.Faces[i])
				}
			}
			for _, fillet := range []bool{true, false} {
				_, err := blendRecord(t, bp, fillet, 1)
				require.ErrorIs(t, err, decaderr.ErrUnsupported)
				require.ErrorContains(t, err, `brep-modify SB1`)
				require.ErrorContains(t, err, tc.row)
			}
		})
	}
}
