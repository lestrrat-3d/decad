package revolveplan_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// loopOf joins the points into a closed loop of whole lines.
func loopOf(pts ...sectionrecord.Point2) momentinput.Profile {
	var segs []sectionrecord.CurveSegment
	for i, p := range pts {
		segs = append(segs, sectionrecord.LineSeg{Start: p, End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1})
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
}

// shaftPayload is the full revolution of the rectangle u in [0, 40],
// v in [0, 10] about the u axis on the XY plane.
func shaftPayload(t *testing.T) revolveplan.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return revolveplan.Payload{
		Frame: frame, Xform: r3.Identity(), Phi1: 2 * math.Pi, Full: true,
		Ax:      revolveaxis.Frame{DU: 1, SnapTol: 4e-8},
		Profile: loopOf(sectionrecord.Point2{}, sectionrecord.Point2{U: 40}, sectionrecord.Point2{U: 40, V: 10}, sectionrecord.Point2{V: 10}),
	}
}

func TestCylinderPrism(t *testing.T) {
	t.Parallel()

	t.Run("axis along u", func(t *testing.T) {
		t.Parallel()
		move, err := r3.Translation(r3.Vec{X: 1, Y: 2, Z: 3})
		require.NoError(t, err)
		rp := shaftPayload(t)
		rp.Xform = move
		pp, ok := rp.CylinderPrism()
		require.True(t, ok)
		// (V, N, U) of the plane: y, z, x.
		require.Equal(t, r3.NewVec(0, 1, 0), pp.Frame.U())
		require.Equal(t, r3.NewVec(0, 0, 1), pp.Frame.V())
		require.Equal(t, r3.NewVec(1, 0, 0), pp.Frame.N())
		require.Equal(t, r3.Vec{}, pp.Frame.Origin())
		require.Equal(t, 0.0, pp.Z0)
		require.Equal(t, 40.0, pp.Z1)
		require.Equal(t, move, pp.Xform)
		require.Empty(t, pp.Profile.Holes)
		require.Equal(t, []sectionrecord.CurveSegment{sectionrecord.CircleSeg{
			Radius: units.Millimeters(10), CCW: true, TStart: 0, TEnd: 1,
		}}, pp.Profile.Outer.Segments)
	})

	t.Run("axis along v, off the origin, walked clockwise", func(t *testing.T) {
		t.Parallel()
		// The rectangle u in [-7, -2], v in [5, 30] about the line u = -2,
		// walked clockwise, and its axis pointing down v.
		rp := shaftPayload(t)
		rp.Ax = revolveaxis.Frame{AU: -2, DV: -1, SnapTol: 4e-8}
		rp.Profile = loopOf(sectionrecord.Point2{U: -2, V: 5}, sectionrecord.Point2{U: -7, V: 5},
			sectionrecord.Point2{U: -7, V: 30}, sectionrecord.Point2{U: -2, V: 30})
		pp, ok := rp.CylinderPrism()
		require.True(t, ok)
		// (N, U, V) of the plane: z, x, y.
		require.Equal(t, r3.NewVec(0, 0, 1), pp.Frame.U())
		require.Equal(t, r3.NewVec(1, 0, 0), pp.Frame.V())
		require.Equal(t, r3.NewVec(0, 1, 0), pp.Frame.N())
		require.Equal(t, 5.0, pp.Z0)
		require.Equal(t, 30.0, pp.Z1)
		require.Equal(t, []sectionrecord.CurveSegment{sectionrecord.CircleSeg{
			Center: sectionrecord.Point2{V: -2}, Radius: units.Millimeters(5), CCW: true, TStart: 0, TEnd: 1,
		}}, pp.Profile.Outer.Segments)
	})

	// Each edit leaves a record whose solid the view would not state
	// exactly, or one outside the plain turned shaft; each must miss.
	turned, err := r3.NewFrame(r3.Vec{}, r3.NewVec(math.Cos(0.1), math.Sin(0.1), 0), r3.NewVec(-math.Sin(0.1), math.Cos(0.1), 0))
	require.NoError(t, err)
	// 10.3 − 0.1 is not a float.
	inexact := new(big.Rat).Sub(new(big.Rat).SetFloat64(10.3), new(big.Rat).SetFloat64(0.1))
	require.NotEqual(t, 0, inexact.Cmp(new(big.Rat).SetFloat64(10.3-0.1)))
	for name, edit := range map[string]func(*revolveplan.Payload){
		"partial turn":       func(p *revolveplan.Payload) { p.Full = false },
		"sheet":              func(p *revolveplan.Payload) { p.SurfaceResult = true },
		"section delta":      func(p *revolveplan.Payload) { p.SectionDelta = 1e-12 },
		"whole section":      func(p *revolveplan.Payload) { p.SectionWhole = true },
		"blend roles":        func(p *revolveplan.Payload) { p.BlendSegs = []map[int]struct{}{{1: {}}} },
		"draft caps":         func(p *revolveplan.Payload) { p.DraftCircleCaps = true },
		"draft shell roles":  func(p *revolveplan.Payload) { p.DraftCircleShellRoles = true },
		"anchor u bound":     func(p *revolveplan.Payload) { p.Ax.AUBound = 1e-15 },
		"anchor v bound":     func(p *revolveplan.Payload) { p.Ax.AVBound = 1e-15 },
		"direction u bound":  func(p *revolveplan.Payload) { p.Ax.DUBound = 1e-15 },
		"direction v bound":  func(p *revolveplan.Payload) { p.Ax.DVBound = 1e-15 },
		"radial allowance":   func(p *revolveplan.Payload) { p.Ax.RadialAdmitAllow = 1e-15 },
		"snap allowance":     func(p *revolveplan.Payload) { p.Ax.Snap.Area = 1e-15 },
		"oblique axis":       func(p *revolveplan.Payload) { p.Ax.DU, p.Ax.DV = 0.6, 0.8 },
		"axis u and v":       func(p *revolveplan.Payload) { p.Ax.DV = 1 },
		"turned plane frame": func(p *revolveplan.Payload) { p.Frame = turned },
		"meridian hole": func(p *revolveplan.Payload) {
			p.Profile.Holes = []sectionrecord.LoopRecord{loopOf(sectionrecord.Point2{U: 1, V: 1},
				sectionrecord.Point2{U: 2, V: 1}, sectionrecord.Point2{U: 2, V: 2}).Outer}
		},
		"stepped meridian": func(p *revolveplan.Payload) {
			p.Profile = loopOf(sectionrecord.Point2{}, sectionrecord.Point2{U: 40}, sectionrecord.Point2{U: 40, V: 8},
				sectionrecord.Point2{U: 20, V: 8}, sectionrecord.Point2{U: 20, V: 10}, sectionrecord.Point2{V: 10})
		},
		"arc segment": func(p *revolveplan.Payload) {
			p.Profile.Outer.Segments[1] = sectionrecord.ArcSeg{Center: sectionrecord.Point2{U: 40, V: 5},
				Start: sectionrecord.Point2{U: 40}, End: sectionrecord.Point2{U: 40, V: 10}, TStart: 0, TEnd: 1}
		},
		"trimmed line": func(p *revolveplan.Payload) {
			l := p.Profile.Outer.Segments[0].(sectionrecord.LineSeg)
			l.TEnd = 0.5
			p.Profile.Outer.Segments[0] = l
		},
		"open loop": func(p *revolveplan.Payload) {
			l := p.Profile.Outer.Segments[0].(sectionrecord.LineSeg)
			l.End = sectionrecord.Point2{U: 39}
			p.Profile.Outer.Segments[0] = l
		},
		"crossed diagonals": func(p *revolveplan.Payload) {
			p.Profile = loopOf(sectionrecord.Point2{}, sectionrecord.Point2{U: 40, V: 10},
				sectionrecord.Point2{U: 40}, sectionrecord.Point2{V: 10})
		},
		"collinear loop": func(p *revolveplan.Payload) {
			p.Profile = loopOf(sectionrecord.Point2{}, sectionrecord.Point2{U: 20},
				sectionrecord.Point2{U: 40}, sectionrecord.Point2{U: 10})
		},
		"radial collinear loop": func(p *revolveplan.Payload) {
			p.Profile = loopOf(sectionrecord.Point2{}, sectionrecord.Point2{V: 5},
				sectionrecord.Point2{V: 10}, sectionrecord.Point2{V: 3})
		},
		"spike loop": func(p *revolveplan.Payload) {
			p.Profile = loopOf(sectionrecord.Point2{}, sectionrecord.Point2{U: 40},
				sectionrecord.Point2{U: 40, V: 10}, sectionrecord.Point2{U: 40})
		},
		"zero-length fifth line": func(p *revolveplan.Payload) {
			p.Profile = loopOf(sectionrecord.Point2{}, sectionrecord.Point2{U: 40},
				sectionrecord.Point2{U: 40, V: 10}, sectionrecord.Point2{V: 10}, sectionrecord.Point2{})
		},
		"tube": func(p *revolveplan.Payload) {
			p.Profile = loopOf(sectionrecord.Point2{V: 4}, sectionrecord.Point2{U: 40, V: 4},
				sectionrecord.Point2{U: 40, V: 10}, sectionrecord.Point2{V: 10})
		},
		"inexact radius": func(p *revolveplan.Payload) {
			p.Ax.AV = 0.1
			p.Profile = loopOf(sectionrecord.Point2{V: 0.1}, sectionrecord.Point2{U: 40, V: 0.1},
				sectionrecord.Point2{U: 40, V: 10.3}, sectionrecord.Point2{V: 10.3})
		},
		"radius within the snap tolerance": func(p *revolveplan.Payload) {
			p.Profile = loopOf(sectionrecord.Point2{}, sectionrecord.Point2{U: 40},
				sectionrecord.Point2{U: 40, V: 1e-8}, sectionrecord.Point2{V: 1e-8})
		},
	} {
		rp := shaftPayload(t)
		edit(&rp)
		_, ok := rp.CylinderPrism()
		require.False(t, ok, name)
	}
}

func TestCylinderSteps(t *testing.T) {
	t.Parallel()

	pt := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	circle := func(center sectionrecord.Point2, r float64) []sectionrecord.CurveSegment {
		return []sectionrecord.CurveSegment{sectionrecord.CircleSeg{
			Center: center, Radius: units.Millimeters(r), CCW: true, TStart: 0, TEnd: 1,
		}}
	}

	t.Run("two steps about u", func(t *testing.T) {
		t.Parallel()
		// Radius 10 over u in [0, 20], 8 over [20, 40], walked from the
		// axis's far end.
		rp := shaftPayload(t)
		rp.Profile = loopOf(pt(40, 0), pt(40, 8), pt(20, 8), pt(20, 10), pt(0, 10), pt(0, 0))
		steps, ok := rp.CylinderSteps()
		require.True(t, ok)
		require.Len(t, steps, 2)
		for i, want := range []struct{ z0, z1, r float64 }{{0, 20, 10}, {20, 40, 8}} {
			require.Equal(t, want.z0, steps[i].Z0)
			require.Equal(t, want.z1, steps[i].Z1)
			require.Equal(t, circle(sectionrecord.Point2{}, want.r), steps[i].Profile.Outer.Segments)
			require.Equal(t, r3.NewVec(1, 0, 0), steps[i].Frame.N())
		}
		_, ok = rp.CylinderPrism()
		require.False(t, ok, `a stepped meridian has no single prism view`)
	})

	t.Run("three steps about v, below the axis, walked clockwise", func(t *testing.T) {
		t.Parallel()
		// The axis is the line u = -2 pointing down v; the rims lie at
		// u = -7, -4 and -9 over v in [5, 10], [10, 20] and [20, 30].
		rp := shaftPayload(t)
		rp.Ax = revolveaxis.Frame{AU: -2, DV: -1, SnapTol: 4e-8}
		rp.Profile = loopOf(pt(-2, 5), pt(-7, 5), pt(-7, 10), pt(-4, 10), pt(-4, 20), pt(-9, 20),
			pt(-9, 30), pt(-2, 30))
		steps, ok := rp.CylinderSteps()
		require.True(t, ok)
		require.Len(t, steps, 3)
		for i, want := range []struct{ z0, z1, r float64 }{{5, 10, 5}, {10, 20, 2}, {20, 30, 7}} {
			require.Equal(t, want.z0, steps[i].Z0)
			require.Equal(t, want.z1, steps[i].Z1)
			require.Equal(t, circle(sectionrecord.Point2{V: -2}, want.r), steps[i].Profile.Outer.Segments)
		}
	})

	// Each meridian is one loop of whole lines along or across the axis that
	// is no staircase on the axis; each must miss.
	for name, profile := range map[string]momentinput.Profile{
		// The rims lie on both sides of the axis, joined by a riser through it.
		"rims on both sides": loopOf(pt(0, 0), pt(40, 0), pt(40, 8), pt(20, 8), pt(20, -10), pt(0, -10)),
		// The r = 8 rim runs back over the r = 10 one.
		"overhanging rim": loopOf(pt(0, 0), pt(40, 0), pt(40, 8), pt(10, 8), pt(10, 10), pt(30, 10),
			pt(30, 12), pt(0, 12)),
		// Two rims of radius 10 meet at u = 20, and the riser at u = 40 is
		// split in two, so the risers still number one more than the rims.
		"equal neighbouring rims": loopOf(pt(0, 0), pt(40, 0), pt(40, 5), pt(40, 10), pt(20, 10), pt(0, 10)),
		// Two slanted lines cross between the steps' ends.
		"slanted risers": loopOf(pt(0, 0), pt(40, 0), pt(40, 8), pt(20, 8), pt(0, 10), pt(20, 10)),
		"two axis lines": loopOf(pt(0, 0), pt(20, 0), pt(20, 10), pt(40, 10), pt(40, 0), pt(30, 0),
			pt(30, -5), pt(0, -5)),
		"extra riser": loopOf(pt(0, 0), pt(40, 0), pt(40, 10), pt(40, 12), pt(40, 10), pt(0, 10)),
		"inexact second radius": func() momentinput.Profile {
			return loopOf(pt(0, 0.1), pt(40, 0.1), pt(40, 8), pt(20, 8), pt(20, 10.3), pt(0, 10.3))
		}(),
	} {
		rp := shaftPayload(t)
		if name == "inexact second radius" {
			rp.Ax.AV = 0.1
		}
		rp.Profile = profile
		_, ok := rp.CylinderSteps()
		require.False(t, ok, name)
	}

	t.Run("second step within the snap tolerance", func(t *testing.T) {
		t.Parallel()
		rp := shaftPayload(t)
		rp.Profile = loopOf(pt(0, 0), pt(40, 0), pt(40, 1e-8), pt(20, 1e-8), pt(20, 10), pt(0, 10))
		_, ok := rp.CylinderSteps()
		require.False(t, ok)
	})
}
