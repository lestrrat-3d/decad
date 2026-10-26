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
