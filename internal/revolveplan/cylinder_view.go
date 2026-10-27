package revolveplan

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// CylinderPrism is the prism record that denotes exactly the solid rp
// denotes, when rp is a plain turned shaft: a full solid revolution of a
// rectangle with one side on the axis (agent-docs/general-boolean-design.md
// §3 "Revolved shaft as a prism"). The prism's section is one circle of the
// rectangle's radius about the axis, and its sweep is the rectangle's span
// along the axis. ok=false is a silent miss: every condition is an exact
// comparison of recorded data, and any record outside them has no view.
//
// The conditions are a full turn of a solid (not a sheet) with no section
// displacement and no blend roles; an axis along one of the plane's own axes,
// with every axis bound, the radial admission allowance and every snap
// allowance zero; one outer loop of exactly four whole lines, each along or
// across the axis, whose corners are the four corners of one rectangle
// with one side exactly on the axis; a radius the rectangle's coordinates
// state exactly as a float; and a frame that r3.NewFrame builds from the
// plane's axes bit for bit.
func (rp Payload) CylinderPrism() (prismrecord.Payload, bool) {
	if !rp.Full || rp.SurfaceResult || rp.SectionDelta != 0 || rp.SectionWhole ||
		len(rp.BlendSegs) != 0 || rp.DraftCircleCaps || rp.DraftCircleShellRoles {
		return prismrecord.Payload{}, false
	}
	ax := rp.Ax
	if ax.AUBound != 0 || ax.AVBound != 0 || ax.DUBound != 0 || ax.DVBound != 0 ||
		ax.RadialAdmitAllow != 0 || ax.Snap != (revolveaxis.SnapAllow{}) {
		return prismrecord.Payload{}, false
	}
	// along reads a meridian point's coordinate along the axis, across its
	// coordinate across it; axisAt is the axis line's across coordinate.
	var along, across func(sectionrecord.Point2) float64
	var axisAt float64
	var onV bool
	switch {
	case (ax.DU == 1 || ax.DU == -1) && ax.DV == 0:
		along = func(p sectionrecord.Point2) float64 { return p.U }
		across = func(p sectionrecord.Point2) float64 { return p.V }
		axisAt = ax.AV
	case (ax.DV == 1 || ax.DV == -1) && ax.DU == 0:
		along = func(p sectionrecord.Point2) float64 { return p.V }
		across = func(p sectionrecord.Point2) float64 { return p.U }
		axisAt, onV = ax.AU, true
	default:
		return prismrecord.Payload{}, false
	}
	lo, hi, radius, ok := axisRectangle(rp.Profile, along, across, axisAt)
	// A rim within the axis classification tolerance could read as the axis.
	if !ok || radius <= ax.SnapTol {
		return prismrecord.Payload{}, false
	}
	// The prism's sweep runs along the axis. With the axis on the plane's U,
	// (V, N, U) is right-handed; with it on V, (N, U, V) is. The circle's
	// centre is the axis point: its across coordinate along the plane axis
	// that carries it, zero along the plane normal.
	pu, pv, pn := rp.Frame.U(), rp.Frame.V(), rp.Frame.N()
	u, v, n := pv, pn, pu
	center := sectionrecord.Point2{U: axisAt}
	if onV {
		u, v, n = pn, pu, pv
		center = sectionrecord.Point2{V: axisAt}
	}
	frame, err := r3.NewFrame(rp.Frame.Origin(), u, v)
	if err != nil || frame.U() != u || frame.V() != v || frame.N() != n {
		return prismrecord.Payload{}, false
	}
	circle := sectionrecord.CircleSeg{Center: center, Radius: units.Millimeters(radius), CCW: true, TStart: 0, TEnd: 1}
	return prismrecord.Payload{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}},
		Frame:   frame,
		Z0:      lo,
		Z1:      hi,
		Xform:   rp.Xform,
	}, true
}

// axisRectangle reads a meridian that is one rectangle with one side on the
// axis: one outer loop of four whole lines, each along or across the axis,
// joined end to start bit for bit, whose corners take exactly two along
// values lo < hi and exactly two across values, one of them axisAt. The
// radius is the other across value's distance from axisAt, admitted only
// when that difference is itself a float.
func axisRectangle(profile momentinput.Profile, along, across func(sectionrecord.Point2) float64,
	axisAt float64) (float64, float64, float64, bool) {
	segs := profile.Outer.Segments
	if len(profile.Holes) != 0 || len(segs) != 4 {
		return 0, 0, 0, false
	}
	alongs := map[float64]struct{}{}
	acrosses := map[float64]struct{}{}
	corners := map[sectionrecord.Point2]struct{}{}
	for i, seg := range segs {
		l, ok := seg.(sectionrecord.LineSeg)
		if !ok || l.TStart != 0 || l.TEnd != 1 {
			return 0, 0, 0, false
		}
		next, ok := segs[(i+1)%len(segs)].(sectionrecord.LineSeg)
		if !ok || next.Start != l.End {
			return 0, 0, 0, false
		}
		if along(l.Start) != along(l.End) && across(l.Start) != across(l.End) {
			return 0, 0, 0, false
		}
		alongs[along(l.Start)] = struct{}{}
		acrosses[across(l.Start)] = struct{}{}
		corners[l.Start] = struct{}{}
	}
	if len(alongs) != 2 || len(acrosses) != 2 || len(corners) != 4 {
		return 0, 0, 0, false
	}
	if _, onAxis := acrosses[axisAt]; !onAxis {
		return 0, 0, 0, false
	}
	var lo, hi, rim float64
	first := true
	for a := range alongs {
		if first || a < lo {
			lo = a
		}
		if first || a > hi {
			hi = a
		}
		first = false
	}
	for c := range acrosses {
		if c != axisAt {
			rim = c
		}
	}
	radius := rim - axisAt
	if radius < 0 {
		radius = -radius
	}
	exact := new(big.Rat).Sub(new(big.Rat).SetFloat64(rim), new(big.Rat).SetFloat64(axisAt))
	if exact.Abs(exact).Cmp(new(big.Rat).SetFloat64(radius)) != 0 {
		return 0, 0, 0, false
	}
	return lo, hi, radius, true
}
