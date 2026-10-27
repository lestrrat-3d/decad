package revolveplan

import (
	"cmp"
	"math/big"
	"slices"

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
// §3 "Revolved shaft as a prism"). It is CylinderSteps' reading of a meridian
// with one step. ok=false is a silent miss.
func (rp Payload) CylinderPrism() (prismrecord.Payload, bool) {
	steps, ok := rp.CylinderSteps()
	if !ok || len(steps) != 1 {
		return prismrecord.Payload{}, false
	}
	return steps[0], true
}

// CylinderSteps is the stack of coaxial circular prism records that denotes
// exactly the solid rp denotes, when rp is a turned shaft, plain or stepped: a
// full solid revolution of a staircase with one side on the axis
// (agent-docs/general-boolean-design.md §3 "Revolved shaft as a prism"). Each
// prism's section is one circle of its step's radius about the axis, and its
// sweep is the step's span along the axis; the prisms are in order along the
// axis, and each one's Z1 is the next one's Z0 bit for bit. ok=false is a
// silent miss: every condition is an exact comparison of recorded data, and
// any record outside them has no view.
//
// The conditions are a full turn of a solid (not a sheet) with no section
// displacement and no blend roles; an axis along one of the plane's own axes,
// with every axis bound, the radial admission allowance and every snap
// allowance zero; one outer loop of whole lines, each along or across the
// axis, whose lines are exactly the boundary of one staircase (axisStaircase);
// radii the meridian's coordinates state exactly as floats; and a frame that
// r3.NewFrame builds from the plane's axes bit for bit.
func (rp Payload) CylinderSteps() ([]prismrecord.Payload, bool) {
	if !rp.Full || rp.SurfaceResult || rp.SectionDelta != 0 || rp.SectionWhole ||
		len(rp.BlendSegs) != 0 || rp.DraftCircleCaps || rp.DraftCircleShellRoles {
		return nil, false
	}
	ax := rp.Ax
	if ax.AUBound != 0 || ax.AVBound != 0 || ax.DUBound != 0 || ax.DVBound != 0 ||
		ax.RadialAdmitAllow != 0 || ax.Snap != (revolveaxis.SnapAllow{}) {
		return nil, false
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
		return nil, false
	}
	steps, ok := axisStaircase(rp.Profile, along, across, axisAt)
	if !ok {
		return nil, false
	}
	for _, s := range steps {
		// A rim within the axis classification tolerance could read as the
		// axis.
		if s.radius <= ax.SnapTol {
			return nil, false
		}
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
		return nil, false
	}
	out := make([]prismrecord.Payload, len(steps))
	for i, s := range steps {
		circle := sectionrecord.CircleSeg{Center: center, Radius: units.Millimeters(s.radius), CCW: true, TStart: 0, TEnd: 1}
		out[i] = prismrecord.Payload{
			Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}},
			Frame:   frame,
			Z0:      s.lo,
			Z1:      s.hi,
			Xform:   rp.Xform,
		}
	}
	return out, true
}

// meridianStep is one step of a staircase meridian: its span [lo, hi] along
// the axis, its rim's across coordinate and its radius.
type meridianStep struct {
	lo, hi, rim, radius float64
}

// axisStaircase reads a meridian that is one staircase with one side on the
// axis: one outer loop of whole lines, joined end to start bit for bit, each
// along or across the axis and of nonzero length. Exactly one line runs along
// the axis itself, over [lo, hi]. The lines along the axis off it, the rims,
// all lie on one side of the axis, and sorted by their start along it they
// tile [lo, hi] end to start bit for bit, each with a rim different from its
// neighbour's. The lines across the axis, the risers, number one more than
// the rims. Each radius, a rim's distance from axisAt, is admitted only when
// that difference is itself a float.
//
// The risers are then the staircase's: one at lo from the axis to the first
// rim, one at hi from the axis to the last, and one at each step boundary
// between the two rims there. The lines along the axis end at 2n+2 distinct
// points for n rims, one line at each, and at each along value two of them
// lie. The loop is a closed walk, so every point it visits is the end of an
// even number of its lines, and each of those points needs an odd number of
// riser ends. The n+1 risers have 2n+2 ends, so each point is the end of
// exactly one riser and no riser ends anywhere else; a riser keeps its along
// value, so it joins the two points at one along value. The meridian is the
// staircase, and its revolution the stack of coaxial cylinders. The one axis
// line is needed: a zero-area spike out along the axis and back adds two axis
// lines whose ends pair with each other.
func axisStaircase(profile momentinput.Profile, along, across func(sectionrecord.Point2) float64,
	axisAt float64) ([]meridianStep, bool) {
	segs := profile.Outer.Segments
	if len(profile.Holes) != 0 || len(segs) < 4 {
		return nil, false
	}
	var axisLines, risers int
	var lo, hi float64
	var steps []meridianStep
	for i, seg := range segs {
		l, ok := seg.(sectionrecord.LineSeg)
		if !ok || l.TStart != 0 || l.TEnd != 1 {
			return nil, false
		}
		next, ok := segs[(i+1)%len(segs)].(sectionrecord.LineSeg)
		if !ok || next.Start != l.End {
			return nil, false
		}
		alongSame := along(l.Start) == along(l.End)
		acrossSame := across(l.Start) == across(l.End)
		switch {
		case alongSame == acrossSame:
			// Zero length or slanted.
			return nil, false
		case acrossSame && across(l.Start) == axisAt:
			axisLines++
			lo, hi = min(along(l.Start), along(l.End)), max(along(l.Start), along(l.End))
		case acrossSame:
			steps = append(steps, meridianStep{
				lo: min(along(l.Start), along(l.End)), hi: max(along(l.Start), along(l.End)), rim: across(l.Start),
			})
		default:
			risers++
		}
	}
	if axisLines != 1 || len(steps) == 0 || risers != len(steps)+1 {
		return nil, false
	}
	slices.SortFunc(steps, func(a, b meridianStep) int {
		switch {
		case a.lo != b.lo:
			return cmp.Compare(a.lo, b.lo)
		default:
			return cmp.Compare(a.hi, b.hi)
		}
	})
	if steps[0].lo != lo || steps[len(steps)-1].hi != hi {
		return nil, false
	}
	above := steps[0].rim > axisAt
	for i := range steps {
		s := &steps[i]
		if (s.rim > axisAt) != above {
			return nil, false
		}
		if i > 0 && (steps[i-1].hi != s.lo || steps[i-1].rim == s.rim) {
			return nil, false
		}
		radius := s.rim - axisAt
		if radius < 0 {
			radius = -radius
		}
		exact := new(big.Rat).Sub(new(big.Rat).SetFloat64(s.rim), new(big.Rat).SetFloat64(axisAt))
		if exact.Abs(exact).Cmp(new(big.Rat).SetFloat64(radius)) != 0 {
			return nil, false
		}
		s.radius = radius
	}
	return steps, true
}
