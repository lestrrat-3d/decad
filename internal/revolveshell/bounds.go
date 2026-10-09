package revolveshell

import (
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// AxisCurve is the revolve axis as the held line the offset's mirror
// joins meet (offset2d.OffsetOpenChain).
func AxisCurve(ax revolveaxis.Frame) offset2d.Curve {
	return offset2d.Curve{IsLine: true, PX: ax.AU, PY: ax.AV, DX: ax.DU, DY: ax.DV}
}

// ChainSectionDelta is offsetSectionDelta for the open offset chain of a
// revolve shell's wall (docs/modify-reach-design.md §9.3.1): a proven upper
// bound on how far any boundary point of the recorded wall sits from the wall
// the shell denotes, three times offset2d.ChainReach's largest reach on
// offsetSectionDelta's own argument. The denoted axis is the receiver's axis
// line widened by axisInPlane's four proven bounds, so an axis end's mirror
// join is enclosed against every line the record allows. The kept chain K and
// the axis points it leaves from are the receiver's own record and move by
// nothing, so the figure is exactly zero wherever every join encloses to the
// float the build holds, which keeps a right-angle shell Exact.
func ChainSectionDelta(budget *proofbound.WorkBudget, chain []survey2d.SideWalk, ax revolveaxis.Frame, ends [2]offset2d.ChainEnd, s, t, tDelta, tol float64) (float64, error) {
	if slices.ContainsFunc([]float64{ax.AU, ax.AV, ax.DU, ax.DV, ax.AUBound, ax.AVBound, ax.DUBound, ax.DVBound}, proofbound.IsNonFinite) {
		return 0, offset2d.ErrUnbounded
	}
	widen := func(x, b float64) proofbound.RatInterval {
		return proofbound.IntervalWiden(proofbound.PointInterval(proofarith.FloatRat(x)), proofarith.FloatRat(math.Abs(b)))
	}
	line := offset2d.MirrorLine{
		Held: AxisCurve(ax),
		Enclosure: capcontour.Carrier{
			IsLine: true,
			P:      capcontour.Point{U: widen(ax.AU, ax.AUBound), V: widen(ax.AV, ax.AVBound)},
			Dir:    capcontour.Point{U: widen(ax.DU, ax.DUBound), V: widen(ax.DV, ax.DVBound)},
		},
	}
	return offset2d.ChainSectionDelta(budget, chain, line, ends, s, t, tDelta, tol)
}
