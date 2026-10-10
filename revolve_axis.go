package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// This file resolves a revolve's axis into the sketch plane and decides what
// the profile may do around it: revolveaxis.Line2, the 2D line the axis projects to;
// axisFrame, the axis-local frame every later reading is taken in; and the
// two gates that refuse a profile crossing or touching the axis where the
// sweep would be degenerate.
//
// wallKind is decided here because it follows from the axis alone: a segment
// parallel to the axis sweeps a cylinder, one meeting it at an angle a cone,
// one perpendicular an annulus. A segment whose kind the axis cannot decide
// exactly refuses rather than being assigned the nearest one. See
// docs/evaluator-design.md §6.

// axisInPlane reads the public axis variant and resolves its coordinates.
func axisInPlane(a Axis, frame r3.Frame) (revolveaxis.Line2, error) {
	var input revolveaxis.AxisInput
	switch value := a.(type) {
	case SketchLine:
		input = revolveaxis.SketchLine{
			StartU: value.Start.U, StartV: value.Start.V,
			EndU: value.End.U, EndV: value.End.V,
		}
	case ConstructionAxis:
		input = revolveaxis.ConstructionAxis{Origin: value.Origin, Dir: value.Dir}
	default:
		return revolveaxis.Line2{}, fmt.Errorf(`%w: axis %T is not supported by this evaluator`, ErrUnsupported, a)
	}
	return revolveaxis.AxisInPlane(input, frame)
}

// axisFrame is the revolve axis as a proper plane-local frame with the
// region on its non-negative side: z = (p−a)·d runs along the axis and
// ρ = cross(d, p−a) ≥ 0 is the radial coordinate. snapTol is the
// scale-relative tolerance that classified axis contact; a coordinate
// within it of the axis IS on the axis.
//
// radialAdmitAllow and axialExtentUpper are resolveAxisSide's own charge for
// admitting a region under UNCERTAINTY rather than proof: they are zero
// whenever resolveAxisSide proved the region's radial minimum non-negative —
// which is every axis-aligned fixture in the tree, since the arithmetic is
// then exact and admits nothing it has not proven — and otherwise they carry
// the worst case a genuine straddle leaves open. radialAdmitAllow bounds how
// far below zero the TRUE radial minimum can sit despite being admitted, and
// axialExtentUpper bounds the recorded region's own axial reach; together they
// are what revolve_build.go charges into the published volume, cap area and
// centroid bounds (revolvePayload.ax's own doc comment there), since the
// admitted region's faces are built from the SNAPPED profile while the
// integrals behind those measurements read the UNSNAPPED one. Both are the
// zero value for any axisFrame not built by resolveAxisSide (a full-sweep
// composite payload's own literal), which is the safe default: no admitted
// uncertainty, no charge.
//
// snap is the SNAP's own share of that same mismatch, and it answers a
// displacement decad COMMITS rather than one it admits without proof: wherever
// axisFrame.walk assigns an endpoint exactly 0 that the arithmetic put a
// positive distance out, the region whose boundary the built faces follow is no
// longer the recorded one, and every measurement integrated over the recorded
// one owes the difference. Its four fields bound how far the region's own area
// and its three axis-frame moments can move, and revolve_build.go adds each to
// the reading it belongs to. Every one of them is exactly zero for a profile
// whose on-axis endpoints already sit on the axis, which is every axis-incident
// fixture in the tree.
type axisFrame struct {
	aU, aV           float64
	aUBound, aVBound float64
	dU, dV           float64
	dUBound, dVBound float64
	snapTol          float64
	radialAdmitAllow float64
	// radialProof is a strict zero-threshold proof from this profile's own
	// build scan. Only a payload retaining that profile and axis may reuse it.
	radialProof      bool
	axialExtentUpper float64
	snap             regionSnapAllow
}

// numeric returns the axis values used to re-express one boundary walk.
func (ax axisFrame) numeric() revolveaxis.Frame {
	return revolveaxis.Frame{
		AU: ax.aU, AV: ax.aV, AUBound: ax.aUBound, AVBound: ax.aVBound,
		DU: ax.dU, DV: ax.dV, DUBound: ax.dUBound, DVBound: ax.dVBound,
		SnapTol: ax.snapTol,
	}
}

func (ax axisFrame) toAxisRhoBound(u, v float64) float64 {
	return ax.numeric().ToAxisRhoBound(u, v)
}

func (ax axisFrame) radialUpper(coordUpper float64) float64 {
	return ax.numeric().RadialUpper(coordUpper)
}

func (ax axisFrame) walk(w survey2d.SegmentWalk) survey2d.SegmentWalk {
	return ax.numeric().Walk(w)
}

func (ax axisFrame) walkCharged(
	w survey2d.SegmentWalk, startCharge, endCharge proofbound.WalkEndBound,
) survey2d.SegmentWalk {
	return ax.numeric().WalkCharged(w, startCharge, endCharge)
}

// wallKind classifies what one boundary walk sweeps.
type wallKind = revolveaxis.WallKind

const (
	// wallAxis is a line lying along the axis: it sweeps a zero-area set
	// and emits no face — the neighboring segments' faces close the solid.
	wallAxis = revolveaxis.WallAxis
	// wallCylinder is a line parallel to the axis.
	wallCylinder = revolveaxis.WallCylinder
	// wallPlane is a line perpendicular to the axis: a planar annulus, or a
	// disk when it reaches the axis.
	wallPlane = revolveaxis.WallPlane
	// wallCone is an inclined line; an endpoint on the axis is its apex.
	wallCone = revolveaxis.WallCone
	// wallSphere is a circular walk whose center lies on the axis.
	wallSphere = revolveaxis.WallSphere
	// wallTorus is a circular walk whose center lies off the axis.
	wallTorus = revolveaxis.WallTorus
)

// classify names the surface of revolution one axis-coordinate walk sweeps.
func (ax axisFrame) classify(w survey2d.SegmentWalk) wallKind {
	return revolveaxis.Classify(w, ax.snapTol)
}

// IsAxis reports whether a meridian walk sweeps no face.
func (ax axisFrame) IsAxis(w survey2d.SegmentWalk) bool {
	return ax.classify(w) == wallAxis
}

// resolveAxisSide scans the record, asks revolveaxis to decide its side, then
// audits each walk for axis contact and charges any snapped endpoint.
func resolveAxisSide(ctx context.Context, profile profileRecord, line revolveaxis.Line2, work *freeform.FreeformWork) (axisFrame, float64, error) {
	nU, nV := -line.DV, line.DU
	rlo, rhi, rBound, err := boundaryExtremesBoundedContext(ctx, profile, nU, nV, work, nil)
	if err != nil {
		return axisFrame{}, 0, err
	}
	zlo, zhi, zBound, err := boundaryExtremesBoundedContext(ctx, profile, line.DU, line.DV, work, nil)
	if err != nil {
		return axisFrame{}, 0, err
	}

	// The side gate charges the scan's dot-product rounding separately from
	// the positional bounds returned with the extremes.
	coordUpper, err := momentinput.CoordinateEnvelope(profile, work, nil)
	if err != nil {
		return axisFrame{}, 0, err
	}
	resolved, err := revolveaxis.ResolveSide(line, revolveaxis.SideExtremes{
		RLo: rlo, RHi: rhi, RBound: rBound,
		ZLo: zlo, ZHi: zhi, ZBound: zBound,
		CoordUpper: coordUpper,
	})
	if err != nil {
		return axisFrame{}, 0, err
	}
	frame := resolved.Frame
	ax := axisFrame{
		aU: frame.AU, aV: frame.AV, aUBound: frame.AUBound, aVBound: frame.AVBound,
		dU: frame.DU, dV: frame.DV, dUBound: frame.DUBound, dVBound: frame.DVBound,
		snapTol:          frame.SnapTol,
		radialAdmitAllow: resolved.RadialAdmitAllow,
		radialProof:      resolved.RadialProof,
		axialExtentUpper: resolved.AxialExtentUpper,
	}
	snap, err := ax.auditAxisContact(profile, work)
	if err != nil {
		return axisFrame{}, 0, err
	}
	ax.snap = snap
	return ax, resolved.Side, nil
}

// regionSnapAllow keeps the root payload's four bounded integrals. The
// axis-contact audit and arithmetic live in internal/revolveaxis.
type regionSnapAllow struct {
	area, first, mixed, second float64
}

// auditAxisContact keeps record walking at the root while revolveaxis checks
// each resolved walk in the original order and charges its snap allowance.
func (ax axisFrame) auditAxisContact(profile profileRecord, work *freeform.FreeformWork) (regionSnapAllow, error) {
	loops := append([]loopRecord{profile.Outer}, profile.Holes...)
	snap, err := revolveaxis.AuditAxisContact(ax.numeric(), loops, func(seg curveSegment) (survey2d.SegmentWalk, error) {
		w, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return survey2d.SegmentWalk{}, err
		}
		if err := boundarywalk.RequireAnalyticWalk(w, "the revolve axis-contact audit"); err != nil {
			return survey2d.SegmentWalk{}, err
		}
		return w, nil
	})
	if err != nil {
		return regionSnapAllow{}, err
	}
	return regionSnapAllow{area: snap.Area, first: snap.First, mixed: snap.Mixed, second: snap.Second}, nil
}
