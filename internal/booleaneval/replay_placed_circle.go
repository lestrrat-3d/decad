package booleaneval

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func replayPlacedCircleDepartures(ctx context.Context, body *topo.Body, mesh *topo.Mesh, faces []*topo.Face,
	b *proofbound.ReplayBudget) (placedDepartures, error) {
	if err := ctx.Err(); err != nil {
		return placedDepartures{}, err
	}
	if body == nil || mesh == nil || b == nil || !b.IsBrakeReplay() {
		return placedDepartures{}, errBrakeEvalBoundedCircleTagNeedsABrakeOperand
	}
	pp, ok := body.Payload.(prismrecord.Payload)
	if !ok || pp.SurfaceResult || !pairbox.SignedAxisTransform(pp.Xform) || len(faces) != 3 ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 || pp.Walks == nil ||
		!pp.Walks.Metered || pp.Walks.Spent != 0 || pp.Walks.ReconstructionSpent != 0 ||
		len(pp.Walks.Outer) != 1 || len(pp.Walks.Holes) != 0 || len(pp.Walks.Profile.Holes) != 0 ||
		len(pp.Walks.Profile.Outer.Segments) != 1 {
		return placedDepartures{}, errBrakeEvalBoundedCircleTagNeedsOneCircularSolid
	}
	seg, ok := pp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !seg.CCW || seg.TStart != 0 || seg.TEnd != 1 {
		return placedDepartures{}, errBrakeEvalBoundedCircleTagNeedsAWholeCircle
	}
	if _, ok := pp.Walks.Profile.Outer.Segments[0].(sectionrecord.CircleSeg); !ok {
		return placedDepartures{}, errBrakeEvalBoundedCircleWalkChanged
	}
	if err := b.SpendWork(ctx, 512); err != nil {
		return placedDepartures{}, err
	}
	if err := b.ReserveRecords(ctx, 32); err != nil {
		return placedDepartures{}, err
	}
	if err := b.ReserveBytes(ctx, 8192); err != nil {
		return placedDepartures{}, err
	}
	if !pp.Walks.Reusable(pp.Profile) {
		return placedDepartures{}, errBrakeEvalBoundedCircleRecordChanged
	}
	w := pp.Walks.Outer[0]
	radius, err := seg.Radius.In(units.Millimeter)
	if err != nil || !(radius > 0) || math.IsInf(radius, 0) || w.Kind != survey2d.WalkCircular || !w.Closed ||
		len(w.Spans) != 0 || w.CU != seg.Center.U || w.CV != seg.Center.V || w.Radius != radius ||
		w.Th0 != 0 || w.Th1 != 2*math.Pi {
		return placedDepartures{}, errBrakeEvalBoundedCircleWalkDoesNotMatchItsRecord
	}
	roles := [3]string{"side(0,0)", provenance.RoleCapStart, provenance.RoleCapEnd}
	for i, f := range faces {
		if f == nil || f.Body != body || len(f.Origins) != 1 || f.Origins[0].Role != roles[i] ||
			f.Origins[0].Producer != body.Origin.Producer || (i == 0 && f.Reversed != (w.Th1 < w.Th0)) {
			return placedDepartures{}, errBrakeEvalBoundedCircleSourceFaceOrderChanged
		}
	}
	beta, err := topobuild.VertexBoundsReplay(ctx, mesh, b)
	if err != nil {
		return placedDepartures{}, err
	}
	if err := b.SpendWork(ctx, uint64(len(mesh.Vertices))); err != nil {
		return placedDepartures{}, err
	}
	if len(mesh.Vertices) > 65536 || !(mesh.Bound >= 0) || mesh.Bound > 1e100 {
		return placedDepartures{}, errBrakeEvalBoundedCircleExtent
	}
	for i, v := range mesh.Vertices {
		if !proofbound.FiniteVec(v) || math.Abs(v.X) > 1e100 || math.Abs(v.Y) > 1e100 || math.Abs(v.Z) > 1e100 ||
			!(beta[i] >= 0) || beta[i] > 1e100 {
			return placedDepartures{}, errBrakeEvalBoundedCircleExtent
		}
	}
	for _, size := range []uint64{uint64(unsafe.Sizeof(float64(0))), uint64(unsafe.Sizeof(false))} {
		if err := b.ReserveRecords(ctx, 3); err != nil {
			return placedDepartures{}, err
		}
		if err := b.ReserveArray(ctx, 3, size); err != nil {
			return placedDepartures{}, err
		}
	}
	out := placedDepartures{departure: make([]float64, 3), stated: make([]bool, 3)}
	source := pp
	source.Xform = r3.Identity()
	unplaced := surfacegeom.Cylinder{Origin: source.Point(w.CU, w.CV, source.Z0), Axis: source.Dir(0, 0, 1), Radius: units.Millimeters(w.Radius)}
	expected := surfacegeom.Cylinder{Origin: pp.Point(w.CU, w.CV, pp.Z0), Axis: pp.Dir(0, 0, 1), Radius: units.Millimeters(w.Radius)}
	moved, ok := faces[0].Surface.(surfacegeom.Cylinder)
	if !ok || moved != expected {
		return placedDepartures{}, errBrakeEvalBoundedCircleCylinderTagChanged
	}
	if !proofbound.FiniteVec(moved.Origin) || math.Abs(moved.Origin.X) > 1e100 ||
		math.Abs(moved.Origin.Y) > 1e100 || math.Abs(moved.Origin.Z) > 1e100 {
		return placedDepartures{}, errBrakeEvalBoundedCircleOriginExtent
	}
	if facetproof.TranslationOnly(pp.Xform) {
		out.departure[0], err = facetedrecord.ReplayPlacedBuiltCylinderDeparture(ctx, unplaced, moved, pp.Xform, b)
	} else {
		out.departure[0], err = facetedrecord.ReplaySignedBuiltCylinderDeparture(ctx, unplaced, moved, pp.Xform, b)
	}
	if err != nil {
		return placedDepartures{}, err
	}
	out.stated[0] = true
	for i, capItem := range []struct {
		z    float64
		flip bool
	}{{pp.Z0, true}, {pp.Z1, false}} {
		originalFrame, err := source.CapFrame(capItem.z, capItem.flip)
		if err != nil {
			return placedDepartures{}, err
		}
		expectedFrame, err := pp.CapFrame(capItem.z, capItem.flip)
		if err != nil {
			return placedDepartures{}, err
		}
		plane, ok := faces[i+1].Surface.(surfacegeom.Plane)
		if !ok || plane.Frame != expectedFrame {
			return placedDepartures{}, errBrakeEvalBoundedCircleCapTagChanged
		}
		if facetproof.TranslationOnly(pp.Xform) {
			out.departure[i+1], err = facetedrecord.ReplayPlacedBuiltPlaneDeparture(ctx, surfacegeom.Plane{Frame: originalFrame}, plane, pp.Xform, b)
		} else {
			out.departure[i+1], err = facetedrecord.ReplaySignedBuiltPlaneDeparture(ctx, surfacegeom.Plane{Frame: originalFrame}, plane, pp.Xform, b)
		}
		if err != nil {
			return placedDepartures{}, err
		}
		out.stated[i+1] = true
	}
	return out, nil
}
