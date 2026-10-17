package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func loftCylinderCanonicalFrame(z float64) (r3.Frame, error) {
	return r3.NewFrame(r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
}

func loftCylinderFrameMatchesPlane(frame r3.Frame, plane planeRecord) bool {
	return frame.Origin() == plane.Origin && frame.U() == plane.U && frame.V() == plane.V
}

// These cheap source-shape checks run before a private reconstruction. A
// mismatch belongs to the ordinary Boolean; a failure after reconstruction
// starts is a proof or work-limit error and remains visible to the caller.
func loftCylinderPlacedBlankShape(pp prismPayload) bool {
	if pp.SurfaceResult || pp.Z0 != 0 || pp.Z1 <= 0 || proofbound.IsNonFinite(pp.Z1) ||
		pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 {
		return false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(circleSeg)
	return ok && circle.Center == (planePoint{}) && circle.CCW &&
		circle.Radius.Base() > 0 && !proofbound.IsNonFinite(circle.Radius.Base())
}

func loftCylinderPlacedToothShape(body *Body) bool {
	lp, ok := body.n.Payload.(loftPayload)
	return ok && lp.AuthenticatedReconstruction && !lp.SurfaceResult && lp.SourceIdentity != nil &&
		len(lp.Profile0.Holes) == 0 && len(lp.Profile1.Holes) == 0 &&
		len(lp.Profile0.Outer.Segments) == 6 && len(lp.Profile1.Outer.Segments) == 6 &&
		len(lp.Segment) == 1 && len(lp.Verts)%2 == 0 && len(lp.Verts) >= 16 &&
		loftCylinderSectionKinds(lp)
}

// A continuation can only reuse the prior local certificate for the same
// authenticated Loft source. Check this before rebuilding the next tooth so
// an unrelated Loft keeps the ordinary Boolean's error and admission policy.
func loftCylinderPlacedSourcesMatch(teeth []*Body) bool {
	return loftcylinder.SourceIdentitiesMatch(len(teeth), func(i int) topo.Payload { return teeth[i].n.Payload })
}

// loftCylinderPlacedNodeSourcesMatch is loftCylinderPlacedSourcesMatch over
// the body nodes a join record stores.
func loftCylinderPlacedNodeSourcesMatch(teeth []*topo.Body) bool {
	return loftcylinder.SourceIdentitiesMatch(len(teeth), func(i int) topo.Payload { return teeth[i].Payload })
}

// loftCylinderLocalBlank rebuilds the exact circular source in XY. The two
// source maps must agree as rational affine maps before its proof is used.
func loftCylinderLocalBlank(ctx context.Context, d *Document, blank *Body) (*Body, r3.Transform, bool, error) {
	pp, ok := blank.n.Payload.(prismPayload)
	if !ok || !loftCylinderPlacedBlankShape(pp) {
		return nil, r3.Transform{}, false, nil
	}
	frame, err := r3.FromFrame(pp.Frame)
	if err != nil {
		return nil, r3.Transform{}, false, err
	}
	pose, err := frame.Then(pp.Xform)
	if err != nil || pose == r3.Identity() || !loftcylinder.SignedPose(pose) ||
		!loftcylinder.AffineOf(frame).Then(loftcylinder.AffineOf(pp.Xform)).Equals(loftcylinder.AffineOf(pose)) {
		return nil, r3.Transform{}, false, nil //nolint:nilerr // an invalid pose cannot use this certificate
	}
	localFrame, err := loftCylinderCanonicalFrame(0)
	if err != nil {
		return nil, r3.Transform{}, false, err
	}
	local := pp
	local.Frame, local.Xform = localFrame, r3.Identity()
	localBody, err := placePayload(ctx, local, d, blank.n.Origin.Producer, r3.Identity())
	return localBody, pose, err == nil, err
}

// loftCylinderLocalTooth accepts only a source whose two recorded section
// planes are exact images of one axial local Loft under pose. The local copy
// keeps the original producer ID and authenticated source identity.
func loftCylinderLocalTooth(ctx context.Context, d *Document, tooth *Body,
	pose r3.Transform, z1 float64) (*Body, bool, error) {
	lp, ok := tooth.n.Payload.(loftPayload)
	if !ok || !lp.AuthenticatedReconstruction || lp.SourceIdentity == nil ||
		!loftCylinderFrameMatchesPlane(lp.Frame0, lp.Plane0) ||
		!loftCylinderFrameMatchesPlane(lp.Frame1, lp.Plane1) {
		return nil, false, nil
	}
	first, err := r3.FromFrame(lp.Frame0)
	if err != nil {
		return nil, false, err
	}
	last, err := r3.FromFrame(lp.Frame1)
	if err != nil {
		return nil, false, err
	}
	inverse, err := pose.Inverse()
	if err != nil {
		return nil, false, nil //nolint:nilerr // an inverse without a valid map cannot certify this tooth
	}
	firstWorld, err := first.Then(lp.Xform)
	if err != nil {
		return nil, false, nil //nolint:nilerr // an invalid source map keeps the generic fallback
	}
	localXform, err := firstWorld.Then(inverse)
	if err != nil || !loftCylinderAxialPlacement(localXform) {
		return nil, false, nil //nolint:nilerr // an invalid local map cannot certify axial placement
	}
	local0, err := loftCylinderCanonicalFrame(0)
	if err != nil {
		return nil, false, err
	}
	local1, err := loftCylinderCanonicalFrame(z1)
	if err != nil {
		return nil, false, err
	}
	localFirst, err := r3.FromFrame(local0)
	if err != nil {
		return nil, false, err
	}
	localLast, err := r3.FromFrame(local1)
	if err != nil {
		return nil, false, err
	}
	actual0 := loftcylinder.AffineOf(first).Then(loftcylinder.AffineOf(lp.Xform))
	actual1 := loftcylinder.AffineOf(last).Then(loftcylinder.AffineOf(lp.Xform))
	want0 := loftcylinder.AffineOf(localFirst).Then(loftcylinder.AffineOf(localXform)).
		Then(loftcylinder.AffineOf(pose))
	want1 := loftcylinder.AffineOf(localLast).Then(loftcylinder.AffineOf(localXform)).
		Then(loftcylinder.AffineOf(pose))
	if !actual0.Equals(want0) || !actual1.Equals(want1) {
		return nil, false, nil
	}
	local := lp
	local.Plane0 = planeRecord{Origin: local0.Origin(), U: local0.U(), V: local0.V()}
	local.Plane1 = planeRecord{Origin: local1.Origin(), U: local1.U(), V: local1.V()}
	local.Frame0, local.Frame1, local.Xform = local0, local1, localXform
	localBody, err := placePayload(ctx, local, d, tooth.n.Origin.Producer, localXform)
	return localBody, err == nil, err
}

func tryPlacedLoftCylinderUnion(ctx context.Context, d *Document, ref producerID,
	a, b *Body) (*Body, bool, error) {
	var blank *Body
	var teeth []*Body
	var localBlank *Body
	var localTeeth []*Body
	var pose r3.Transform
	if _, ok := a.n.Payload.(prismPayload); ok {
		blank, teeth = a, []*Body{b}
	} else if _, ok := b.n.Payload.(prismPayload); ok {
		blank, teeth = b, []*Body{a}
	} else if fp, ok := a.n.Payload.(facetedPayload); ok && fp.LoftCylinderJoin != nil &&
		fp.LoftCylinderJoin.Placement != nil && fp.Xform == fp.LoftCylinderJoin.Placement.Pose {
		placement := fp.LoftCylinderJoin.Placement
		blank, teeth, pose = bodyHandle(fp.LoftCylinderJoin.Blank),
			append(bodyHandlesCopy(fp.LoftCylinderJoin.Teeth), b), placement.Pose
		localBlank = bodyHandle(placement.Blank)
		localTeeth = bodyHandlesCopy(placement.Teeth)
	} else if fp, ok := b.n.Payload.(facetedPayload); ok && fp.LoftCylinderJoin != nil &&
		fp.LoftCylinderJoin.Placement != nil && fp.Xform == fp.LoftCylinderJoin.Placement.Pose {
		placement := fp.LoftCylinderJoin.Placement
		blank, teeth, pose = bodyHandle(fp.LoftCylinderJoin.Blank),
			append(bodyHandlesCopy(fp.LoftCylinderJoin.Teeth), a), placement.Pose
		localBlank = bodyHandle(placement.Blank)
		localTeeth = bodyHandlesCopy(placement.Teeth)
	} else {
		return nil, false, nil
	}
	if !loftCylinderPlacedToothShape(teeth[len(teeth)-1]) {
		return nil, false, nil
	}
	if localBlank != nil && !loftCylinderPlacedSourcesMatch(teeth) {
		return nil, false, nil
	}
	if localBlank == nil {
		var ok bool
		var err error
		localBlank, pose, ok, err = loftCylinderLocalBlank(ctx, d, blank)
		if err != nil || !ok {
			return nil, false, err
		}
	}
	pp, ok := localBlank.n.Payload.(prismPayload)
	if !ok {
		return nil, false, nil
	}
	localTooth, ok, err := loftCylinderLocalTooth(ctx, d, teeth[len(teeth)-1], pose, pp.Z1)
	if err != nil || !ok {
		return nil, false, err
	}
	localTeeth = append(localTeeth, localTooth)
	localJoin, admitted, err := tryLoftCylinderUnionTeeth(ctx, d, ref, localBlank, localTeeth)
	if err != nil || !admitted {
		return nil, admitted, err
	}
	localPayload, ok := localJoin.n.Payload.(facetedPayload)
	if !ok || localPayload.LoftCylinderJoin == nil {
		return nil, false, nil
	}
	world, err := bodyResult(localPayload.PlacedWith(ctx, envOf(documentNode(d), ref), pose, nil))
	if err != nil {
		return nil, true, err
	}
	placed, ok := world.n.Payload.(facetedPayload)
	if !ok || placed.MeshBound > 0.1 || proofbound.IsNonFinite(placed.MeshBound) {
		return nil, false, nil
	}
	record := *localPayload.LoftCylinderJoin
	record.Blank = bodyNode(blank)
	record.Teeth = bodyNodes(teeth)
	record.Placement = &loftCylinderPlacement{Pose: pose, Blank: bodyNode(localBlank), Teeth: bodyNodes(localTeeth)}
	placed.LoftCylinderJoin = &record
	world.n.Payload = placed
	return world, true, nil
}

func tryPlacedLoftCylinderBoreCut(ctx context.Context, d *Document, ref producerID,
	target, tool *Body) (*Body, bool, error) {
	joined, ok := target.n.Payload.(facetedPayload)
	if !ok || joined.LoftCylinderJoin == nil || joined.LoftCylinderJoin.Placement == nil {
		return nil, false, nil
	}
	placement := joined.LoftCylinderJoin.Placement
	if joined.Xform != placement.Pose || !joined.ContactAudited ||
		placement.Blank == nil || len(placement.Teeth) == 0 ||
		!loftCylinderPlacedNodeSourcesMatch(placement.Teeth) {
		return nil, false, nil
	}
	record := joined.LoftCylinderJoin
	// Mirror the local bore's source-level gates before rebuilding the tool.
	// A mismatch belongs to generic Cut; a rebuild error remains observable.
	pp, ok := tool.n.Payload.(prismPayload)
	if !ok || pp.SurfaceResult || pp.SectionDelta != 0 || pp.Z0Delta != 0 ||
		pp.Z1Delta != 0 || len(pp.Profile.Holes) != 0 ||
		len(pp.Profile.Outer.Segments) != 1 {
		return nil, false, nil
	}
	circle, ok := pp.Profile.Outer.Segments[0].(circleSeg)
	if !ok || circle.Center != (planePoint{}) ||
		!((circle.TStart == 0 && circle.TEnd == 1) ||
			(circle.TStart == 1 && circle.TEnd == 0)) {
		return nil, false, nil
	}
	radius, err := circle.Radius.In(units.Millimeter)
	if err != nil {
		return nil, false, err
	}
	if radius <= 0 || radius >= record.Radius || proofbound.IsNonFinite(radius) ||
		proofbound.IsNonFinite(pp.Z0) || proofbound.IsNonFinite(pp.Z1) {
		return nil, false, nil
	}
	frame, err := r3.FromFrame(pp.Frame)
	if err != nil {
		return nil, false, err
	}
	worldFrame, err := frame.Then(pp.Xform)
	if err != nil {
		return nil, false, nil //nolint:nilerr // an invalid tool map keeps the generic fallback
	}
	inverse, err := placement.Pose.Inverse()
	if err != nil {
		return nil, false, nil //nolint:nilerr // an invalid pose inverse cannot certify this bore
	}
	localFrameMap, err := worldFrame.Then(inverse)
	if err != nil || localFrameMap.Basis() != r3.Identity().Basis() ||
		localFrameMap.Translation().X != 0 || localFrameMap.Translation().Y != 0 {
		return nil, false, nil //nolint:nilerr // an invalid local tool map keeps the generic fallback
	}
	localFrame, err := loftCylinderCanonicalFrame(localFrameMap.Translation().Z)
	if err != nil {
		return nil, false, err
	}
	localMap, err := r3.FromFrame(localFrame)
	if err != nil {
		return nil, false, err
	}
	if !loftcylinder.AffineOf(frame).Then(loftcylinder.AffineOf(pp.Xform)).Equals(
		loftcylinder.AffineOf(localMap).Then(loftcylinder.AffineOf(placement.Pose))) {
		return nil, false, nil
	}
	origin := localFrame.Origin()
	if !proofbound.FiniteVec(origin) {
		return nil, false, nil
	}
	start := new(big.Rat).Add(proofarith.FloatRat(origin.Z), proofarith.FloatRat(pp.Z0))
	end := new(big.Rat).Add(proofarith.FloatRat(origin.Z), proofarith.FloatRat(pp.Z1))
	if start.Cmp(proofarith.FloatRat(record.Z0)) >= 0 ||
		end.Cmp(proofarith.FloatRat(record.Z1)) <= 0 {
		return nil, false, nil
	}
	// The source and tool gates above precede both private rebuilds. The join
	// is only needed for an eligible bore; later tooth joins use the sources.
	localJoin, admitted, err := tryLoftCylinderUnionTeeth(ctx, d,
		target.originProducer(), bodyHandle(placement.Blank), bodyHandles(placement.Teeth))
	if err != nil || !admitted {
		return nil, admitted, err
	}
	local := pp
	local.Frame, local.Xform = localFrame, r3.Identity()
	localTool, err := placePayload(ctx, local, d, tool.n.Origin.Producer, r3.Identity())
	if err != nil {
		return nil, false, err
	}
	localBore, admitted, err := tryLoftCylinderBoreCut(ctx, meshbool.OpCut,
		d, ref, localJoin, localTool)
	if err != nil || !admitted {
		return nil, admitted, err
	}
	localPayload, ok := localBore.n.Payload.(facetedPayload)
	if !ok {
		return nil, false, nil
	}
	world, err := bodyResult(localPayload.PlacedWith(ctx, envOf(documentNode(d), ref), placement.Pose, nil))
	if err != nil {
		return nil, true, err
	}
	placed, ok := world.n.Payload.(facetedPayload)
	if !ok || placed.MeshBound > 0.1 || proofbound.IsNonFinite(placed.MeshBound) {
		return nil, false, nil
	}
	return world, true, nil
}
