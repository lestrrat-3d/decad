package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// tryPlacedLoftCylinderUnion joins a common-placement cylinder and Loft
// tooth, or a placed join and its next tooth, in local coordinates and places
// the audited shell back under the pose. It stays in root because the local
// join is certified by tryLoftCylinderUnionTeeth, which verifies meshes
// through root Body.Tessellate.
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
	if !loftcylinder.PlacedToothShape(bodyNode(teeth[len(teeth)-1])) {
		return nil, false, nil
	}
	if localBlank != nil && !loftcylinder.SourceIdentitiesMatch(bodyNodes(teeth)) {
		return nil, false, nil
	}
	if localBlank == nil {
		node, localPose, ok, err := loftcylinder.LocalBlank(ctx, envOf(documentNode(d), ref), bodyNode(blank))
		if err != nil || !ok {
			return nil, false, err
		}
		localBlank, pose = bodyHandle(node), localPose
	}
	pp, ok := localBlank.n.Payload.(prismPayload)
	if !ok {
		return nil, false, nil
	}
	localTooth, ok, err := loftcylinder.LocalTooth(ctx, envOf(documentNode(d), ref),
		bodyNode(teeth[len(teeth)-1]), pose, pp.Z1)
	if err != nil || !ok {
		return nil, false, err
	}
	localTeeth = append(localTeeth, bodyHandle(localTooth))
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

// tryPlacedLoftCylinderBoreCut cuts a centered through-bore from a
// common-placement join in local coordinates and places the result back
// under the pose. It stays in root because it rebuilds the local join through
// tryLoftCylinderUnionTeeth, which verifies meshes through root
// Body.Tessellate.
func tryPlacedLoftCylinderBoreCut(ctx context.Context, d *Document, ref producerID,
	target, tool *Body) (*Body, bool, error) {
	joined, ok := target.n.Payload.(facetedPayload)
	if !ok || joined.LoftCylinderJoin == nil || joined.LoftCylinderJoin.Placement == nil {
		return nil, false, nil
	}
	placement := joined.LoftCylinderJoin.Placement
	if joined.Xform != placement.Pose || !joined.ContactAudited ||
		placement.Blank == nil || len(placement.Teeth) == 0 ||
		!loftcylinder.SourceIdentitiesMatch(placement.Teeth) {
		return nil, false, nil
	}
	record := joined.LoftCylinderJoin
	// Run the local bore's tool-profile and through-span gates before
	// rebuilding the tool. A mismatch belongs to generic Cut; a rebuild error
	// remains observable.
	pp, ok := tool.n.Payload.(prismPayload)
	if !ok {
		return nil, false, nil
	}
	if _, ok, err := loftcylinder.BoreTool(pp, record.Radius); err != nil || !ok {
		return nil, false, err
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
	localFrame, err := loftcylinder.CanonicalFrame(localFrameMap.Translation().Z)
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
	if !proofbound.FiniteVec(origin) || !loftcylinder.BoreThrough(origin.Z, pp.Z0, pp.Z1, record.Z0, record.Z1) {
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
