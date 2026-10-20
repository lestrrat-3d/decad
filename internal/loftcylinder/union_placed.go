package loftcylinder

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/verifiedmesh"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// PlacedUnion joins a common-placement cylinder and Loft
// tooth, or a placed join and its next tooth, in local coordinates and places
// the audited shell back under the pose. The local join is certified by
// UnionTeeth.
func PlacedUnion(ctx context.Context, env topo.Env, mesher verifiedmesh.Mesher,
	a, b *topo.Body) (*topo.Body, bool, error) {
	var blank *topo.Body
	var teeth []*topo.Body
	var localBlank *topo.Body
	var localTeeth []*topo.Body
	var pose r3.Transform
	if _, ok := a.Payload.(prismrecord.Payload); ok {
		blank, teeth = a, []*topo.Body{b}
	} else if _, ok := b.Payload.(prismrecord.Payload); ok {
		blank, teeth = b, []*topo.Body{a}
	} else if fp, ok := a.Payload.(facetedrecord.Payload); ok && fp.LoftCylinderJoin != nil &&
		fp.LoftCylinderJoin.Placement != nil && fp.Xform == fp.LoftCylinderJoin.Placement.Pose {
		placement := fp.LoftCylinderJoin.Placement
		blank, teeth, pose = fp.LoftCylinderJoin.Blank,
			append(slices.Clone(fp.LoftCylinderJoin.Teeth), b), placement.Pose
		localBlank = placement.Blank
		localTeeth = slices.Clone(placement.Teeth)
	} else if fp, ok := b.Payload.(facetedrecord.Payload); ok && fp.LoftCylinderJoin != nil &&
		fp.LoftCylinderJoin.Placement != nil && fp.Xform == fp.LoftCylinderJoin.Placement.Pose {
		placement := fp.LoftCylinderJoin.Placement
		blank, teeth, pose = fp.LoftCylinderJoin.Blank,
			append(slices.Clone(fp.LoftCylinderJoin.Teeth), a), placement.Pose
		localBlank = placement.Blank
		localTeeth = slices.Clone(placement.Teeth)
	} else {
		return nil, false, nil
	}
	if !PlacedToothShape(teeth[len(teeth)-1]) {
		return nil, false, nil
	}
	if localBlank != nil && !SourceIdentitiesMatch(teeth) {
		return nil, false, nil
	}
	if localBlank == nil {
		node, localPose, ok, err := LocalBlank(ctx, env, blank)
		if err != nil || !ok {
			return nil, false, err
		}
		localBlank, pose = node, localPose
	}
	pp, ok := localBlank.Payload.(prismrecord.Payload)
	if !ok {
		return nil, false, nil
	}
	localTooth, ok, err := LocalTooth(ctx, env,
		teeth[len(teeth)-1], pose, pp.Z1)
	if err != nil || !ok {
		return nil, false, err
	}
	localTeeth = append(localTeeth, localTooth)
	localJoin, admitted, err := UnionTeeth(ctx, env, mesher, localBlank, localTeeth)
	if err != nil || !admitted {
		return nil, admitted, err
	}
	localPayload, ok := localJoin.Payload.(facetedrecord.Payload)
	if !ok || localPayload.LoftCylinderJoin == nil {
		return nil, false, nil
	}
	world, err := localPayload.PlacedWith(ctx, env, pose, nil)
	if err != nil {
		return nil, true, err
	}
	placed, ok := world.Payload.(facetedrecord.Payload)
	if !ok || placed.MeshBound > 0.1 || proofbound.IsNonFinite(placed.MeshBound) {
		return nil, false, nil
	}
	record := *localPayload.LoftCylinderJoin
	record.Blank = blank
	record.Teeth = teeth
	record.Placement = &facetedrecord.LoftCylinderPlacement{Pose: pose, Blank: localBlank, Teeth: localTeeth}
	placed.LoftCylinderJoin = &record
	world.Payload = placed
	return world, true, nil
}

// PlacedBoreCut cuts a centered through-bore from a
// common-placement join in local coordinates and places the result back
// under the pose. It rebuilds the local join through UnionTeeth.
func PlacedBoreCut(ctx context.Context, env topo.Env, mesher verifiedmesh.Mesher,
	target, tool *topo.Body) (*topo.Body, bool, error) {
	joined, ok := target.Payload.(facetedrecord.Payload)
	if !ok || joined.LoftCylinderJoin == nil || joined.LoftCylinderJoin.Placement == nil {
		return nil, false, nil
	}
	placement := joined.LoftCylinderJoin.Placement
	if joined.Xform != placement.Pose || !joined.ContactAudited ||
		placement.Blank == nil || len(placement.Teeth) == 0 ||
		!SourceIdentitiesMatch(placement.Teeth) {
		return nil, false, nil
	}
	record := joined.LoftCylinderJoin
	// Run the local bore's tool-profile and through-span gates before
	// rebuilding the tool. A mismatch belongs to generic Cut; a rebuild error
	// remains observable.
	pp, ok := tool.Payload.(prismrecord.Payload)
	if !ok {
		return nil, false, nil
	}
	if _, ok, err := BoreTool(pp, record.Radius); err != nil || !ok {
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
	localFrame, err := CanonicalFrame(localFrameMap.Translation().Z)
	if err != nil {
		return nil, false, err
	}
	localMap, err := r3.FromFrame(localFrame)
	if err != nil {
		return nil, false, err
	}
	if !AffineOf(frame).Then(AffineOf(pp.Xform)).Equals(
		AffineOf(localMap).Then(AffineOf(placement.Pose))) {
		return nil, false, nil
	}
	origin := localFrame.Origin()
	if !proofbound.FiniteVec(origin) || !BoreThrough(origin.Z, pp.Z0, pp.Z1, record.Z0, record.Z1) {
		return nil, false, nil
	}
	// The source and tool gates above precede both private rebuilds. The join
	// is only needed for an eligible bore; later tooth joins use the sources.
	localJoin, admitted, err := UnionTeeth(ctx,
		topo.Env{Doc: env.Doc, Ref: target.Origin.Producer, Alloc: env.Alloc}, mesher, placement.Blank, placement.Teeth)
	if err != nil || !admitted {
		return nil, admitted, err
	}
	local := pp
	local.Frame, local.Xform = localFrame, r3.Identity()
	localTool, err := local.Placed(ctx, topo.Env{Doc: env.Doc, Ref: tool.Origin.Producer, Alloc: env.Alloc}, r3.Identity())
	if err != nil {
		return nil, false, err
	}
	localBore, admitted, err := JoinBoreCut(ctx, meshbool.OpCut,
		env, mesher, localJoin, localTool)
	if err != nil || !admitted {
		return nil, admitted, err
	}
	localPayload, ok := localBore.Payload.(facetedrecord.Payload)
	if !ok {
		return nil, false, nil
	}
	world, err := localPayload.PlacedWith(ctx, env, placement.Pose, nil)
	if err != nil {
		return nil, true, err
	}
	placed, ok := world.Payload.(facetedrecord.Payload)
	if !ok || placed.MeshBound > 0.1 || proofbound.IsNonFinite(placed.MeshBound) {
		return nil, false, nil
	}
	return world, true, nil
}
