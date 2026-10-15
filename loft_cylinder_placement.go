package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// loftCylinderPlacement keeps the local source bodies used by the structural
// certificate. The public operands and their producer identities stay in the
// surrounding join record. A later bore rebuilds the local join from these
// sources instead of retaining its mesh. Only this exact common pose maps the
// local result to those operands; an ordinary later placement drops the record.
type loftCylinderPlacement struct {
	Pose  r3.Transform
	Blank *topo.Body
	Teeth []*topo.Body
}

// loftCylinderAffine compares the exact rational maps represented by stored
// frame and transform entries. It is an admission check, not coordinate math
// used to construct geometry. A rounded composition cannot establish that
// two source solids are rigid images of the same local source.
type loftCylinderAffine struct {
	linear [3][3]*big.Rat
	shift  [3]*big.Rat
}

func loftCylinderAffineOf(t r3.Transform) loftCylinderAffine {
	b := t.Basis()
	columns := [3]r3.Vec{b.EX, b.EY, b.EZ}
	var out loftCylinderAffine
	for j, column := range columns {
		for i, value := range [3]float64{column.X, column.Y, column.Z} {
			out.linear[i][j] = proofarith.FloatRat(value)
		}
	}
	for i, value := range [3]float64{t.Translation().X, t.Translation().Y, t.Translation().Z} {
		out.shift[i] = proofarith.FloatRat(value)
	}
	return out
}

func (a loftCylinderAffine) then(next loftCylinderAffine) loftCylinderAffine {
	var out loftCylinderAffine
	for i := range 3 {
		out.shift[i] = new(big.Rat).Set(next.shift[i])
		for k := range 3 {
			out.shift[i].Add(out.shift[i], new(big.Rat).Mul(next.linear[i][k], a.shift[k]))
		}
		for j := range 3 {
			out.linear[i][j] = new(big.Rat)
			for k := range 3 {
				out.linear[i][j].Add(out.linear[i][j],
					new(big.Rat).Mul(next.linear[i][k], a.linear[k][j]))
			}
		}
	}
	return out
}

func (a loftCylinderAffine) equals(b loftCylinderAffine) bool {
	for i := range 3 {
		if a.shift[i].Cmp(b.shift[i]) != 0 {
			return false
		}
		for j := range 3 {
			if a.linear[i][j].Cmp(b.linear[i][j]) != 0 {
				return false
			}
		}
	}
	return true
}

// A signed permutation has exact unit length, determinant +1, and no basis
// roundoff. Other common rotations can use the generic Boolean until their
// source-image and output bounds have a separate proof.
func loftCylinderSignedPose(pose r3.Transform) bool {
	if !pose.IsValid() || pose.IsReflection() || !proofbound.FiniteVec(pose.Translation()) {
		return false
	}
	b := pose.Basis()
	used := [3]bool{}
	for _, column := range [3]r3.Vec{b.EX, b.EY, b.EZ} {
		values := [3]float64{column.X, column.Y, column.Z}
		found := -1
		for i, value := range values {
			if value == 0 {
				continue
			}
			if (value != 1 && value != -1) || found >= 0 || used[i] {
				return false
			}
			found = i
		}
		if found < 0 {
			return false
		}
		used[found] = true
	}
	return true
}

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
	return loftCylinderSourceIdentitiesMatch(len(teeth), func(i int) topo.Payload { return teeth[i].n.Payload })
}

// loftCylinderPlacedNodeSourcesMatch is loftCylinderPlacedSourcesMatch over
// the body nodes a join record stores.
func loftCylinderPlacedNodeSourcesMatch(teeth []*topo.Body) bool {
	return loftCylinderSourceIdentitiesMatch(len(teeth), func(i int) topo.Payload { return teeth[i].Payload })
}

// loftCylinderSourceIdentitiesMatch reports whether the payloads of teeth
// 0..n-1, read through payload in index order, are authenticated Lofts
// sharing the first tooth's source identity. n must be at least 1.
func loftCylinderSourceIdentitiesMatch(n int, payload func(i int) topo.Payload) bool {
	first, ok := payload(0).(loftPayload)
	if !ok || first.SourceIdentity == nil {
		return false
	}
	for i := 1; i < n; i++ {
		next, ok := payload(i).(loftPayload)
		if !ok || next.SourceIdentity != first.SourceIdentity {
			return false
		}
	}
	return true
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
	if err != nil || pose == r3.Identity() || !loftCylinderSignedPose(pose) ||
		!loftCylinderAffineOf(frame).then(loftCylinderAffineOf(pp.Xform)).equals(loftCylinderAffineOf(pose)) {
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
	actual0 := loftCylinderAffineOf(first).then(loftCylinderAffineOf(lp.Xform))
	actual1 := loftCylinderAffineOf(last).then(loftCylinderAffineOf(lp.Xform))
	want0 := loftCylinderAffineOf(localFirst).then(loftCylinderAffineOf(localXform)).
		then(loftCylinderAffineOf(pose))
	want1 := loftCylinderAffineOf(localLast).then(loftCylinderAffineOf(localXform)).
		then(loftCylinderAffineOf(pose))
	if !actual0.equals(want0) || !actual1.equals(want1) {
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
	world, err := localPayload.placedWith(ctx, d, ref, pose, nil)
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
	if !loftCylinderAffineOf(frame).then(loftCylinderAffineOf(pp.Xform)).equals(
		loftCylinderAffineOf(localMap).then(loftCylinderAffineOf(placement.Pose))) {
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
	world, err := localPayload.placedWith(ctx, d, ref, placement.Pose, nil)
	if err != nil {
		return nil, true, err
	}
	placed, ok := world.n.Payload.(facetedPayload)
	if !ok || placed.MeshBound > 0.1 || proofbound.IsNonFinite(placed.MeshBound) {
		return nil, false, nil
	}
	return world, true, nil
}
