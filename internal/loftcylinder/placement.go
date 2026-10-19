package loftcylinder

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// CanonicalFrame returns the local XY frame at axial level z that a
// common-placement join rebuilds its blank, teeth and bore tool in.
func CanonicalFrame(z float64) (r3.Frame, error) {
	return r3.NewFrame(r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
}

func frameMatchesPlane(frame r3.Frame, plane sectionrecord.PlaneRecord) bool {
	return frame.Origin() == plane.Origin && frame.U() == plane.U && frame.V() == plane.V
}

// These cheap source-shape checks run before a private reconstruction. A
// mismatch belongs to the ordinary Boolean; a failure after reconstruction
// starts is a proof or work-limit error and remains visible to the caller.
func placedBlankShape(pp prismrecord.Payload) bool {
	if pp.SurfaceResult || pp.Z0 != 0 || pp.Z1 <= 0 || proofbound.IsNonFinite(pp.Z1) ||
		pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 {
		return false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	return ok && circle.Center == (sectionrecord.Point2{}) && circle.CCW &&
		circle.Radius.Base() > 0 && !proofbound.IsNonFinite(circle.Radius.Base())
}

// PlacedToothShape reports whether tooth is an authenticated, solid,
// hole-free six-curve Loft with one segment, an even held vertex count of at
// least 16, and the circular Union's section curve kinds.
// It is the cheap check a common-placement Union runs before rebuilding the
// tooth.
func PlacedToothShape(tooth *topo.Body) bool {
	lp, ok := tooth.Payload.(loftrecord.Payload)
	return ok && lp.AuthenticatedReconstruction && !lp.SurfaceResult && lp.SourceIdentity != nil &&
		len(lp.Profile0.Holes) == 0 && len(lp.Profile1.Holes) == 0 &&
		len(lp.Profile0.Outer.Segments) == 6 && len(lp.Profile1.Outer.Segments) == 6 &&
		len(lp.Segment) == 1 && len(lp.Verts)%2 == 0 && len(lp.Verts) >= 16 &&
		SectionKinds(lp)
}

// LocalBlank rebuilds the exact circular blank in the canonical XY frame and
// returns it with the blank's world pose. The blank must be a centered
// counterclockwise circle prism starting at its frame, and the frame and
// placement must compose to a signed-axis pose other than identity whose
// float composition equals the exact rational one. A mismatch reports false
// with a nil error. The local blank is placed through env's document and
// allocator under blank's own producer.
func LocalBlank(ctx context.Context, env topo.Env, blank *topo.Body) (*topo.Body, r3.Transform, bool, error) {
	pp, ok := blank.Payload.(prismrecord.Payload)
	if !ok || !placedBlankShape(pp) {
		return nil, r3.Transform{}, false, nil
	}
	frame, err := r3.FromFrame(pp.Frame)
	if err != nil {
		return nil, r3.Transform{}, false, err
	}
	pose, err := frame.Then(pp.Xform)
	if err != nil || pose == r3.Identity() || !SignedPose(pose) ||
		!AffineOf(frame).Then(AffineOf(pp.Xform)).Equals(AffineOf(pose)) {
		return nil, r3.Transform{}, false, nil //nolint:nilerr // an invalid pose cannot use this certificate
	}
	localFrame, err := CanonicalFrame(0)
	if err != nil {
		return nil, r3.Transform{}, false, err
	}
	local := pp
	local.Frame, local.Xform = localFrame, r3.Identity()
	env.Ref = blank.Origin.Producer
	localBody, err := local.Placed(ctx, env, r3.Identity())
	return localBody, pose, err == nil, err
}

// LocalTooth accepts only a source whose two recorded section planes are
// exact images of one axial local Loft under pose, and rebuilds that local
// Loft with its top section at level z1. The local copy keeps the original
// producer ID and authenticated source identity; it is placed through env's
// document and allocator. A mismatch reports false with a nil error.
func LocalTooth(ctx context.Context, env topo.Env, tooth *topo.Body,
	pose r3.Transform, z1 float64) (*topo.Body, bool, error) {
	lp, ok := tooth.Payload.(loftrecord.Payload)
	if !ok || !lp.AuthenticatedReconstruction || lp.SourceIdentity == nil ||
		!frameMatchesPlane(lp.Frame0, lp.Plane0) ||
		!frameMatchesPlane(lp.Frame1, lp.Plane1) {
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
	if err != nil || !AxialPlacement(localXform) {
		return nil, false, nil //nolint:nilerr // an invalid local map cannot certify axial placement
	}
	local0, err := CanonicalFrame(0)
	if err != nil {
		return nil, false, err
	}
	local1, err := CanonicalFrame(z1)
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
	actual0 := AffineOf(first).Then(AffineOf(lp.Xform))
	actual1 := AffineOf(last).Then(AffineOf(lp.Xform))
	want0 := AffineOf(localFirst).Then(AffineOf(localXform)).Then(AffineOf(pose))
	want1 := AffineOf(localLast).Then(AffineOf(localXform)).Then(AffineOf(pose))
	if !actual0.Equals(want0) || !actual1.Equals(want1) {
		return nil, false, nil
	}
	local := lp
	local.Plane0 = sectionrecord.PlaneRecord{Origin: local0.Origin(), U: local0.U(), V: local0.V()}
	local.Plane1 = sectionrecord.PlaneRecord{Origin: local1.Origin(), U: local1.U(), V: local1.V()}
	local.Frame0, local.Frame1, local.Xform = local0, local1, localXform
	env.Ref = tooth.Origin.Producer
	localBody, err := local.Placed(ctx, env, localXform)
	return localBody, err == nil, err
}
