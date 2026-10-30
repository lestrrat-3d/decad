package revolvebore

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/r3"
)

// MapTool maps an undisplaced prism into an exact frame on the Revolve axis.
// Both tool caps must coincide with the meridian's ends.
func MapTool(ctx context.Context, rp revolveplan.Payload, mer Meridian, tool prismrecord.Payload) (prismrecord.Payload, bool, error) {
	b := rp.Basis()
	if rp.Lift().BasisRound(b) != ([4]float64{}) {
		return prismrecord.Payload{}, false, nil
	}
	frame, err := r3.NewFrame(b.A3, b.E0, b.E1)
	if err != nil {
		//nolint:nilerr // An unrepresentable frame is an admission miss.
		return prismrecord.Payload{}, false, nil
	}
	if frame.U() != b.E0 || frame.V() != b.E1 || frame.N() != b.W {
		return prismrecord.Payload{}, false, nil
	}
	target := prismrecord.Payload{Frame: frame, Xform: rp.Xform, Z0: mer.Low, Z1: mer.High}
	pa, pb := target.PlacementOperand(), tool.PlacementOperand()
	if !sameRevolveCaps(rp, pa, mer) {
		return prismrecord.Payload{}, false, nil
	}
	composeA, composeB := pa, pb
	if pa.Xform == pb.Xform {
		composeA.Xform, composeB.Xform = r3.Identity(), r3.Identity()
	}
	relative, err := prismplacement.Compose(composeA, composeB)
	if err != nil {
		return prismrecord.Payload{}, false, err
	}
	if !exactRelative(pa, pb, relative) {
		return prismrecord.Payload{}, false, nil
	}
	loop, low, high, ok, err := classbgeom.ReexpressAxialBoreTool(ctx, tool.Profile.Outer, relative,
		pa, pb)
	if err != nil || !ok || low != mer.Low || high != mer.High {
		return prismrecord.Payload{}, false, err
	}
	tool.Profile.Outer = loop
	tool.Frame, tool.Xform, tool.Z0, tool.Z1 = frame, rp.Xform, low, high
	tool.Z0Denoted, tool.Z1Denoted = prismplacement.LevelDenotation{}, prismplacement.LevelDenotation{}
	return tool, true, nil
}

// sameRevolveCaps compares both cap axis points and planes denoted by the
// Revolve and target axial frame, without treating a rounded normal as exact.
func sameRevolveCaps(rp revolveplan.Payload, target prismplacement.Operand, mer Meridian) bool {
	b := rp.Basis()
	origin, axes := exactFrame(target)
	normal := proofarith.DvCross(axes[0], axes[1])
	if proofarith.DvIsZero(normal) {
		return false
	}
	for _, level := range []float64{mer.Low, mer.High} {
		if proofarith.FloatRat(level) == nil {
			return false
		}
		s := proofarith.MustDyOf(level)
		hostPoint := proofarith.DvTransform(rp.Xform,
			proofarith.DvAdd(proofarith.DyVec(b.A3), proofarith.DvScale(proofarith.DyVec(b.W), s)))
		targetPoint := proofarith.DvAdd(origin, proofarith.DvScale(axes[2], s))
		if !proofarith.DvEqual(hostPoint, targetPoint) {
			return false
		}
	}
	return true
}

// exactRelative checks the relative map against the two exact frame lifts.
// Four affine points establish equality over every section point and level.
func exactRelative(target, tool prismplacement.Operand, relative prismplacement.Relative) bool {
	var origin, ex, ey, ez r3.Vec
	if relative.Identity {
		shift := prismplacement.ZShift(target, tool)
		z, _ := shift.Float64()
		if proofarith.RationalFloatError(shift, z) != 0 {
			return false
		}
		origin, ex, ey, ez = r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}
	} else {
		origin = relative.Map.Translation()
		basis := relative.Map.Basis()
		ex, ey, ez = basis.EX, basis.EY, basis.EZ
	}
	if origin.X != 0 || origin.Y != 0 {
		return false
	}
	for _, v := range []r3.Vec{origin, ex, ey, ez} {
		if proofarith.FloatRat(v.X) == nil || proofarith.FloatRat(v.Y) == nil || proofarith.FloatRat(v.Z) == nil {
			return false
		}
	}
	toolOrigin, toolAxes := exactFrame(tool)
	targetOrigin, targetAxes := exactFrame(target)
	lift := func(p r3.Vec) proofarith.DyV3 {
		out := targetOrigin
		for i, v := range []float64{p.X, p.Y, p.Z} {
			out = proofarith.DvAdd(out, proofarith.DvScale(targetAxes[i], proofarith.MustDyOf(v)))
		}
		return out
	}
	if !proofarith.DvEqual(toolOrigin, lift(origin)) {
		return false
	}
	for i, col := range []r3.Vec{ex, ey, ez} {
		mapped := proofarith.DvSub(lift(col), targetOrigin)
		if !proofarith.DvEqual(toolAxes[i], mapped) {
			return false
		}
	}
	return true
}

func exactFrame(op prismplacement.Operand) (proofarith.DyV3, [3]proofarith.DyV3) {
	u, v := proofarith.DyVec(op.Frame.U()), proofarith.DyVec(op.Frame.V())
	axes := [3]proofarith.DyV3{u, v, proofarith.DvCross(u, v)}
	origin := proofarith.DvTransform(op.Xform, proofarith.DyVec(op.Frame.Origin()))
	translation := proofarith.DyVec(op.Xform.Translation())
	for i := range axes {
		axes[i] = proofarith.DvSub(proofarith.DvTransform(op.Xform, axes[i]), translation)
	}
	return origin, axes
}
