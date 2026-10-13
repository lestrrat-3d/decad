package prismrecord

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// ExactBoxInFrame admits only an authenticated, undisplaced rectangular
// source prism and asks prismplacement.ExactBoxInFrame for its exact box in ref.
func (pp Payload) ExactBoxInFrame(ref r3.Frame) (prismplacement.Box, bool) {
	r, ok := momentinput.AxisRectangle(pp.Profile)
	if !ok || pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		pp.BlendKind != "" || len(pp.BlendSegs) != 0 || pp.Z0 >= pp.Z1 {
		return prismplacement.Box{}, false
	}
	return prismplacement.ExactBoxInFrame(r.U0, r.U1, r.V0, r.V1, pp.Z0, pp.Z1, pp.Frame, ref)
}

// SectionPlaneInside proves that the entire placed sweep preserves the
// plane's u/v coordinates and that the requested level lies between both caps.
// The equalities use exact arithmetic over the held frame and transform bits.
func (pp Payload) SectionPlaneInside(plane r3.Frame) bool {
	source := proofarith.DyVec(pp.Frame.Origin())
	placed := proofarith.DvTransform(pp.Xform, source)
	for _, pair := range [][2]r3.Vec{
		{pp.Frame.U(), plane.U()},
		{pp.Frame.V(), plane.V()},
		{pp.Frame.N(), plane.N()},
	} {
		end := proofarith.DvTransform(pp.Xform, proofarith.DvAdd(source, proofarith.DyVec(pair[0])))
		if !proofarith.DvEqual(proofarith.DvSub(end, placed), proofarith.DyVec(pair[1])) {
			return false
		}
	}
	normal := proofarith.DyVec(plane.N())
	delta := proofarith.DvSub(placed, proofarith.DyVec(plane.Origin()))
	if !proofarith.DvIsZero(proofarith.DvCross(delta, normal)) {
		return false
	}
	for axis := range 3 {
		if normal[axis].IsZero() {
			continue
		}
		level := new(big.Rat).Neg(new(big.Rat).Quo(delta[axis].Rat(), normal[axis].Rat()))
		return level.Cmp(proofarith.FloatRat(pp.Z0)) > 0 && level.Cmp(proofarith.FloatRat(pp.Z1)) < 0
	}
	return false
}
