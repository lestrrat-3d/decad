package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// Section returns the material profile where this solid meets plane. The
// returned Sketch uses plane and its profiles can enter another feature. The
// query is read-only, including when the body has been retired.
//
// One class is an exact straight prism cut strictly between its caps by a
// plane whose frame preserves the prism's recorded section coordinates.
// Another is a whole-circle prism cut by a cardinal plane through its axis.
// Other body or plane classes return [ErrUnsupported]. A nil body or context,
// or an invalid plane, returns [ErrDegenerate]. A sheet or unsound solid returns
// [ErrNotSolid]. See docs/section-design.md.
func (b *Body) Section(ctx context.Context, plane r3.Frame) (*sketch.Sketch, []*sketch.Profile, error) {
	if b == nil || ctx == nil || !plane.IsValid() {
		return nil, nil, fmt.Errorf(`%w: a section needs a body, context and valid plane`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, nil, err
	}
	if b.Kind() != BodySolid || !b.IsSolid() {
		return nil, nil, fmt.Errorf(`%w: a section needs a sound solid`, ErrNotSolid)
	}
	pp, ok := b.payload.(prismPayload)
	if !ok || pp.sectionDelta != 0 || pp.z0Delta != 0 || pp.z1Delta != 0 {
		return nil, nil, fmt.Errorf(`%w: section needs an exact straight prism`, ErrUnsupported)
	}
	profile := pp.profile
	if !prismSectionPlaneInside(pp, plane) {
		var admitted bool
		profile, admitted = cardinalCircleSectionRecord(pp, plane)
		if !admitted {
			return nil, nil, fmt.Errorf(`%w: section plane does not cross an admitted exact prism section`, ErrUnsupported)
		}
	}
	work := freeform.NewFreeformWork()
	if _, err := momentinput.ChargeReconstruction(profile, work); err != nil {
		return nil, nil, err
	}
	s, result, err := momentinput.ReconstructProfileOnFrame(ctx, profile, plane)
	if err != nil {
		return nil, nil, err
	}
	return s, []*sketch.Profile{result}, nil
}

// prismSectionPlaneInside proves that the entire placed sweep preserves the
// plane's u/v coordinates and that the requested level lies between both caps.
// The equalities use exact arithmetic over the held frame and transform bits.
func prismSectionPlaneInside(pp prismPayload, plane r3.Frame) bool {
	source := proofarith.DyVec(pp.frame.Origin())
	placed := proofarith.DvTransform(pp.xform, source)
	for _, pair := range [][2]r3.Vec{
		{pp.frame.U(), plane.U()},
		{pp.frame.V(), plane.V()},
		{pp.frame.N(), plane.N()},
	} {
		end := proofarith.DvTransform(pp.xform, proofarith.DvAdd(source, proofarith.DyVec(pair[0])))
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
		return level.Cmp(proofarith.FloatRat(pp.z0)) > 0 && level.Cmp(proofarith.FloatRat(pp.z1)) < 0
	}
	return false
}
