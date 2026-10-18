package prismshell

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/r3"
)

// ReverseClosedBossStack makes a narrow lower slab the upper boss of the
// existing certified construction. Flipping the frame's V axis, each section's
// V coordinates, and the recorded levels preserves the same world geometry
// and a right-handed face frame.
func ReverseClosedBossStack(ctx context.Context, sp stackedrecord.Payload) (stackedrecord.Payload, bool, error) {
	if len(sp.Slabs) != 2 || len(sp.Interfaces) != 1 ||
		len(sp.Slabs[0].Regions) != 1 || len(sp.Slabs[1].Regions) != 1 {
		return stackedrecord.Payload{}, false, nil
	}
	inner, innerOK := RecordedBossRect(sp.Slabs[0].Regions[0])
	outer, outerOK := RecordedBossRect(sp.Slabs[1].Regions[0])
	if !innerOK || !outerOK || !(inner.U0 > outer.U0 && inner.V0 > outer.V0 &&
		inner.U1 < outer.U1 && inner.V1 < outer.V1) {
		return stackedrecord.Payload{}, false, nil
	}
	frame, err := r3.NewFrame(sp.Frame.Origin(), sp.Frame.U(), sp.Frame.V().Scale(-1))
	if err != nil {
		return stackedrecord.Payload{}, true, fmt.Errorf("%w: the closed boss shell cannot reverse its section frame: %v",
			decaderr.ErrUnsupported, err)
	}
	if frame.Origin() != sp.Frame.Origin() || frame.U() != sp.Frame.U() ||
		frame.V() != sp.Frame.V().Scale(-1) || frame.N() != sp.Frame.N().Scale(-1) {
		return stackedrecord.Payload{}, true, fmt.Errorf("%w: the closed boss shell needs an exact reversed section frame",
			decaderr.ErrUnsupported)
	}
	reflectRect := func(r BossRect) profileRecord {
		return (BossRect{U0: r.U0, V0: -r.V1, U1: r.U1, V1: -r.V0}).Profile()
	}
	exposed, err := stackedrecord.UnionExposed(ctx, reflectRect(outer), reflectRect(inner))
	if err != nil {
		return stackedrecord.Payload{}, true, fmt.Errorf("%w: the closed boss shell cannot reverse its interface: %v",
			decaderr.ErrUnsupported, err)
	}
	a, b := sp.Slabs[0], sp.Slabs[1]
	return stackedrecord.Payload{
		Slabs: []stackedrecord.Slab{
			{Regions: []profileRecord{reflectRect(outer)}, Z0: -b.Z1, Z1: -b.Z0, Z0Delta: b.Z1Delta, Z1Delta: b.Z0Delta},
			{Regions: []profileRecord{reflectRect(inner)}, Z0: -a.Z1, Z1: -a.Z0, Z0Delta: a.Z1Delta, Z1Delta: a.Z0Delta},
		},
		Interfaces: []stackedrecord.Interface{{
			LowerExposed: exposed,
		}},
		Frame: frame, Xform: sp.Xform, SectionDelta: sp.SectionDelta,
	}, true, nil
}
