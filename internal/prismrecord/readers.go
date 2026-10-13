package prismrecord

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/r3"
)

// CapFrame is the plane frame of a cap at height z, under the placement;
// flip swaps the in-plane axes so the frame's normal is the cap's OUTWARD
// normal, and a reflected placement swaps once more (a reflection flips the
// cross product's handedness). The payload frame is orthonormal and the
// placement rigid, so the transformed axes cannot be degenerate; NewFrame
// re-validates anyway and the error propagates rather than being discarded.
func (pp Payload) CapFrame(z float64, flip bool) (r3.Frame, error) {
	if pp.Reflected() {
		flip = !flip
	}
	u, v := pp.Dir(1, 0, 0), pp.Dir(0, 1, 0)
	if flip {
		u, v = v, u
	}
	f, err := r3.NewFrame(pp.Point(0, 0, z), u, v)
	if err != nil {
		return r3.Frame{}, fmt.Errorf(`%w: the placed cap frame is degenerate: %s`, decaderr.ErrDegenerate, err)
	}
	return f, nil
}

// MassRecord presents the prism record to the mass integrator.
func (pp Payload) MassRecord() massmoment.PrismRecord {
	return massmoment.PrismRecord{
		Profile: pp.Profile, Frame: pp.Frame, Transform: pp.Xform,
		Z0: pp.Z0, Z1: pp.Z1, SectionDelta: pp.SectionDelta,
		Z0Delta: pp.Z0Delta, Z1Delta: pp.Z1Delta,
	}
}
