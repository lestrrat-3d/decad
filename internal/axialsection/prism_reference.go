package axialsection

import (
	"math/big"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/patternrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// AsReference reads an unplaced prism operand the way a loft operand is read, as
// a section between two recorded planes, so a set made only of prisms can build
// the loft this route returns. The two planes are the prism's frame raised along
// its normal by each sweep level. A raised origin is a float multiple of the
// normal added to the frame origin, so its distance off the sweep axis is
// charged as the operand's Lateral exactly as for a loft's end frame. ok is false
// for a placed operand, and for one whose planes are not a sweep of one section.
// A loft operand is returned unchanged.
func (o Operand) AsReference() (Operand, bool) {
	if o.FromLoft {
		return o, true
	}
	if o.Xform != r3.Identity() {
		return o, false
	}
	f0, ok := raised(o.Frame0, o.Level0)
	if !ok {
		return o, false
	}
	f1, ok := raised(o.Frame1, o.Level1)
	if !ok {
		return o, false
	}
	shift, lateral, ok := patternrecord.SweepShift(f0, f1, r3.Identity())
	if !ok || shift.Sign() <= 0 {
		return o, false
	}
	o.Frame0, o.Frame1 = f0, f1
	o.Plane0, o.Plane1 = planeRecordOf(f0), planeRecordOf(f1)
	o.FromLoft, o.Level0, o.Level1, o.Lateral = true, new(big.Rat), shift, lateral
	return o, true
}

// raised is f moved along its normal by level, with its axes untouched. ok is
// false when the level is not a finite float or the moved frame is not valid.
func raised(f r3.Frame, level *big.Rat) (r3.Frame, bool) {
	z, _ := level.Float64()
	origin := f.Origin().Add(f.N().Scale(z))
	moved, err := r3.NewFrame(origin, f.U(), f.V())
	if err != nil || moved.U() != f.U() || moved.V() != f.V() {
		return r3.Frame{}, false
	}
	return moved, true
}

func planeRecordOf(f r3.Frame) sectionrecord.PlaneRecord {
	return sectionrecord.PlaneRecord{Origin: f.Origin(), U: f.U(), V: f.V()}
}
