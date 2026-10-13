package classbgeom

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// ShiftedTool re-expresses Y's record into g, Y's axes at the reference
// origin: every coordinate moves by the exact rational dot of the origin
// difference with its axis. The shift is admitted only where every shifted
// coordinate and level is itself a float, so the re-expression rounds
// nothing: a datum plane and its CreateOffsetPlane at a float distance meet
// that, and a pair whose sums would round misses.
func ShiftedTool(ref r3.Frame, y prismrecord.Payload, g r3.Frame) (prismrecord.Payload, bool) {
	loops := append([]sectionrecord.LoopRecord{y.Profile.Outer}, y.Profile.Holes...)
	moved, z0, z1, ok := ShiftedRecord(ref, y.Frame, g, loops, y.Z0, y.Z1)
	out := y
	out.Frame = g
	out.Profile = momentinput.Profile{Outer: moved[0], Holes: moved[1:]}
	out.Z0, out.Z1 = z0, z1
	out.Walks = nil
	return out, ok
}
