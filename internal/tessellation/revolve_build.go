package tessellation

import (
	"github.com/lestrrat-3d/r3"
)

// RevolveBasis is the unplaced world anchor of the sweep: a3 the axis
// origin, w the unit axis direction, e0 the in-plane radial direction at
// sweep angle zero, and e1 = w × e0 the sweep-velocity direction at zero —
// so a rotation by +φ about w carries e0 toward e1, the right-handed sense
// Along means.
type RevolveBasis struct {
	A3, W, E0, E1 r3.Vec
}
