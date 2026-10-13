package revolvemesh

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// CapFrame is the plane frame of a cap at sweep angle φ, under the
// placement. The start cap's outward (material-leaving) normal points
// against the sweep, the end cap's along it; the axes are ordered so the
// frame's normal IS the outward normal, and a reflected placement swaps them
// once more (a reflection flips the cross product's handedness).
func CapFrame(b RevolveBasis, xform r3.Transform, phi float64, start bool) (r3.Frame, error) {
	sin, cos := math.Sincos(phi)
	radial := b.E0.Scale(cos).Add(b.E1.Scale(sin))
	// (w, radial) has w × radial = the rotated plane normal — the sweep
	// velocity direction, the END cap's outward normal.
	u3, v3 := b.W, radial
	if start {
		u3, v3 = v3, u3
	}
	if xform.IsReflection() {
		u3, v3 = v3, u3
	}
	f, err := r3.NewFrame(xform.Apply(b.A3), xform.ApplyDir(u3), xform.ApplyDir(v3))
	if err != nil {
		return r3.Frame{}, fmt.Errorf(`%w: the placed cap frame is degenerate: %s`, decaderr.ErrDegenerate, err)
	}
	return f, nil
}
