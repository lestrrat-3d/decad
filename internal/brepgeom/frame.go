package brepgeom

import (
	"errors"

	"github.com/lestrrat-3d/r3"
)

// ErrAxisFrame marks a reference frame whose axes do not rebuild bit for bit
// as a signed-permutation frame.
var ErrAxisFrame = errors.New("the reference axes do not rebuild as a signed-permutation frame")

// AxisFrame is the right-handed signed permutation of ref whose normal is
// reference axis k (docs/brep-modify-design.md §4.1 P3): U is axis k+1 and
// V axis k+2, modulo 3, with every sign positive. It shares ref's origin, so
// a coordinate along its normal is the reference coordinate along axis k.
// The returned Embed maps the frame's local axes onto ref's. A frame
// r3.NewFrame does not rebuild with exactly those axes is ErrAxisFrame.
func AxisFrame(ref r3.Frame, k int) (r3.Frame, Embed, error) {
	axes := [3]r3.Vec{ref.U(), ref.V(), ref.N()}
	e := Embed{Axis: [3]int{(k + 1) % 3, (k + 2) % 3, k}, Sign: [3]float64{1, 1, 1}}
	frame, err := r3.NewFrame(ref.Origin(), axes[e.Axis[0]], axes[e.Axis[1]])
	if err != nil {
		return r3.Frame{}, Embed{}, ErrAxisFrame
	}
	if frame.U() != axes[e.Axis[0]] || frame.V() != axes[e.Axis[1]] || frame.N() != axes[e.Axis[2]] {
		return r3.Frame{}, Embed{}, ErrAxisFrame
	}
	return frame, e, nil
}
