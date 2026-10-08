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
// V axis k+2, modulo 3, with every sign positive. It is PlanarFrame(ref, k,
// +1).
func AxisFrame(ref r3.Frame, k int) (r3.Frame, Embed, error) {
	return PlanarFrame(ref, k, 1)
}

// PlanarFrame is the right-handed signed permutation of ref whose normal is
// reference axis k scaled by the sign of sign (docs/brep-modify-design.md
// §5.2). For a positive sign U is axis k+1 and V axis k+2, modulo 3, every
// sign positive; for a negative one U is axis k+2 and V axis k+1, and the
// normal is −k. It shares ref's origin, so a coordinate along its normal is
// the reference coordinate along axis k times that sign. The returned Embed
// maps the frame's local axes onto ref's. A frame r3.NewFrame does not
// rebuild with exactly those axes is ErrAxisFrame.
func PlanarFrame(ref r3.Frame, k int, sign float64) (r3.Frame, Embed, error) {
	e := Embed{Axis: [3]int{(k + 1) % 3, (k + 2) % 3, k}, Sign: [3]float64{1, 1, 1}}
	if sign < 0 {
		e = Embed{Axis: [3]int{(k + 2) % 3, (k + 1) % 3, k}, Sign: [3]float64{1, 1, -1}}
	}
	axes := [3]r3.Vec{ref.U(), ref.V(), ref.N()}
	var vecs [3]r3.Vec
	for i := range 3 {
		vecs[i] = axes[e.Axis[i]]
		if e.Sign[i] < 0 {
			vecs[i] = vecs[i].Scale(-1)
		}
	}
	frame, err := r3.NewFrame(ref.Origin(), vecs[0], vecs[1])
	if err != nil {
		return r3.Frame{}, Embed{}, ErrAxisFrame
	}
	if frame.U() != vecs[0] || frame.V() != vecs[1] || frame.N() != vecs[2] {
		return r3.Frame{}, Embed{}, ErrAxisFrame
	}
	return frame, e, nil
}
