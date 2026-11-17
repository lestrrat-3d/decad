package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// resolvePlanarFace resolves sel to exactly one face of b and returns its
// plane and its outward unit normal. A selector that matches no face or
// several is its own error or ErrDegenerate, and a face that is not an
// analytic plane is ErrUnsupported. feature names the caller in the message.
func resolvePlanarFace(b *Body, sel FaceSelector, feature string) (Plane, VecMeasurement, error) {
	faces, err := sel.SelectFaces(b)
	if err != nil {
		return Plane{}, VecMeasurement{}, err
	}
	if len(faces) != 1 {
		return Plane{}, VecMeasurement{}, fmt.Errorf(`%w: %s needs exactly one face, the selector matched %d`,
			decaderr.ErrDegenerate, feature, len(faces))
	}
	plane, ok := faces[0].Surface().(Plane)
	if !ok {
		return Plane{}, VecMeasurement{}, fmt.Errorf(`%w: %s needs an analytic planar face, this one is %T`,
			decaderr.ErrUnsupported, feature, faces[0].Surface())
	}
	if faces[0].n.ExtentDelta != 0 {
		return Plane{}, VecMeasurement{}, fmt.Errorf(`%w: %s cannot read this planar face as its plane: its tag is an `+
			`oblique wall of a stacked solid that lies off the true plane by an offset the body states (ExtentDelta), `+
			`over the face only, so the plane's frame may not be where the wall is`, decaderr.ErrUnsupported, feature)
	}
	normal, err := faces[0].NormalAt(plane.Frame.Origin())
	if err != nil {
		return Plane{}, VecMeasurement{}, err
	}
	return plane, normal, nil
}
