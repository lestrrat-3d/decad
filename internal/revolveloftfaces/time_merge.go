package revolveloftfaces

import (
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/sketch"
)

type RevolveLoftFacesJointKind uint8

const (
	RevolveLoftFacesFitKnotContact RevolveLoftFacesJointKind = iota + 1
	RevolveLoftFacesRadialArcContact
	RevolveLoftFacesWholeArcContact
	RevolveLoftFacesFitLineContact
)

// Each root owns one live source endpoint. Fit-knot, radial-arc and fit-line
// records remain distinct even when their time brackets occupy one span.
type RevolveLoftFacesOwnedRoot struct {
	Root  loftface.EndpointRoot
	Cell  RevolveLoftFacesJointCell
	Kind  RevolveLoftFacesJointKind
	Point *sketch.Point
	AtEnd bool
}
