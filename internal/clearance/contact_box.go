package clearance

import (
	"github.com/lestrrat-3d/r3"
)

func SignedAxis(v r3.Vec) (int, int, bool) {
	switch v {
	case r3.Vec{X: -1}:
		return 0, 0, true
	case r3.Vec{X: 1}:
		return 0, 1, true
	case r3.Vec{Y: -1}:
		return 1, 0, true
	case r3.Vec{Y: 1}:
		return 1, 1, true
	case r3.Vec{Z: -1}:
		return 2, 0, true
	case r3.Vec{Z: 1}:
		return 2, 1, true
	default:
		return 0, 0, false
	}
}
