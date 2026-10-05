package dynamics

import (
	"github.com/lestrrat-3d/r3"
)

func sameOrientation(a, b r3.Transform) bool {
	for _, basis := range []r3.Vec{{X: 1}, {Y: 1}, {Z: 1}} {
		if a.ApplyDir(basis) != b.ApplyDir(basis) {
			return false
		}
	}
	return true
}
