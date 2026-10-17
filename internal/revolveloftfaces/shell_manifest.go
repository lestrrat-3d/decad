package revolveloftfaces

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
)

func RevolveLoftFacesSameRat(a, b *big.Rat) bool {
	if a == nil || b == nil {
		return a == nil && b == nil
	}
	return a.Cmp(b) == 0
}

func RevolveLoftFacesSameBezier(a, b freeform.BezierSpan) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if !RevolveLoftFacesSameRat(a[i].U, b[i].U) ||
			!RevolveLoftFacesSameRat(a[i].V, b[i].V) {
			return false
		}
	}
	return true
}
