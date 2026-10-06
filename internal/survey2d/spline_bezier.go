package survey2d

import (
	"math/big"
)

// RatPoint is a plane-local coordinate over exact rationals — Point2's exact
// counterpart, in millimetres by the same core §5.2 convention.
type RatPoint struct{ U, V *big.Rat }

// BezierSpan is one polynomial Bézier piece of a converted free-form curve:
// its control points in order, so its degree is len(BezierSpan)-1. A Bézier
// interpolates its first and last control point exactly, which is why
// consecutive spans join on a shared coordinate value and the chain's first and
// last control points ARE the recorded curve's own endpoints.
type BezierSpan []RatPoint
