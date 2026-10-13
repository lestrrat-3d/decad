package smoothloft

import "math/big"

// fourCubicControls interpolates exact values at t=0, 1/3, 2/3, 1.
func fourCubicControls(values [4]*big.Rat) [4]*big.Rat {
	var controls [4]*big.Rat
	controls[0], controls[3] = values[0], values[3]
	// The Bernstein weights at 1/3 and 2/3 are (8,12,6,1)/27
	// and (1,6,12,8)/27.
	a := div(sub(sub(mul(rint(27), values[1]), mul(rint(8), values[0])), values[3]), rint(6))
	b := div(sub(sub(mul(rint(27), values[2]), values[0]), mul(rint(8), values[3])), rint(6))
	controls[1] = div(sub(mul(rint(2), a), b), rint(3))
	controls[2] = div(sub(mul(rint(2), b), a), rint(3))
	return controls
}

func fourCubicAt(controls [4]*big.Rat, t *big.Rat) *big.Rat {
	u := sub(rint(1), t)
	return add(add(mul(controls[0], mul(mul(u, u), u)),
		mul(rint(3), mul(controls[1], mul(mul(u, u), t)))),
		add(mul(rint(3), mul(controls[2], mul(u, mul(t, t)))),
			mul(controls[3], mul(mul(t, t), t))))
}

// A cubic's second derivative is linear in its two second differences.
func fourCubicCurvature(controls [4]*big.Rat) *big.Rat {
	first := add(sub(controls[0], mul(rint(2), controls[1])), controls[2])
	second := add(sub(controls[1], mul(rint(2), controls[2])), controls[3])
	return mul(rint(6), maxRat(abs(first), abs(second)))
}

func fourCubicMaximum(controls [4]*big.Rat) *big.Rat {
	maximum := controls[0]
	for _, control := range controls[1:] {
		maximum = maxRat(maximum, control)
	}
	return maximum
}

// The derivative is a quadratic Bernstein curve with these three controls.
func fourCubicSlopeMaximum(controls [4]*big.Rat) *big.Rat {
	maximum := rint(0)
	for i := range 3 {
		maximum = maxRat(maximum, mul(rint(3), abs(sub(controls[i+1], controls[i]))))
	}
	return maximum
}
