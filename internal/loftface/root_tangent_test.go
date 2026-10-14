package loftface_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/stretchr/testify/require"
)

func unitBox() [4]*big.Rat { return [4]*big.Rat{rat(-1, 1), rat(1, 1), rat(-1, 1), rat(1, 1)} }

// planeYMap is the cap map P(u, v) = (u, y, v): the plane Y = y.
func planeYMap(y *big.Rat) loftface.Map {
	return loftface.Map{
		Constant: loftface.RatVec{rat(0, 1), rat(0, 1), rat(0, 1)},
		Q0:       loftface.RatVec{rat(0, 1), new(big.Rat).Set(y), rat(0, 1)},
		QU:       ratVecXY(1, 0),
		QV:       loftface.RatVec{rat(0, 1), rat(0, 1), rat(1, 1)},
		G0:       rat(1, 1), GU: rat(0, 1), GV: rat(0, 1),
	}
}

// touchFixture is the unit root circle at X = 0 and the plane Y = 1 over the
// unit box. The plane touches the circle at (0, 1, 0).
func touchFixture() (corners [4][2]*big.Rat, m, b, x *big.Rat) {
	return loftface.BoxCorners(unitBox()), rat(0, 1), rat(1, 1), rat(0, 1)
}

func TestCapPlaneTouchesRootCircle(t *testing.T) {
	corners, m, b, x := touchFixture()
	capMap := planeYMap(rat(1, 1))

	// The circle lies in Y <= 1, so a source band at Y = 0 is on its side.
	require.True(t, loftface.CapPlaneTouchesRootCircle(capMap, corners, m, b, x, planeYMap(rat(0, 1))),
		"a touch from the source side is accepted")
	require.False(t, loftface.CapPlaneTouchesRootCircle(capMap, corners, m, b, x, planeYMap(rat(2, 1))),
		"a touch with the source on the far side of the circle is refused")
	require.False(t, loftface.CapPlaneTouchesRootCircle(capMap, corners, m, b, x, planeYMap(rat(1, 1))),
		"a source on the cap plane has no strict side")
	for _, slope := range []int64{2, -2} {
		// Y = 2v or Y = -2v straddles Y = 1, whichever corner is read last.
		tilted := planeYMap(rat(0, 1))
		tilted.QV = loftface.RatVec{rat(0, 1), rat(slope, 1), rat(1, 1)}
		require.False(t, loftface.CapPlaneTouchesRootCircle(capMap, corners, m, b, x, tilted),
			"a source straddling the cap plane has no single side (slope %d)", slope)
	}
	// Y = (v+1)/2 reaches the cap plane at two corners and stays on the
	// source side at the others.
	grazing := planeYMap(rat(1, 2))
	grazing.QV = loftface.RatVec{rat(0, 1), rat(1, 2), rat(1, 1)}
	require.False(t, loftface.CapPlaneTouchesRootCircle(capMap, corners, m, b, x, grazing),
		"a source reaching the cap plane at a corner has no strict side")
}

func TestCapPlaneTouchesRootCircleRefusesCrossingAndGap(t *testing.T) {
	corners, m, b, x := touchFixture()
	source := planeYMap(rat(0, 1))
	eps := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 80))

	crossing := planeYMap(new(big.Rat).Sub(rat(1, 1), eps))
	require.False(t, loftface.CapPlaneTouchesRootCircle(crossing, corners, m, b, x, source),
		"a plane 2^-80 inside the circle crosses it twice")
	gap := planeYMap(new(big.Rat).Add(rat(1, 1), eps))
	require.False(t, loftface.CapPlaneTouchesRootCircle(gap, corners, m, b, x, source),
		"a plane 2^-80 outside the circle misses it")
}

func TestCapPlaneTouchesRootCircleRefusesBadCircleAndPlane(t *testing.T) {
	corners, m, b, x := touchFixture()
	capMap := planeYMap(rat(1, 1))
	source := planeYMap(rat(0, 1))

	// A negative radius gives the same a^2 = y^2+z^2 identity.
	require.False(t, loftface.CapPlaneTouchesRootCircle(capMap, corners, m, rat(-1, 1), x, source),
		"a negative radius is not a circle")
	require.False(t, loftface.CapPlaneTouchesRootCircle(capMap, corners, m, rat(0, 1), x, source),
		"a zero radius is not a circle")

	// The plane X = 0 contains the whole circle: a = 0.
	containing := loftface.Map{
		Constant: loftface.RatVec{rat(0, 1), rat(0, 1), rat(0, 1)},
		Q0:       loftface.RatVec{rat(0, 1), rat(0, 1), rat(0, 1)},
		QU:       loftface.RatVec{rat(0, 1), rat(1, 1), rat(0, 1)},
		QV:       loftface.RatVec{rat(0, 1), rat(0, 1), rat(1, 1)},
		G0:       rat(1, 1), GU: rat(0, 1), GV: rat(0, 1),
	}
	for _, side := range []int64{-1, 1} {
		source := containing
		source.Constant = loftface.RatVec{rat(side, 1), rat(0, 1), rat(0, 1)}
		require.False(t, loftface.CapPlaneTouchesRootCircle(containing, corners, m, b, x, source),
			"a plane containing the circle is not a single touch (source side %d)", side)
	}

	collapsed := planeYMap(rat(1, 1))
	collapsed.QU = ratVecXY(0, 0)
	require.False(t, loftface.CapPlaneTouchesRootCircle(collapsed, corners, m, b, x, source),
		"a collapsed cap has no supporting plane")
}

// tangentFixture is the cone x + r = 8 with the tangent end map along +Y.
func tangentFixture(t *testing.T) loftface.Map {
	t.Helper()
	end, err := loftface.NewTangentEnd(rat(8, 1), rat(1, 1), 1, 0)
	require.NoError(t, err)
	return end.Map()
}

func TestCapPlaneTouchesRootCircleOnTangentMap(t *testing.T) {
	tangent := tangentFixture(t)
	// The cone cross-section at X = 5 is the circle of radius 3, so the
	// tangent plane touches it at one point: radius = -x + 8.
	corners := loftface.BoxCorners(unitBox())
	m, b, x := rat(-1, 1), rat(8, 1), rat(5, 1)

	// The cone's axis side is the side the circle is on. e is read from
	// the map: QV = (0, -e_z, e_y).
	ey, ez := tangent.QV[2], new(big.Rat).Neg(tangent.QV[1])
	shifted := func(w *big.Rat) loftface.Map {
		source := tangent
		source.Constant = loftface.RatVec{rat(8, 1), new(big.Rat).Mul(w, ey), new(big.Rat).Mul(w, ez)}
		return source
	}
	require.True(t, loftface.CapPlaneTouchesRootCircle(tangent, corners, m, b, x, shifted(rat(-1, 2))),
		"the issued tangent map touches its own cone cross-section from the axis side")
	require.False(t, loftface.CapPlaneTouchesRootCircle(tangent, corners, m, b, x, shifted(rat(1, 2))),
		"the same touch from the far side is refused")
}

// nestedMap is the cap map P(u, v) = (8 + qux*u, quy*u, v).
func nestedMap(qux, quy *big.Rat) loftface.Map {
	return loftface.Map{
		Constant: loftface.RatVec{rat(8, 1), rat(0, 1), rat(0, 1)},
		Q0:       loftface.RatVec{rat(0, 1), rat(0, 1), rat(0, 1)},
		QU:       loftface.RatVec{qux, quy, rat(0, 1)},
		QV:       loftface.RatVec{rat(0, 1), rat(0, 1), rat(1, 1)},
		G0:       rat(1, 1), GU: rat(0, 1), GV: rat(0, 1),
	}
}

func nestedBox(u0, u1 *big.Rat) [4]*big.Rat { return [4]*big.Rat{u0, u1, rat(-1, 1), rat(1, 1)} }

func TestCapOutsideNestedCone(t *testing.T) {
	tangent := tangentFixture(t)
	box := nestedBox(rat(-4, 1), rat(-3, 1))
	outerApex, outerSlope, outerFar := rat(8, 1), rat(1, 1), rat(0, 1) // R = 8
	innerApex, innerSlope := rat(6, 1), rat(1, 1)                      // G(0) = 2, G(R) = 2
	check := func(m loftface.Map, b [4]*big.Rat, apex, slope, far, iApex, iSlope *big.Rat) bool {
		return loftface.CapOutsideNestedCone(m, b, apex, slope, far, iApex, iSlope)
	}
	require.True(t, check(tangent, box, outerApex, outerSlope, outerFar, innerApex, innerSlope),
		"the issued tangent map nests in a wider blank cone")

	// q3 = 3 > 0: the plane is strictly outside the outer cone.
	strict := nestedMap(rat(1, 1), rat(2, 1))
	strictBox := nestedBox(rat(-3, 1), rat(-5, 2))
	require.True(t, check(strict, strictBox, outerApex, outerSlope, outerFar, innerApex, innerSlope))

	t.Run("G(R) negative by 2^-80", func(t *testing.T) {
		eps := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 80))
		// G(R) = -innerApex + 8*slope = -2^-80.
		slope := new(big.Rat).Quo(new(big.Rat).Sub(innerApex, eps), rat(8, 1))
		require.False(t, check(tangent, box, outerApex, outerSlope, outerFar, innerApex, slope))
	})
	t.Run("G(0) zero", func(t *testing.T) {
		require.False(t, check(tangent, box, outerApex, outerSlope, outerFar, rat(8, 1), rat(2, 1)))
	})
	t.Run("box reaches beyond R", func(t *testing.T) {
		// R = 1 while the box has r >= 2.
		require.False(t, check(tangent, box, outerApex, outerSlope, rat(7, 1), innerApex, innerSlope))
	})
	t.Run("outer q3 negative", func(t *testing.T) {
		negative := nestedMap(rat(2, 1), rat(1, 2)) // q3 = 1/4 - 4
		q := loftface.ConePullback(negative, outerApex, outerSlope)
		require.Negative(t, q[3].Sign())
		require.False(t, check(negative, box, outerApex, outerSlope, outerFar, innerApex, innerSlope))
	})
	t.Run("pullback has a constant term", func(t *testing.T) {
		moved := tangent
		moved.Constant = loftface.RatVec{rat(8, 1), rat(1, 1), rat(0, 1)}
		require.False(t, check(moved, box, outerApex, outerSlope, outerFar, innerApex, innerSlope))
	})
	t.Run("pullback has no v^2 term", func(t *testing.T) {
		flat := tangent
		flat.QV = loftface.RatVec{rat(0, 1), rat(0, 1), rat(0, 1)}
		require.False(t, check(flat, box, outerApex, outerSlope, outerFar, innerApex, innerSlope))
	})
	t.Run("negative denominator", func(t *testing.T) {
		flipped := tangent
		flipped.Constant = tangent.Constant
		for i := range 3 {
			flipped.Q0[i] = new(big.Rat).Neg(tangent.Q0[i])
			flipped.QU[i] = new(big.Rat).Neg(tangent.QU[i])
			flipped.QV[i] = new(big.Rat).Neg(tangent.QV[i])
		}
		flipped.G0 = rat(-1, 1)
		// The same plane and points, so only the denominator sign differs.
		require.Zero(t, flipped.At(rat(-3, 1), rat(1, 2))[0].Cmp(tangent.At(rat(-3, 1), rat(1, 2))[0]))
		require.False(t, check(flipped, box, outerApex, outerSlope, outerFar, innerApex, innerSlope))
	})
	t.Run("box X reaches the inner apex", func(t *testing.T) {
		// X = 8 + u/sqrt(2) reaches 6.59 to 7.29 here, above the inner apex 6.
		require.False(t, check(tangent, nestedBox(rat(-2, 1), rat(-1, 1)),
			outerApex, outerSlope, outerFar, innerApex, innerSlope))
	})
	t.Run("negative outer slope", func(t *testing.T) {
		// R = (8-16)/-1 = 8 is positive and G(R) > 0, but the slope is not.
		require.False(t, check(tangent, box, outerApex, rat(-1, 1), rat(16, 1), innerApex, innerSlope))
	})
	t.Run("negative R", func(t *testing.T) {
		require.False(t, check(tangent, box, outerApex, outerSlope, rat(16, 1), innerApex, innerSlope))
	})
}
