package prismplacement_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// inchLevel is the level 0.2 in denotes, fl(0.2)·fl(25.4), its held float
// 5.08 and a displacement covering the rounding between them.
func inchLevel(t *testing.T) (*big.Rat, float64, float64) {
	t.Helper()
	exact := new(big.Rat).Mul(new(big.Rat).SetFloat64(0.2), new(big.Rat).SetFloat64(25.4))
	held, _ := exact.Float64()
	require.Equal(t, 5.08, held)
	require.Negative(t, exact.Cmp(new(big.Rat).SetFloat64(held)), "0.2 in denotes a level below its float")
	return exact, held, 1e-15
}

// levelOperand is an operand on the XY frame raised by originZ, over z0..z1.
func levelOperand(t *testing.T, originZ, z0, z1 float64) prismplacement.Operand {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{Z: originZ}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	return prismplacement.Operand{Frame: frame, Xform: r3.Identity(), Z0: z0, Z1: z1}
}

// displacedTop sets op's top level displacement and, when exact is not nil,
// the level it denotes.
func displacedTop(op prismplacement.Operand, delta float64, exact *big.Rat) prismplacement.Operand {
	op.Z1Delta = delta
	op.Z1Denoted = prismplacement.DenoteLevel(exact, op.Z1, delta)
	return op
}

func TestCutZIntervalSpans(t *testing.T) {
	t.Parallel()
	inch, held, delta := inchLevel(t)
	plate := levelOperand(t, 0, -3, held)
	inchPlate := displacedTop(plate, delta, inch)
	tool := levelOperand(t, 0, -20, held)
	for _, tc := range []struct {
		name         string
		target, tool prismplacement.Operand
		want         bool
	}{
		{"undisplaced caps meet", plate, tool, true},
		{"displaced tool top without a denotation", plate, displacedTop(tool, delta, nil), false},
		{"tool top denotes a level below the target top", plate, displacedTop(tool, delta, inch), false},
		{"both tops denote one level", inchPlate, displacedTop(tool, delta, inch), true},
		{"displaced target top without a denotation", displacedTop(plate, delta, nil), tool, false},
		{"tool top clears its displacement", plate, displacedTop(levelOperand(t, 0, -20, 5.09), delta, nil), true},
		{"tool bottom within its displacement", plate, func() prismplacement.Operand {
			op := levelOperand(t, 0, -3, 6)
			op.Z0Delta = delta
			return op
		}(), false},
		{"tool bottom clears its displacement", plate, func() prismplacement.Operand {
			op := levelOperand(t, 0, -3.01, 6)
			op.Z0Delta = delta
			return op
		}(), true},
		{"shifted tool top meets exactly", plate, levelOperand(t, 1, -21, held-1), true},
		{"negative displacement", plate, displacedTop(tool, -delta, nil), false},
		{"non-finite displacement", plate, displacedTop(tool, math.NaN(), nil), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, prismplacement.CutZIntervalSpans(tc.target, tc.tool))
		})
	}
}

func TestUnionZIntervalMatches(t *testing.T) {
	t.Parallel()
	inch, held, delta := inchLevel(t)
	a := levelOperand(t, 0, -3, held)
	for _, tc := range []struct {
		name string
		a, b prismplacement.Operand
		want bool
	}{
		{"undisplaced equal intervals", a, a, true},
		{"both tops denote one level", displacedTop(a, delta, inch), displacedTop(a, delta, inch), true},
		{"equal floats, one top displaced without a denotation", a, displacedTop(a, delta, nil), false},
		{"equal floats, both tops displaced without a denotation", displacedTop(a, delta, nil), displacedTop(a, delta, nil), false},
		{"equal floats denoting different levels", a, displacedTop(a, delta, inch), false},
		{"different floats", a, levelOperand(t, 0, -3, 5), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, prismplacement.UnionZIntervalMatches(tc.a, tc.b))
		})
	}
}

func TestIntersectZIntervalOverlaps(t *testing.T) {
	t.Parallel()
	inch, held, delta := inchLevel(t)
	// B starts fl(5e-17) below its plane at 5.08: above the level 0.2 in
	// denotes, below its held float.
	b := levelOperand(t, held, -5e-17, 10)
	bottom := new(big.Rat).Sub(new(big.Rat).SetFloat64(held), new(big.Rat).SetFloat64(5e-17))
	require.Positive(t, bottom.Cmp(inch))
	a := levelOperand(t, 0, -3, held)
	for _, tc := range []struct {
		name string
		a, b prismplacement.Operand
		want bool
	}{
		{"undisplaced overlap", a, b, true},
		{"denoted top below the other's bottom", displacedTop(a, delta, inch), b, false},
		{"displaced top without a denotation", displacedTop(a, delta, nil), b, false},
		{"displaced top clears its displacement", displacedTop(levelOperand(t, 0, -3, 5.09), delta, nil), b, true},
		{"touching caps", levelOperand(t, 0, -3, 0), levelOperand(t, 0, 0, 1), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, prismplacement.IntersectZIntervalOverlaps(tc.a, tc.b))
		})
	}
}

func TestIntersectEndChargesBothDisplacements(t *testing.T) {
	t.Parallel()
	five, up := big.NewRat(5, 1), math.Nextafter(5, 6)
	upRat := new(big.Rat).SetFloat64(up)
	// covers reports whether delta reaches from held to the farthest true
	// result end, far.
	covers := func(held float64, delta float64, far *big.Rat) bool {
		gap := new(big.Rat).Sub(far, new(big.Rat).SetFloat64(held))
		return new(big.Rat).SetFloat64(delta).Cmp(gap.Abs(gap)) >= 0
	}
	t.Run("A wins, B's range reaches past it", func(t *testing.T) {
		t.Parallel()
		// B's upper end holds one ulp above 5 with a 1e-14 displacement, so
		// B's true end may lie below A's and become the result.
		held, delta := prismplacement.IntersectUpperEnd(5, 0, upRat, 1e-14)
		require.Equal(t, 5.0, held)
		require.True(t, covers(held, delta, new(big.Rat).Sub(upRat, new(big.Rat).SetFloat64(1e-14))))
	})
	t.Run("B wins, A's range reaches past it", func(t *testing.T) {
		t.Parallel()
		// A's lower end holds 5 with a 1e-14 displacement and B's one ulp
		// above it with none: A's true start may lie above B's.
		held, delta := prismplacement.IntersectLowerEnd(5, 1e-14, upRat, 0)
		require.Equal(t, up, held)
		require.True(t, covers(held, delta, new(big.Rat).Add(five, new(big.Rat).SetFloat64(1e-14))))
	})
	t.Run("a loser that cannot reach keeps the winner's displacement", func(t *testing.T) {
		t.Parallel()
		// B's upper end is 3 above A's: its displacement cannot make it the
		// result, so A's exact end stays exact.
		held, delta := prismplacement.IntersectUpperEnd(2, 0, big.NewRat(5, 1), 1e-14)
		require.Equal(t, 2.0, held)
		require.Zero(t, delta)
		held, delta = prismplacement.IntersectLowerEnd(-3, 1e-14, big.NewRat(-1, 1), 0)
		require.Equal(t, -1.0, held)
		require.Zero(t, delta)
	})
}
