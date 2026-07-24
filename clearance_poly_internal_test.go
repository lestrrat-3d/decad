package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRatPolyOfRejectsNonFiniteCoefficient(t *testing.T) {
	for _, coeff := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		p, ok := ratPolyOf(1, coeff, 2)
		require.False(t, ok)
		require.Nil(t, p)
	}
}

func TestLineCircleBracketsRejectsNonFinitePolynomial(t *testing.T) {
	cp := circleParam{
		c: [3]float64{math.MaxFloat64, 0, 0},
		u: [3]float64{1, 0, 0},
		v: [3]float64{0, 1, 0},
		r: 1,
	}
	brackets, nonConstant, err := lineCircleBracketsContext(
		t.Context(),
		cp,
		[3]float64{},
		[3]float64{0, 0, 1},
		1e-9,
	)
	require.ErrorIs(t, err, errNonFiniteClearancePolynomial)
	require.False(t, nonConstant)
	require.Empty(t, brackets)
}

func TestLineCircleBracketsAcceptsFinitePolynomial(t *testing.T) {
	cp := circleParam{
		c: [3]float64{3, 0, 0},
		u: [3]float64{1, 0, 0},
		v: [3]float64{0, 1, 0},
		r: 1,
	}
	brackets, nonConstant, err := lineCircleBracketsContext(
		t.Context(),
		cp,
		[3]float64{},
		[3]float64{0, 0, 1},
		1e-9,
	)
	require.NoError(t, err)
	require.True(t, nonConstant)
	require.NotEmpty(t, brackets)
}

func TestTorusCrossingsRejectsNonFinitePolynomial(t *testing.T) {
	face := cFace{
		kind:   ckTorus,
		axis:   r3.NewVec(0, 0, 1),
		major:  math.MaxFloat64,
		radius: 1,
	}
	count, decided, err := face.torusCrossings(
		t.Context(),
		r3.Vec{},
		r3.NewVec(1, 0, 0),
		1e-9,
	)
	require.NoError(t, err)
	require.False(t, decided)
	require.Zero(t, count)
}
