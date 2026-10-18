package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/stretchr/testify/require"
)

// TestSpiralTangentRecordCurrent checks the exact record gate on the real
// rectangle trim record of the cone-tangent chain. Each mutation changes one
// immutable value by the smallest amount the gate must notice.
func TestSpiralTangentRecordCurrent(t *testing.T) {
	c := buildTangentTrimChain(t)
	trimBody, err := Intersect(t.Context(), c.cut, c.heelTool)
	require.NoError(t, err)
	trim := trimBody.n.Payload.(facetedPayload).LoftFacesTrim
	require.NotNil(t, trim)
	require.NotNil(t, trim.Tangent)
	require.True(t, spiralsector.SpiralTangentRecordCurrent(trim), "the real record is current")

	t.Run("no record is not current", func(t *testing.T) {
		bare := *trim
		bare.Tangent = nil
		require.False(t, spiralsector.SpiralTangentRecordCurrent(&bare))
		require.False(t, spiralsector.SpiralTangentRecordCurrent(nil))
	})

	t.Run("unequal tangent copy", func(t *testing.T) {
		other := *trim.Tangent
		other.T = new(big.Rat).Add(other.T, big.NewRat(1, 1<<40))
		mutated := *trim
		mutated.Tangent = &other
		require.False(t, spiralsector.SpiralTangentRecordCurrent(&mutated))
	})

	t.Run("equal copy by value is current", func(t *testing.T) {
		same := *trim.Tangent
		same.T = new(big.Rat).Set(same.T)
		same.Rho = new(big.Rat).Set(same.Rho)
		same.Apex = new(big.Rat).Set(same.Apex)
		same.Slope = new(big.Rat).Set(same.Slope)
		copied := *trim
		copied.Tangent = &same
		require.True(t, spiralsector.SpiralTangentRecordCurrent(&copied), "equality is by value, not pointer")
	})

	t.Run("base cap record tangent differs", func(t *testing.T) {
		other := *trim.Tangent
		other.Rho = new(big.Rat).Add(other.Rho, big.NewRat(1, 1<<40))
		capRecord := *trim.Base.LoftFacesCap
		capRecord.Tangent = &other
		mutated := *trim
		mutated.Base.LoftFacesCap = &capRecord
		require.False(t, spiralsector.SpiralTangentRecordCurrent(&mutated))
		capRecord.Tangent = nil
		require.False(t, spiralsector.SpiralTangentRecordCurrent(&mutated), "the base record must carry it too")
	})

	t.Run("heel slope one float step off", func(t *testing.T) {
		slope, _ := trim.Heel.Slope.Float64()
		stepped := new(big.Rat)
		stepped.SetFloat64(math.Nextafter(slope, math.Inf(1)))
		require.NotZero(t, stepped.Cmp(trim.Heel.Slope))
		heel := *trim.Heel
		heel.Slope = stepped
		mutated := *trim
		mutated.Heel = &heel
		require.False(t, spiralsector.SpiralTangentRecordCurrent(&mutated))
	})

	t.Run("one changed final-map coefficient", func(t *testing.T) {
		maps := append([]loftFaceMap(nil), trim.Maps...)
		last := maps[len(maps)-1]
		qu := last.QU
		qu[1] = new(big.Rat).Add(qu[1], big.NewRat(1, 1<<60))
		last.QU = qu
		maps[len(maps)-1] = last
		mutated := *trim
		mutated.Maps = maps
		require.False(t, spiralsector.SpiralTangentRecordCurrent(&mutated))
		require.True(t, loftface.MapEqual(trim.Tangent.Map(), trim.Maps[len(trim.Maps)-1]),
			"the original map is untouched")
	})
}
