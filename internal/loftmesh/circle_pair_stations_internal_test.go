package loftmesh

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func recordedCircleSide(t *testing.T, radius, cu, cv, start, end float64, ccw bool) LoftCircularSide {
	t.Helper()
	segment := sectionrecord.CircleSeg{
		Center: sectionrecord.Point2{U: cu, V: cv}, Radius: units.Millimeters(radius),
		TStart: start, TEnd: end, CCW: ccw,
	}
	w, err := boundarywalk.WalkOf(segment, nil)
	require.NoError(t, err)
	return RecordCircularSide(w, segment)
}

// The reference retains the former two independent station walks and the
// published refusal order. It does not call the shared-turn path.
func independentCircularCellStations(a, b LoftCircularSide, target float64) (
	[]LoftStation, []LoftStation, float64, []float64, float64, error,
) {
	m, s0, s1, err := LoftSettleStationCount(a, b, target)
	if err != nil {
		return nil, nil, 0, nil, 0, err
	}
	stations0, d0 := LoftCircularStationChain(a, m)
	stations1, d1 := LoftCircularStationChain(b, m)
	stationUpper := math.Max(d0, d1)
	if proofbound.IsNonFinite(stationUpper) {
		return nil, nil, 0, nil, 0, ErrLoftStationDisplacementUnderivable
	}
	for k := range m - 1 {
		eq0 := stations0[k] == stations0[k+1]
		eq1 := stations1[k] == stations1[k+1]
		if eq0 != eq1 {
			return nil, nil, 0, nil, 0, fmt.Errorf(
				`%w: chord cell %d of this paired segment collapses to one point on only one of the two sections`,
				decaderr.ErrUnsupported, k,
			)
		}
	}
	sagitta := math.Max(s0, s1)
	matchedDelta := make([]float64, m)
	for i := range matchedDelta {
		matchedDelta[i] = sagitta
	}
	return stations0, stations1, sagitta, matchedDelta, stationUpper, nil
}

func TestCircularCellStationsMatchIndependentWalkReference(t *testing.T) {
	circle := recordedCircleSide(t, 10, 2, -3, 0, 1, true)
	shared := recordedCircleSide(t, 5, -7, 4, 0, 1, true)
	differentRange := recordedCircleSide(t, 5, -7, 4, 0.125, 0.875, true)
	zeroRadius := recordedCircleSide(t, 0, 0, 0, 0, 1, true)
	arcSegment := sectionrecord.ArcSeg{
		Center: sectionrecord.Point2{}, Start: sectionrecord.Point2{U: 5}, End: sectionrecord.Point2{V: 5},
		TStart: 0, TEnd: 1,
	}
	arcWalk, err := boundarywalk.WalkOf(arcSegment, nil)
	require.NoError(t, err)
	arc := RecordCircularSide(arcWalk, arcSegment)
	invalidRange := shared
	invalidRange.RangeOK = false
	for _, tc := range []struct {
		name string
		a, b LoftCircularSide
	}{
		{"shared circle", circle, shared},
		{"different ranges", circle, differentRange},
		{"one-sided collapse", circle, zeroRadius},
		{"arc pair", arc, arc},
		{"underivable range", circle, invalidRange},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got0, got1, gotSagitta, gotMatched, gotRound, gotErr := LoftCircularCellStations(tc.a, tc.b, 0.25)
			want0, want1, wantSagitta, wantMatched, wantRound, wantErr :=
				independentCircularCellStations(tc.a, tc.b, 0.25)
			if wantErr != nil {
				require.EqualError(t, gotErr, wantErr.Error())
				require.Equal(t, want0, got0)
				require.Equal(t, want1, got1)
				return
			}
			require.NoError(t, gotErr)
			requireSameCircleChainBits(t, got0, want0, gotRound, wantRound)
			requireSameCircleChainBits(t, got1, want1, gotRound, wantRound)
			require.Equal(t, math.Float64bits(wantSagitta), math.Float64bits(gotSagitta))
			require.Equal(t, len(wantMatched), len(gotMatched))
			for i := range wantMatched {
				require.Equal(t, math.Float64bits(wantMatched[i]), math.Float64bits(gotMatched[i]))
			}
		})
	}
}

func requireSameCircleChainBits(t *testing.T, got, want []LoftStation, gotDelta, wantDelta float64) {
	t.Helper()
	require.Equal(t, len(want), len(got))
	for k := range want {
		require.Equal(t, math.Float64bits(want[k].U), math.Float64bits(got[k].U), "station %d U", k)
		require.Equal(t, math.Float64bits(want[k].V), math.Float64bits(got[k].V), "station %d V", k)
	}
	require.Equal(t, math.Float64bits(wantDelta), math.Float64bits(gotDelta))
}

func TestSharedCircleStationChainsMatchIndependentRecordedWalks(t *testing.T) {
	cases := []struct {
		name       string
		start, end float64
		ccw        bool
	}{
		{"full forward", 0, 1, true},
		{"full reverse", 1, 0, false},
		{"trimmed", 0.125, 0.875, true},
		{"large binary denominator", math.Nextafter(0.125, 1), math.Nextafter(0.875, 0), true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			a := recordedCircleSide(t, 10, math.Nextafter(3, 4), -2, tc.start, tc.end, tc.ccw)
			b := recordedCircleSide(t, 5, -7, math.Nextafter(1, 2), tc.start, tc.end, tc.ccw)
			startA, spanA := a.TStart.RatString(), a.DT.RatString()
			startB, spanB := b.TStart.RatString(), b.DT.RatString()
			counts := []int{7, 32}
			if tc.name == "full forward" || tc.name == "full reverse" {
				counts = []int{7, 31, 32, 281, 345}
			}
			for _, m := range counts {
				got0, got1, d0, d1, shared := sharedCircleStationChains(a, b, m)
				require.True(t, shared, "m=%d", m)
				want0, oldD0 := LoftCircularStationChain(a, m)
				want1, oldD1 := LoftCircularStationChain(b, m)
				requireSameCircleChainBits(t, got0, want0, d0, oldD0)
				requireSameCircleChainBits(t, got1, want1, d1, oldD1)
			}
			require.Equal(t, startA, a.TStart.RatString())
			require.Equal(t, spanA, a.DT.RatString())
			require.Equal(t, startB, b.TStart.RatString())
			require.Equal(t, spanB, b.DT.RatString())
		})
	}
}

func TestSharedCircleMemoEligibility(t *testing.T) {
	negativeZero := new(big.Rat).SetFloat64(math.Copysign(0, -1))
	require.NotNil(t, negativeZero)
	for _, tc := range []struct {
		name        string
		start, span *big.Rat
		m           int
		want        bool
	}{
		{"forward below threshold", big.NewRat(0, 1), big.NewRat(1, 1), 31, false},
		{"forward at threshold", big.NewRat(0, 1), big.NewRat(1, 1), 32, true},
		{"odd forward", big.NewRat(0, 1), big.NewRat(1, 1), 281, true},
		{"odd reverse", big.NewRat(1, 1), big.NewRat(-1, 1), 345, true},
		{"negative zero start", negativeZero, big.NewRat(1, 1), 32, true},
		{"partial", big.NewRat(0, 1), big.NewRat(1, 2), 345, false},
		{"shifted", big.NewRat(1, 4), big.NewRat(1, 1), 345, false},
		{"reverse without full turn", big.NewRat(1, 1), big.NewRat(-1, 2), 345, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			require.Equal(t, tc.want, sharedCircleMemoEligible(tc.start, tc.span, tc.m))
		})
	}
}

func TestSharedCircleStationChainsKeepIndependentFallbacks(t *testing.T) {
	a := recordedCircleSide(t, 10, 0, 0, 0, 1, true)
	b := recordedCircleSide(t, 5, 0, 0, 0.125, 0.875, true)
	_, _, _, _, shared := sharedCircleStationChains(a, b, 7) //nolint:dogsled // This case checks only the fallback choice.
	require.False(t, shared, "different exact ranges must keep their independent proofs")

	b = recordedCircleSide(t, 0, 0, 0, 0, 1, true)
	_, _, _, _, shared = sharedCircleStationChains(a, b, 7) //nolint:dogsled // This case checks only the fallback choice.
	require.False(t, shared, "a zero-radius circle has no turn interval to share")

	b = a
	b.RangeOK = false
	_, _, _, _, shared = sharedCircleStationChains(a, b, 7) //nolint:dogsled // This case checks only the fallback choice.
	require.False(t, shared, "an underivable range retains its old refusal path")
	b.TStart, b.DT = nil, nil
	_, _, _, _, shared = sharedCircleStationChains(a, b, 345) //nolint:dogsled // The invalid range must return before its parameters are read.
	require.False(t, shared, "invalid scalars must not enter the memo path")
}
