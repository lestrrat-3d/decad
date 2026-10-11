package circularbounds_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPreparedEndpointMatchesEachRecordedReading(t *testing.T) {
	cases := map[string]circularbounds.CurveSegment{
		"circle": circularbounds.CircleSeg{
			Center: circularbounds.Point2{U: 3, V: -2}, Radius: units.Millimeters(2.5),
		},
		"zero circle": circularbounds.CircleSeg{
			Center: circularbounds.Point2{U: 3, V: -2}, Radius: units.Millimeters(0),
		},
		"arc across zero": circularbounds.ArcSeg{
			Center: circularbounds.Point2{U: 3, V: -2},
			Start:  circularbounds.Point2{U: 5, V: -3},
			End:    circularbounds.Point2{U: 5, V: -1},
		},
		"arc away from zero": circularbounds.ArcSeg{
			Center: circularbounds.Point2{U: -1, V: 1},
			Start:  circularbounds.Point2{U: 1, V: 2},
			End:    circularbounds.Point2{U: -2, V: 3},
		},
		"arc crossing angle wrap": circularbounds.ArcSeg{
			Center: circularbounds.Point2{U: 3, V: -2},
			Start:  circularbounds.Point2{U: 3, V: 0},
			End:    circularbounds.Point2{U: 5, V: -3},
		},
	}
	params := []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 4), big.NewRat(1, 3), big.NewRat(1, 1)}
	for name, segment := range cases {
		t.Run(name, func(t *testing.T) {
			prepared, ok := circularbounds.PrepareEndpoint(segment)
			require.True(t, ok)
			for _, param := range params {
				gotU, gotV, gotOK := prepared.At(param)
				wantU, wantV, wantOK := circularbounds.EndpointInterval(segment, param)
				require.Equal(t, wantOK, gotOK)
				if !wantOK {
					continue
				}
				require.Zero(t, gotU.Lo.Cmp(wantU.Lo))
				require.Zero(t, gotU.Hi.Cmp(wantU.Hi))
				require.Zero(t, gotV.Lo.Cmp(wantV.Lo))
				require.Zero(t, gotV.Hi.Cmp(wantV.Hi))
			}
		})
	}
}
