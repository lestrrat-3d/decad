package apitest_test

import (
	"fmt"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestStackedUnionCrossingBossFarFromOrigin stands the Ø10 boss of
// TestStackedUnionCrossingBossStanding on the 40 mm plate 1e6 mm and 3e7 mm
// along y from the origin, its centre 2 mm inside the wall x = 20 and 1.3 mm
// off the plate's centre line. The A1 brep pins each crossing of the wall
// with the boss's circle to one keyed vertex, the wall's walked point there.
// The scene cut the wall at a parameter that, this far out, is off the
// crossing by the coordinates' own rounding (about ulp(1e6) = 1.2e-10 mm)
// rather than the cut allowance of 8 ulps of [0, 1] times the wall's 40 mm,
// and every face pinned at the vertex inherits that distance. The area is
// 4800 + 150π + 2·S, S = 25·acos(0.4) − 2·√21, whatever the translation; the
// volume is the plate plus the whole boss, 16000 + 375π.
//
// Shown-to-fail: without the keyed vertex's own crossing offset
// (stackedbrep's crossingOffset in Engine.junction) the area at 1e6 reads
// 5.5e-9 mm² off the exact value under a 2.0e-10 bound, and at 3e7 1.3e-7
// under 2.0e-10.
func TestStackedUnionCrossingBossFarFromOrigin(t *testing.T) {
	t.Parallel()
	// acos(0.4) = 2·atan(√0.84 / 1.4).
	root := jf(0.84)
	root.Sqrt(root)
	acos := junctionAtan(new(big.Float).SetPrec(junctionPrec).Quo(root, jf(1.4)))
	acos.Mul(acos, jf(2))
	segment := new(big.Float).SetPrec(junctionPrec).Mul(acos, jf(25))
	r21 := jf(21)
	r21.Sqrt(r21)
	segment.Sub(segment, r21.Mul(r21, jf(2)))
	wantArea := new(big.Float).SetPrec(junctionPrec).Mul(junctionPi(), jf(150))
	wantArea.Add(wantArea, jf(4800))
	wantArea.Add(wantArea, segment.Mul(segment, jf(2)))
	wantVolume := new(big.Float).SetPrec(junctionPrec).Mul(junctionPi(), jf(375))
	wantVolume.Add(wantVolume, jf(16000))

	for _, y0 := range []float64{1e6, 3e7} {
		t.Run(fmt.Sprint(y0), func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := boxBody(t, doc, -20, y0-20, 20, y0+20, 10)

			w := sketch.NewWorld()
			plane, err := w.CreateOffsetPlane(w.XY(), 10)
			require.NoError(t, err)
			s, err := w.CreateSketch(plane)
			require.NoError(t, err)
			center := s.CreatePoint(18, y0+1.3)
			s.Fix(center)
			s.CreateCircle(center, 5)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			boss, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(15), Dir: decad.Along})
			require.NoError(t, err)

			got, err := decad.Union(t.Context(), plate, boss)
			require.NoError(t, err)
			require.False(t, anyFaceIsFaceted(got), `the A1 brep, not the mesh path, builds the union`)
			require.Len(t, got.Faces(), 10)

			area, err := got.Area()
			require.NoError(t, err)
			value, err := area.Value.In(units.SquareMillimeter)
			require.NoError(t, err)
			bound, err := area.Bound.In(units.SquareMillimeter)
			require.NoError(t, err)
			requireJunctionEncloses(t, `area`, value, bound, wantArea)

			volume, err := got.Volume()
			require.NoError(t, err)
			requireJunctionEncloses(t, `volume`, volumeMM(t, volume), boundMM3(t, volume), wantVolume)
		})
	}
}
