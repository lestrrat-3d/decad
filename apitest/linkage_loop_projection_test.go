package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestVerifyLinkageLoopProjection pins docs/linkage-check-design.md §5.8's
// expansion over a loop's dependent joint on scene 7's crank-rocker. A block
// x ∈ [96, 104], y ∈ [60, 66], z ∈ [40, 48] rides the follower in a layer of
// its own, under a ceiling x ∈ [50, 150], y ∈ [67.5, 77.5], z ∈ [39, 49]. The
// follower turns by Δ = θ4(θ2) − θ4(0) about O4 = (100, 0), so the block's
// top corners sit at height 66·cos Δ + 4·|sin Δ| and the gap is
// 67.5 − 66·cos Δ − 4·|sin Δ|, smallest, 67.5 − √4372 ≈ 1.379 mm — below the
// 2 mm the layer exclusion proves between the four-bar's own links — where
// Δ = −atan(4/66): twice along the crank's 0° → 90°, as the follower dips
// past that turn and comes back. Each is a flat minimum of the drive.
//
// At the defaults the report reads Sound, the reading encloses the minimum,
// every IntervalClear interval sits at or below the closed-form gap at its
// ends, and no interval is narrower than 1/1024. Measured: 36 poses down to
// 1/128, against 1119 down to 1/8192 under the travel bound alone.
//
// Legs seen to fail when deleted: the dependent's expansion (the pair takes
// the travel bound alone, and the reading refines past 1/1024); the hull of
// the interval in h (the end's enclosure alone serves and the pin tests of
// §15.10 certify across their collisions); and the far end of that hull
// (h a quarter of the hull's width, the same).
func TestVerifyLinkageLoopProjection(t *testing.T) {
	t.Parallel()
	const g, r, f = rockerGround, rockerCrank, rockerFollower
	doc := decad.New()
	t4 := rockerTheta4(0)
	b := [2]float64{g + f*math.Cos(t4), f * math.Sin(t4)}
	crankBody := boxBodyAtZ(t, doc, 0, -4, r, 4, 0, 8)
	coupler := barBody(t, doc, [2]float64{r, 0}, b, 10)
	foll := barBody(t, doc, [2]float64{g, 0}, b, 20)
	block := boxBodyAtZ(t, doc, 96, 60, 104, 66, 40, 8)
	boxBodyAtZ(t, doc, 50, 67.5, 150, 77.5, 39, 10)
	lk := decad.NewLinkage()
	z := r3.NewVec(0, 0, 1)
	crank, err := lk.Ground().Revolute(r3.Vec{}, z, []*decad.Body{crankBody})
	require.NoError(t, err)
	couplerLk, err := crank.Revolute(r3.NewVec(r, 0, 0), z, []*decad.Body{coupler})
	require.NoError(t, err)
	follow, err := lk.Ground().Revolute(r3.NewVec(g, 0, 0), z, []*decad.Body{foll, block})
	require.NoError(t, err)
	_, err = lk.Close(couplerLk, follow, r3.NewVec(b[0], b[1], 0), z)
	require.NoError(t, err)

	gap := func(s float64) float64 {
		delta := rockerTheta4(s*math.Pi/2) - t4
		return 67.5 - 66*math.Cos(delta) - 4*math.Abs(math.Sin(delta))
	}
	minimum := 67.5 - math.Sqrt(66*66+4*4)
	report := verifyLinkage(t, doc, lk, decad.Drive{{Link: crank, From: units.Degrees(0), To: units.Degrees(90)}})
	require.Equal(t, decad.Sound, report.Status)
	requireReadingEncloses(t, report, minimum)
	requireIntervalsBelow(t, report, gap, nil)
	require.GreaterOrEqual(t, narrowest(report), 1.0/1024)
}
