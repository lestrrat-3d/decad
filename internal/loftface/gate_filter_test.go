package loftface

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// legacyGateWorst is the unfiltered tile loop that gateWorst replaced: the
// exact deviation of every tile row, keeping the maximum.
func legacyGateWorst(firstGrid, secondGrid [][]gateNode, tiles, rows int) *big.Rat {
	worst := ratZero()
	for iu := range tiles {
		for iv := range tiles {
			firstCorners := [4]*gateNode{&firstGrid[iu][iv], &firstGrid[iu][iv+1],
				&firstGrid[iu+1][iv], &firstGrid[iu+1][iv+1]}
			secondCorners := [4]*gateNode{&secondGrid[iu][iv], &secondGrid[iu][iv+1],
				&secondGrid[iu+1][iv], &secondGrid[iu+1][iv+1]}
			for row := range rows {
				deviation := ratZero()
				for axis := range 2 {
					hull := IntervalHull(gateDerivRange(firstCorners, row, axis),
						gateDerivRange(secondCorners, row, axis))
					deviation = ratAdd(deviation, IntervalAbs(hull, big.NewRat(int64(boolInt(row == axis)), 1)))
				}
				velocity := IntervalSub(gateScalarRange(secondCorners, row),
					gateScalarRange(firstCorners, row))
				deviation = ratAdd(deviation, IntervalAbs(velocity, big.NewRat(int64(boolInt(row == 2)), 1)))
				if deviation.Cmp(worst) > 0 {
					worst = deviation
				}
			}
		}
	}
	return worst
}

// legacyBandGate is BandGate with the unfiltered tile loop.
func legacyBandGate(ctx context.Context, first, second Map,
	box [4]*big.Rat) (float64, float64, int, error) {
	centerU := ratDiv(ratAdd(box[0], box[1]), big.NewRat(2, 1))
	centerV := ratDiv(ratAdd(box[2], box[3]), big.NewRat(2, 1))
	c0 := VScale(VAdd(first.Deriv(centerU, centerV, 0), second.Deriv(centerU, centerV, 0)),
		big.NewRat(1, 2))
	c1 := VScale(VAdd(first.Deriv(centerU, centerV, 1), second.Deriv(centerU, centerV, 1)),
		big.NewRat(1, 2))
	c2 := VSub(second.At(centerU, centerV), first.At(centerU, centerV))
	rows, det, err := InverseRows(c0, c1, c2)
	if err != nil {
		return 0, 0, 0, err
	}
	const tiles = 16
	widthU := ratDiv(ratSub(box[1], box[0]), big.NewRat(tiles, 1))
	widthV := ratDiv(ratSub(box[3], box[2]), big.NewRat(tiles, 1))
	us, vs := make([]*big.Rat, tiles+1), make([]*big.Rat, tiles+1)
	for i := range us {
		us[i] = ratAdd(box[0], ratMul(big.NewRat(int64(i), 1), widthU))
		vs[i] = ratAdd(box[2], ratMul(big.NewRat(int64(i), 1), widthV))
	}
	if err := ctx.Err(); err != nil {
		return 0, 0, 0, err
	}
	firstGrid, err := gateGrid(first, rows, us, vs)
	if err != nil {
		return 0, 0, 0, err
	}
	secondGrid, err := gateGrid(second, rows, us, vs)
	if err != nil {
		return 0, 0, 0, err
	}
	worst := legacyGateWorst(firstGrid, secondGrid, tiles, len(rows))
	if worst.Cmp(ratOne()) >= 0 {
		return RatUp(worst), 0, det.Sign(),
			fmt.Errorf("%w: loft band has no global source-map injectivity bound (norm %.6g)",
				decaderr.ErrUnsupported, RatUp(worst))
	}
	jac := ratMul(ratAbs(det), ratMul(ratAdd(ratOne(), worst),
		ratMul(ratAdd(ratOne(), worst), ratAdd(ratOne(), worst))))
	return RatUp(worst), RatUp(jac), det.Sign(), nil
}

// gateTestGrids builds both maps' node grids over box with n tiles per axis,
// the way BandGate does. It returns the grid error, if any.
func gateTestGrids(first, second Map, box [4]*big.Rat, n int) ([][]gateNode, [][]gateNode, error) {
	centerU := ratDiv(ratAdd(box[0], box[1]), big.NewRat(2, 1))
	centerV := ratDiv(ratAdd(box[2], box[3]), big.NewRat(2, 1))
	c0 := VScale(VAdd(first.Deriv(centerU, centerV, 0), second.Deriv(centerU, centerV, 0)),
		big.NewRat(1, 2))
	c1 := VScale(VAdd(first.Deriv(centerU, centerV, 1), second.Deriv(centerU, centerV, 1)),
		big.NewRat(1, 2))
	c2 := VSub(second.At(centerU, centerV), first.At(centerU, centerV))
	rows, _, err := InverseRows(c0, c1, c2)
	if err != nil {
		return nil, nil, err
	}
	us, vs := gateAxisGrid(box[0], box[1], n), gateAxisGrid(box[2], box[3], n)
	fg, err := gateGrid(first, rows, us, vs)
	if err != nil {
		return nil, nil, err
	}
	sg, err := gateGrid(second, rows, us, vs)
	if err != nil {
		return nil, nil, err
	}
	return fg, sg, nil
}

const gateFamilies = 9

// gateFamilyBand draws a band of one family. Families 0-1 are the generic
// maps; 2 has derivative numerators that change sign inside tiles; 3 and 4
// scale every coefficient by 2^±40; 5 lets g vary by a factor of 9 over the
// box; 6 is independent of v, so whole tile columns tie exactly; 7 and 8
// scale by 2^-1000 and 2^300, outside the float filter's range. When nearby
// is set the second map is the first with a small shift, so the gate can pass.
func gateFamilyBand(rng *rand.Rand, family int, nearby bool) (Map, Map, [4]*big.Rat) {
	u0, u1, v0, v1 := gateTestBox(rng)
	box := [4]*big.Rat{u0, u1, v0, v1}
	var first Map
	switch family {
	case 0:
		first = randomGateMap(rng)
	case 1:
		first = wideGateMap(rng)
	case 2:
		first = wideGateMap(rng)
		// a0 = G0 rowQU - GU rowQ0 crosses zero inside the box.
		first.GU, first.GV = ratF(0.9), ratF(-0.9)
		first.Q0 = gateCoefficientVec(rng, 8)
	case 3:
		first = scaledGateMap(rng, 40, 0)
	case 4:
		first = scaledGateMap(rng, -40, -40)
	case 5:
		first = wideGateMap(rng)
		first.G0 = ratF(5)
		first.GU, first.GV = ratF(2.0), ratF(-2.0)
		box = [4]*big.Rat{ratF(-1), ratF(1), ratF(-1), ratF(1)}
	case 6:
		first = wideGateMap(rng)
		first.QV = RatVec{new(big.Rat), new(big.Rat), new(big.Rat)}
		first.GV = new(big.Rat)
	case 7:
		first = scaledGateMap(rng, -1000, -1000)
	default:
		first = scaledGateMap(rng, 300, 300)
	}
	second := first
	if nearby {
		second.Constant = RatVec{ratAdd(first.Constant[0], ratF(1)), first.Constant[1], first.Constant[2]}
		second.QU = RatVec{first.QU[0], ratAdd(first.QU[1], ratF(0.01)), first.QU[2]}
	} else {
		second = wideGateMap(rng)
		if family == 6 {
			second.QV = RatVec{new(big.Rat), new(big.Rat), new(big.Rat)}
			second.GV = new(big.Rat)
		}
	}
	return first, second, box
}

func TestGateTileUpperCoversDeviation(t *testing.T) {
	rng := rand.New(rand.NewPCG(151, 157))
	const n = 3
	finite, infinite, built := 0, 0, 0
	for trial := range gateFamilies {
		family := trial % gateFamilies
		first, second, box := gateFamilyBand(rng, family, trial%2 == 0)
		fg, sg, err := gateTestGrids(first, second, box, n)
		if err != nil {
			continue
		}
		built++
		fe, se := gateGridEnc(fg), gateGridEnc(sg)
		for iu := range n {
			for iv := range n {
				f := [4]*gateNodeEnc{&fe[iu][iv], &fe[iu][iv+1], &fe[iu+1][iv], &fe[iu+1][iv+1]}
				s := [4]*gateNodeEnc{&se[iu][iv], &se[iu][iv+1], &se[iu+1][iv], &se[iu+1][iv+1]}
				fn := [4]*gateNode{&fg[iu][iv], &fg[iu][iv+1], &fg[iu+1][iv], &fg[iu+1][iv+1]}
				sn := [4]*gateNode{&sg[iu][iv], &sg[iu][iv+1], &sg[iu+1][iv], &sg[iu+1][iv+1]}
				for row := range 3 {
					upper := gateTileUpper(f, s, row)
					if math.IsInf(upper, 1) {
						infinite++
						continue
					}
					require.False(t, math.IsNaN(upper))
					finite++
					exact := gateTileDeviation(fn, sn, row)
					require.GreaterOrEqual(t, proofarith.FloatRat(upper).Cmp(exact), 0,
						"family %d tile (%d,%d) row %d: bound %g below exact %s", family, iu, iv, row, upper,
						exact.FloatString(20))
				}
			}
		}
	}
	require.Positive(t, built)
	require.Positive(t, finite, "some tile row must get a finite bound")
	require.Positive(t, infinite, "the out-of-range families must fall back")
}

// TestGateTileUpperTightOnSmoothBand shows a finite bound is close to the
// exact deviation on a well-conditioned band, so the filter does not pass by
// returning +Inf or huge bounds.
func TestGateTileUpperTightOnSmoothBand(t *testing.T) {
	rng := rand.New(rand.NewPCG(173, 179))
	first, second, box := gateFamilyBand(rng, 0, true)
	fg, sg, err := gateTestGrids(first, second, box, 4)
	require.NoError(t, err)
	fe, se := gateGridEnc(fg), gateGridEnc(sg)
	f := [4]*gateNodeEnc{&fe[1][1], &fe[1][2], &fe[2][1], &fe[2][2]}
	s := [4]*gateNodeEnc{&se[1][1], &se[1][2], &se[2][1], &se[2][2]}
	fn := [4]*gateNode{&fg[1][1], &fg[1][2], &fg[2][1], &fg[2][2]}
	sn := [4]*gateNode{&sg[1][1], &sg[1][2], &sg[2][1], &sg[2][2]}
	for row := range 3 {
		upper := gateTileUpper(f, s, row)
		exact, _ := gateTileDeviation(fn, sn, row).Float64()
		require.InDelta(t, exact, upper, 1e-9*max(1, exact), "row %d", row)
	}
}

// TestGateTileUpperFallsBack pokes NaN, infinities and an unusable node into
// valid enclosures; every tile bound that reads one must be +Inf.
func TestGateTileUpperFallsBack(t *testing.T) {
	rng := rand.New(rand.NewPCG(181, 191))
	first, second, box := gateFamilyBand(rng, 0, true)
	fg, sg, err := gateTestGrids(first, second, box, 2)
	require.NoError(t, err)
	build := func() ([4]*gateNodeEnc, [4]*gateNodeEnc) {
		fe, se := gateGridEnc(fg), gateGridEnc(sg)
		return [4]*gateNodeEnc{&fe[0][0], &fe[0][1], &fe[1][0], &fe[1][1]},
			[4]*gateNodeEnc{&se[0][0], &se[0][1], &se[1][0], &se[1][1]}
	}
	f, s := build()
	for row := range 3 {
		require.False(t, math.IsInf(gateTileUpper(f, s, row), 1), "unpoked tile must have a finite bound")
	}
	nan, inf := math.NaN(), math.Inf(1)
	pokes := map[string]func(p *[2][4]*gateNodeEnc){
		"deriv NaN":      func(p *[2][4]*gateNodeEnc) { p[0][2].deriv[0][0] = gateEnc{nan, nan} },
		"deriv NaN hi":   func(p *[2][4]*gateNodeEnc) { p[1][1].deriv[0][1].hi = nan },
		"scalar NaN":     func(p *[2][4]*gateNodeEnc) { p[1][3].scalar[0] = gateEnc{nan, nan} },
		"scalar Inf":     func(p *[2][4]*gateNodeEnc) { p[0][0].scalar[0] = gateEnc{-inf, inf} },
		"deriv Inf both": func(p *[2][4]*gateNodeEnc) { p[0][1].deriv[0][0] = gateEnc{-inf, inf} },
		"g NaN":          func(p *[2][4]*gateNodeEnc) { p[0][1].g = gateEnc{nan, nan} },
		"g not positive": func(p *[2][4]*gateNodeEnc) { p[1][0].g = gateEnc{-1, 1} },
		"node not ok":    func(p *[2][4]*gateNodeEnc) { p[1][2].ok = false },
		"first not ok":   func(p *[2][4]*gateNodeEnc) { p[0][3].ok = false },
	}
	for name, poke := range pokes {
		f, s := build()
		pair := [2][4]*gateNodeEnc{f, s}
		poke(&pair)
		got := gateTileUpper(pair[0], pair[1], 0)
		require.True(t, math.IsInf(got, 1), "%s: got %g", name, got)
	}
}

func TestGateEncOfRange(t *testing.T) {
	huge := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 2000))
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 2000))
	for name, x := range map[string]*big.Rat{"huge": huge, "tiny": tiny, "negative huge": new(big.Rat).Neg(huge),
		"above 2^256": ratF(0x1p257), "below 2^-256": ratF(0x1p-257),
		"underflows to zero": new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 1076)),
		"subnormal":          ratF(0x1p-1074)} {
		_, ok := gateEncOf(x)
		require.False(t, ok, name)
	}
	for name, x := range map[string]*big.Rat{"zero": new(big.Rat), "one": ratOne(), "third": big.NewRat(1, 3),
		"edge": ratF(0x1p256), "low edge": ratF(0x1p-256), "negative": big.NewRat(-7, 3)} {
		e, ok := gateEncOf(x)
		require.True(t, ok, name)
		require.GreaterOrEqual(t, x.Cmp(proofarith.FloatRat(e.lo)), 0, name)
		require.LessOrEqual(t, x.Cmp(proofarith.FloatRat(e.hi)), 0, name)
	}
	// An inexact value is enclosed strictly: lo < x < hi.
	e, ok := gateEncOf(big.NewRat(1, 3))
	require.True(t, ok)
	require.Positive(t, big.NewRat(1, 3).Cmp(proofarith.FloatRat(e.lo)))
	require.Negative(t, big.NewRat(1, 3).Cmp(proofarith.FloatRat(e.hi)))
}

func TestBandGateFilterMatchesLegacy(t *testing.T) {
	rng := rand.New(rand.NewPCG(193, 197))
	matched, refused, passed, gridErr := 0, 0, 0, 0
	// Two bands keep the cost down: a generic nearby band and a wide one.
	// Every family, including the out-of-range scales,
	// is covered tile by tile in the tests below.
	for trial, family := range []int{0, 1} {
		first, second, box := gateFamilyBand(rng, family, trial%2 == 0)
		if trial%7 == 3 {
			// Scale the second map's numerators so the norm passes 1: refusals.
			second.QU = RatVec{ratMul(second.QU[0], ratF(50)), second.QU[1], second.QU[2]}
		}
		wantRho, wantJac, wantSign, wantErr := legacyBandGate(t.Context(), first, second, box)
		gotRho, gotJac, gotSign, gotErr := BandGate(t.Context(), first, second, box)
		require.Equal(t, math.Float64bits(wantRho), math.Float64bits(gotRho), "family %d", family)
		require.Equal(t, math.Float64bits(wantJac), math.Float64bits(gotJac), "family %d", family)
		require.Equal(t, wantSign, gotSign)
		matched++
		if wantErr == nil {
			require.NoError(t, gotErr)
			passed++
			continue
		}
		require.EqualError(t, gotErr, wantErr.Error())
		if _, _, gerr := gateTestGrids(first, second, box, 16); gerr != nil {
			gridErr++
			continue
		}
		refused++
	}
	require.Positive(t, passed, "some band must pass the gate")
	require.Positive(t, refused, "some band must be refused by its norm")
	t.Logf("%d bands: %d passed, %d refused by norm, %d refused at the grid", matched, passed, refused, gridErr)
}

// requireGateWorstEqual compares gateWorst with the unfiltered loop by value
// (Num and Denom, equal for equal values) and returns the exact-evaluation
// count.
func requireGateWorstEqual(t *testing.T, fg, sg [][]gateNode, tiles int, what string) int {
	t.Helper()
	want := legacyGateWorst(fg, sg, tiles, 3)
	got, count, err := gateWorstCount(t.Context(), fg, sg, tiles, 3)
	require.NoError(t, err, what)
	require.Zero(t, want.Num().Cmp(got.Num()), "%s numerator: want %s got %s", what, want, got)
	require.Zero(t, want.Denom().Cmp(got.Denom()), "%s denominator", what)
	return count
}

func TestGateWorstMatchesLegacyOnBands(t *testing.T) {
	rng := rand.New(rand.NewPCG(199, 211))
	const n = 3
	built := 0
	for trial := range gateFamilies {
		family := trial % gateFamilies
		first, second, box := gateFamilyBand(rng, family, trial%2 == 0)
		fg, sg, err := gateTestGrids(first, second, box, n)
		if err != nil {
			continue
		}
		built++
		requireGateWorstEqual(t, fg, sg, n, fmt.Sprintf("family %d trial %d", family, trial))
	}
	require.Positive(t, built)
}

// gateSynthPool draws node values for synthetic grids: values chosen to tie,
// nearly tie (one ulp apart, inexact as floats), cross zero, or fall outside
// the filter's float range.
type gateSynthPool struct {
	rng  *rand.Rand
	mode int
}

func (p gateSynthPool) value() *big.Rat {
	third := func(k int64) *big.Rat { return big.NewRat(k, 3) }
	switch p.mode {
	case 0: // every node identical: every tile row ties exactly.
		return third(1)
	case 1: // within a few ulps of 1/3, as exact rationals above and below it.
		x := proofarith.FloatRat(1.0 / 3)
		step := ratF(0x1p-54 * float64(p.rng.IntN(5)-2))
		return ratAdd(x, step)
	case 2: // sign changes, zeros, dyadic and thirds.
		switch p.rng.IntN(4) {
		case 0:
			return new(big.Rat)
		case 1:
			return third(int64(p.rng.IntN(25) - 12))
		default:
			return ratF((p.rng.Float64()*2 - 1) * 4)
		}
	case 3: // huge and tiny magnitudes mixed with unit ones.
		k := []int{-2000, -300, -1, 0, 1, 300, 2000}[p.rng.IntN(7)]
		return ratMul(ratF((p.rng.Float64()*2-1)*4), new(big.Rat).SetFrac(
			new(big.Int).Lsh(big.NewInt(1), uint(max(k, 0))), new(big.Int).Lsh(big.NewInt(1), uint(max(-k, 0)))))
	default: // dyadic values within an ulp-scale spread of one another.
		return ratAdd(ratF(0.5), ratF(0x1p-52*float64(p.rng.IntN(3))))
	}
}

func (p gateSynthPool) grid(tiles int) [][]gateNode {
	grid := make([][]gateNode, tiles+1)
	for i := range grid {
		grid[i] = make([]gateNode, tiles+1)
		for j := range grid[i] {
			n := gateNode{g: ratAdd(ratOne(), ratF(float64(p.rng.IntN(8))/4))}
			if p.mode == 3 && p.rng.IntN(3) == 0 {
				n.g = ratMul(n.g, ratF(0x1p300))
			}
			for r := range 3 {
				n.deriv[r][0], n.deriv[r][1], n.scalar[r] = p.value(), p.value(), p.value()
			}
			grid[i][j] = n
		}
	}
	return grid
}

func TestGateWorstMatchesLegacySynthetic(t *testing.T) {
	rng := rand.New(rand.NewPCG(223, 227))
	const tiles = 3
	for trial := range 100 {
		pool := gateSynthPool{rng: rng, mode: trial % 5}
		fg, sg := pool.grid(tiles), pool.grid(tiles)
		if pool.mode == 0 {
			sg = fg
		}
		requireGateWorstEqual(t, fg, sg, tiles, fmt.Sprintf("mode %d trial %d", pool.mode, trial))
	}
}

// TestGateWorstEveryTileTies uses a band whose every node is identical, so
// every tile row has the same deviation and the maximum is attained everywhere.
func TestGateWorstEveryTileTies(t *testing.T) {
	rng := rand.New(rand.NewPCG(229, 233))
	const tiles = 4
	pool := gateSynthPool{rng: rng, mode: 0}
	fg := pool.grid(tiles)
	for i := range fg {
		for j := range fg[i] {
			fg[i][j].g = ratOne()
		}
	}
	requireGateWorstEqual(t, fg, fg, tiles, "all tie")
}

// TestGateWorstSingleMaximumTile makes one grid corner large, so only the one
// tile touching it attains the maximum; the float pass must reach it however
// the other tiles' bounds order.
func TestGateWorstSingleMaximumTile(t *testing.T) {
	rng := rand.New(rand.NewPCG(239, 241))
	const tiles = 4
	for trial := range 8 {
		pool := gateSynthPool{rng: rng, mode: 4}
		fg, sg := pool.grid(tiles), pool.grid(tiles)
		i, j := (trial%2)*tiles, ((trial/2)%2)*tiles
		for r := range 3 {
			sg[i][j].scalar[r] = ratAdd(sg[i][j].scalar[r], ratF(8+rng.Float64()))
			sg[i][j].deriv[r][r%2] = ratAdd(sg[i][j].deriv[r][r%2], ratF(float64(trial%3)))
		}
		count := requireGateWorstEqual(t, fg, sg, tiles, fmt.Sprintf("corner (%d,%d) trial %d", i, j, trial))
		require.Less(t, count, tiles*tiles*3, "the filter must skip some tile rows")
	}
}

func TestGateWorstSkipsMostTiles(t *testing.T) {
	rng := rand.New(rand.NewPCG(251, 257))
	box := [4]*big.Rat{ratF(-1.25), ratF(2.5), ratF(-0.75), ratF(1.5)}
	const tiles = 16
	evaluated, total := 0, 0
	for range 2 {
		first, second := randomGateMap(rng), randomGateMap(rng)
		fg, sg, err := gateTestGrids(first, second, box, tiles)
		require.NoError(t, err)
		_, count, err := gateWorstCount(t.Context(), fg, sg, tiles, 3)
		require.NoError(t, err)
		evaluated += count
		total += tiles * tiles * 3
	}
	require.LessOrEqual(t, evaluated*4, total, "evaluated %d of %d tile rows", evaluated, total)
	t.Logf("evaluated %d of %d tile rows", evaluated, total)
}

func TestGateWorstCancelled(t *testing.T) {
	rng := rand.New(rand.NewPCG(263, 269))
	first, second, box := gateFamilyBand(rng, 0, true)
	fg, sg, err := gateTestGrids(first, second, box, 4)
	require.NoError(t, err)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = gateWorst(ctx, fg, sg, 4, 3)
	require.ErrorIs(t, err, context.Canceled)
}

// lateCancelCtx reports no error for its first quiet calls to Err and
// context.Canceled afterwards.
type lateCancelCtx struct {
	context.Context //nolint:containedctx // a test double that wraps the test context
	calls, quiet    int
}

func (c *lateCancelCtx) Err() error {
	c.calls++
	if c.calls > c.quiet {
		return context.Canceled
	}
	return nil
}

// TestGateWorstCancelledAfterLastPoll holds the cancellation back until every
// poll inside the passes has run: the call must still return context.Canceled
// rather than a result, at the early stop as well as at the end.
func TestGateWorstCancelledAfterLastPoll(t *testing.T) {
	rng := rand.New(rand.NewPCG(311, 313))
	first, second, box := gateFamilyBand(rng, 0, true)
	fg, sg, err := gateTestGrids(first, second, box, 4)
	require.NoError(t, err)
	_, count, err := gateWorstCount(t.Context(), fg, sg, 4, 3)
	require.NoError(t, err)
	require.Less(t, count, 4*4*3, "the pass must stop early")
	// One poll per tile column in the float pass, one per 16 exact evaluations.
	polls := 4 + (count+15)/16
	late := &lateCancelCtx{Context: t.Context(), quiet: polls}
	_, _, err = gateWorstCount(late, fg, sg, 4, 3)
	require.ErrorIs(t, err, context.Canceled)
}

func benchmarkGate(b *testing.B, gate func(context.Context, Map, Map, [4]*big.Rat) (float64, float64, int, error)) {
	rng := rand.New(rand.NewPCG(271, 277))
	box := [4]*big.Rat{ratF(-1.25), ratF(2.5), ratF(-0.75), ratF(1.5)}
	first, second := randomGateMap(rng), randomGateMap(rng)
	b.ResetTimer()
	for range b.N {
		if _, _, _, err := gate(b.Context(), first, second, box); err != nil && !errors.Is(err, decaderr.ErrUnsupported) {
			b.Fatal(err)
		}
	}
}

func BenchmarkBandGateFiltered(b *testing.B) { benchmarkGate(b, BandGate) }
func BenchmarkBandGateLegacy(b *testing.B)   { benchmarkGate(b, legacyBandGate) }

// randGateEnc draws a float enclosure [lo, hi] of magnitude about 2^e, one to
// a few ulps wide, or a single exact float.
func randGateEnc(rng *rand.Rand, e int, positive bool) gateEnc {
	x := (rng.Float64()*2 - 1) * math.Ldexp(1, e)
	if positive {
		x = (rng.Float64()*3 + 0.25) * math.Ldexp(1, e)
	}
	lo, hi := x, x
	for range rng.IntN(4) {
		hi = math.Nextafter(hi, math.Inf(1))
	}
	if rng.IntN(3) == 0 {
		lo = math.Nextafter(lo, math.Inf(-1))
	}
	return gateEnc{lo, hi}
}

func randGateNodeEnc(rng *rand.Rand, e int) gateNodeEnc {
	n := gateNodeEnc{ok: true, g: randGateEnc(rng, e, true)}
	for r := range 3 {
		for a := range 2 {
			n.deriv[r][a] = randGateEnc(rng, e, false)
			switch {
			case n.deriv[r][a].lo > 0:
				n.sign[r][a] = 1
			case n.deriv[r][a].hi < 0:
				n.sign[r][a] = -1
			default:
				// An enclosure holding zero: the exact value may have either sign.
				n.sign[r][a] = rng.IntN(3) - 1
			}
		}
		n.scalar[r] = randGateEnc(rng, e, false)
	}
	return n
}

func ratSq(x float64) *big.Rat { return ratMul(ratF(x), ratF(x)) }

// TestGateDerivBoundMatchesExactInfimum compares the float bounds of one
// derivative range with the infimum and supremum computed in exact rationals
// from the same enclosure ends. Each rounding step is checked by itself here,
// without the slack later steps add to a whole tile's bound.
func TestGateDerivBoundMatchesExactInfimum(t *testing.T) {
	rng := rand.New(rand.NewPCG(281, 283))
	for trial := range 3000 {
		e := []int{-30, -3, 0, 2, 30}[trial%5]
		var nodes [4]gateNodeEnc
		var c [4]*gateNodeEnc
		for k := range nodes {
			nodes[k] = randGateNodeEnc(rng, e)
			c[k] = &nodes[k]
		}
		row, axis := trial%3, (trial/3)%2
		lo, hi, ok := gateDerivBound(c, row, axis)
		require.True(t, ok)
		gminLo, gminHi, gmaxLo, gmaxHi := c[0].g.lo, c[0].g.hi, c[0].g.lo, c[0].g.hi
		nminLo, nmaxHi := c[0].deriv[row][axis].lo, c[0].deriv[row][axis].hi
		neg, pos := c[0].sign[row][axis] < 0, c[0].sign[row][axis] > 0
		for _, k := range c[1:] {
			gminLo, gminHi = min(gminLo, k.g.lo), min(gminHi, k.g.hi)
			gmaxLo, gmaxHi = max(gmaxLo, k.g.lo), max(gmaxHi, k.g.hi)
			nminLo, nmaxHi = min(nminLo, k.deriv[row][axis].lo), max(nmaxHi, k.deriv[row][axis].hi)
			neg, pos = neg || k.sign[row][axis] < 0, pos || k.sign[row][axis] > 0
		}
		loDenLo, loDenHi := ratSq(gmaxLo), ratSq(gmaxHi)
		if neg {
			loDenLo, loDenHi = ratSq(gminLo), ratSq(gminHi)
		}
		hiDenLo, hiDenHi := ratSq(gmaxLo), ratSq(gmaxHi)
		if pos {
			hiDenLo, hiDenHi = ratSq(gminLo), ratSq(gminHi)
		}
		// Infimum of nmin/loDen: a non-negative numerator over the larger
		// denominator, a negative one over the smaller.
		infDen := loDenLo
		if nminLo >= 0 {
			infDen = loDenHi
		}
		inf := ratDiv(ratF(nminLo), infDen)
		supDen := hiDenHi
		if nmaxHi >= 0 {
			supDen = hiDenLo
		}
		sup := ratDiv(ratF(nmaxHi), supDen)
		require.LessOrEqual(t, proofarith.FloatRat(lo).Cmp(inf), 0, "trial %d low bound %g", trial, lo)
		require.GreaterOrEqual(t, proofarith.FloatRat(hi).Cmp(sup), 0, "trial %d high bound %g", trial, hi)
	}
}

func TestGateAbsAndVelocityBoundsRoundOutward(t *testing.T) {
	rng := rand.New(rand.NewPCG(293, 307))
	for trial := range 3000 {
		e := []int{-30, 0, 30}[trial%3]
		flo, fhi := (rng.Float64()*2-1)*math.Ldexp(1, e), (rng.Float64()*2-1)*math.Ldexp(1, e)
		for _, c := range []float64{0, 1} {
			got := gateAbsBound(flo, fhi, c)
			want := new(big.Rat).Sub(ratF(fhi), ratF(c))
			if other := new(big.Rat).Sub(ratF(c), ratF(flo)); other.Cmp(want) > 0 {
				want = other
			}
			if want.Sign() < 0 {
				want = ratZero()
			}
			require.GreaterOrEqual(t, proofarith.FloatRat(got).Cmp(want), 0, "abs bound trial %d c %g", trial, c)
		}

		var nodes [4]gateNodeEnc
		var first, second [4]*gateNodeEnc
		var other [4]gateNodeEnc
		for k := range nodes {
			nodes[k], other[k] = randGateNodeEnc(rng, e), randGateNodeEnc(rng, e)
			first[k], second[k] = &nodes[k], &other[k]
		}
		row := trial % 3
		vLo, vHi := gateVelocityBound(first, second, row)
		sLo, sHi := other[0].scalar[row].lo, other[0].scalar[row].hi
		fLo, fHi := nodes[0].scalar[row].lo, nodes[0].scalar[row].hi
		for k := 1; k < 4; k++ {
			sLo, sHi = min(sLo, other[k].scalar[row].lo), max(sHi, other[k].scalar[row].hi)
			fLo, fHi = min(fLo, nodes[k].scalar[row].lo), max(fHi, nodes[k].scalar[row].hi)
		}
		require.LessOrEqual(t, proofarith.FloatRat(vLo).Cmp(ratSub(ratF(sLo), ratF(fHi))), 0, "velocity low trial %d", trial)
		require.GreaterOrEqual(t, proofarith.FloatRat(vHi).Cmp(ratSub(ratF(sHi), ratF(fLo))), 0, "velocity high trial %d", trial)

		// The two sums of the three terms round upward.
		terms, ok := gateTileTerms(first, second, row)
		require.True(t, ok)
		upper := gateTileUpper(first, second, row)
		sum := ratAdd(ratAdd(ratF(terms[0]), ratF(terms[1])), ratF(terms[2]))
		require.GreaterOrEqual(t, proofarith.FloatRat(upper).Cmp(sum), 0, "sum trial %d", trial)
	}
}
