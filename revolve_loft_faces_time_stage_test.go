package decad

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/stretchr/testify/require"
)

// legacyRevolveLoftFacesBracketTimeRoot is revolveLoftFacesBracketTimeRoot as
// it was when every restriction repeated the band-time stage on q.
func legacyRevolveLoftFacesBracketTimeRoot(q, j [3][]*big.Rat,
	segment, span, band int, tLo, tHi *big.Rat) ([]revolveLoftFacesMappedCell, bool) {
	box := polynomial.BernsteinRectRestrict(q, lfZero(), lfOne(), tLo, tHi)
	for _, orientation := range []int{1, -1} {
		oriented := box
		if orientation < 0 {
			oriented = loftface.NegateRows(box)
		}
		if polynomial.BernsteinRowsPointSign(oriented, lfZero()) != -1 ||
			polynomial.BernsteinRowsPointSign(oriented, lfOne()) != 1 {
			continue
		}
		sLo, sHi := lfZero(), lfOne()
		for range 10 {
			mid := lfMul(lfAdd(sLo, sHi), big.NewRat(1, 2))
			switch polynomial.BernsteinRowsPointSign(oriented, mid) {
			case -1:
				sLo = mid
			case 1:
				sHi = mid
			default:
				goto check
			}
		}
	check:
		middle := polynomial.BernsteinRectRestrict(q, sLo, sHi, tLo, tHi)
		if orientation < 0 {
			middle = loftface.NegateRows(middle)
		}
		if !polynomial.BernsteinRowsCross(middle) {
			continue
		}
		if sLo.Sign() > 0 {
			left := polynomial.BernsteinRectRestrict(q, lfZero(), sLo, tLo, tHi)
			if orientation > 0 {
				left = loftface.NegateRows(left)
			}
			if !polynomial.BernsteinRowsPositive(left) {
				continue
			}
		}
		if sHi.Cmp(lfOne()) < 0 {
			right := polynomial.BernsteinRectRestrict(q, sHi, lfOne(), tLo, tHi)
			if orientation < 0 {
				right = loftface.NegateRows(right)
			}
			if !polynomial.BernsteinRowsPositive(right) {
				continue
			}
		}
		order := polynomial.BernsteinRectRestrict(j, sLo, sHi, tLo, tHi)
		if orientation < 0 {
			order = loftface.NegateRows(order)
		}
		xDirection := 0
		if polynomial.BernsteinRowsPositive(order) {
			xDirection = 1
		} else if polynomial.BernsteinRowsPositive(loftface.NegateRows(order)) {
			xDirection = -1
		}
		if xDirection == 0 {
			continue
		}
		makeCell := func(side int, a, z *big.Rat) revolveLoftFacesMappedCell {
			return revolveLoftFacesMappedCell{segment: segment, span: span,
				band: band, side: side, sLo: a, sHi: z, tLo: tLo, tHi: tHi}
		}
		var cells []revolveLoftFacesMappedCell
		if sLo.Sign() > 0 {
			cells = append(cells, makeCell(-orientation, lfZero(), sLo))
		}
		crossing := makeCell(loftface.SideCrossed, sLo, sHi)
		crossing.xDirection = xDirection
		cells = append(cells, crossing)
		if sHi.Cmp(lfOne()) < 0 {
			cells = append(cells, makeCell(orientation, sHi, lfOne()))
		}
		return cells, true
	}
	return nil, false
}

// legacyRevolveLoftFacesKnotRootCell is revolveLoftFacesKnotRootCellWith as it
// was when every restriction repeated the band-time stage on q.
func legacyRevolveLoftFacesKnotRootCell(trim *loftFacesTrimRecord,
	section revolveSectionSpan, span freeform.BezierSpan, band int,
	atEnd bool, root loftface.EndpointRoot) (revolveLoftFacesJointCell, bool) {
	var empty revolveLoftFacesJointCell
	if trim == nil || band < 0 || band+1 >= len(trim.Maps) ||
		root.Time.Lo == nil || root.Time.Hi == nil {
		return empty, false
	}
	m, b := revolveLoftFacesMeridian(section)
	q := loftface.MeridianQRows(trim.Maps[band], trim.Maps[band+1], span, m, b)
	endpoint := lfZero()
	if atEnd {
		endpoint = lfOne()
	}
	endQ := []*big.Rat{
		polynomial.BernsteinValue(q[0], endpoint),
		polynomial.BernsteinValue(q[1], endpoint),
		polynomial.BernsteinValue(q[2], endpoint),
	}
	loSign, hiSign := polynomial.BernsteinSign(endQ, root.Time.Lo),
		polynomial.BernsteinSign(endQ, root.Time.Hi)
	if root.Time.Lo.Cmp(root.Time.Hi) == 0 {
		if loSign != 0 ||
			polynomial.BernsteinSign(polynomial.BernsteinDeriv(endQ), root.Time.Lo) != root.Direction {
			return empty, false
		}
	} else if loSign != -root.Direction || hiSign != root.Direction {
		return empty, false
	}
	startX, endX := lfF(section.start.U), lfF(section.end.U)
	if startX.Cmp(endX) > 0 {
		startX, endX = endX, startX
	}
	for depth := range 13 {
		width := big.NewRat(1, int64(1<<depth))
		sLo, sHi := lfZero(), width
		if atEnd {
			sLo, sHi = lfSub(lfOne(), width), lfOne()
		}
		box := polynomial.BernsteinRectRestrict(q, sLo, sHi,
			root.Time.Lo, root.Time.Hi)

		var derivative [3][]*big.Rat
		for row := range box {
			derivative[row] = polynomial.BernsteinDeriv(box[row])
		}
		direction := 0
		if polynomial.BernsteinRowsPositive(derivative) {
			direction = 1
		} else if polynomial.BernsteinRowsPositive(loftface.NegateRows(derivative)) {
			direction = -1
		}
		if direction == 0 {
			continue
		}
		fixed := lfZero()
		if !atEnd {
			fixed = lfOne()
		}
		fixedSide := polynomial.BernsteinRowsPointSign(box, fixed)
		if fixedSide == 0 {
			continue
		}
		if depth > 0 {
			outerLo, outerHi := lfZero(), sLo
			if !atEnd {
				outerLo, outerHi = sHi, lfOne()
			}
			outer := polynomial.BernsteinRectRestrict(q, outerLo, outerHi,
				root.Time.Lo, root.Time.Hi)

			if fixedSide > 0 && !polynomial.BernsteinRowsPositive(outer) ||
				fixedSide < 0 && !polynomial.BernsteinRowsPositive(loftface.NegateRows(outer)) {
				continue
			}
		}
		x, ok := loftface.SpanXRange(trim.Maps[band], trim.Maps[band+1],
			span, sLo, sHi, root.Time.Lo, root.Time.Hi)
		if !ok || root.Finite && (x.Lo.Cmp(startX) <= 0 || x.Hi.Cmp(endX) >= 0) ||
			!root.Finite && x.Hi.Cmp(startX) >= 0 && x.Lo.Cmp(endX) <= 0 {
			continue
		}
		return revolveLoftFacesJointCell{sLo: sLo, sHi: sHi,
			time: root.Time, outsideSide: fixedSide,
			sDerivative: direction, x: x, finite: root.Finite}, true
	}
	return empty, false
}

// timeStageInterval draws lo < hi in [0, 1]; each end is exactly 0 or 1 in a
// quarter of the draws, dyadic in 40 %, and otherwise not dyadic.
func timeStageInterval(rng *rand.Rand) (*big.Rat, *big.Rat) {
	for {
		end := func(exact *big.Rat) *big.Rat {
			switch d := rng.IntN(20); {
			case d < 5:
				return exact
			case d < 13:
				return big.NewRat(rng.Int64N(1<<8), 1<<8)
			default:
				q := rng.Int64N(97) + 3
				return big.NewRat(rng.Int64N(q+1), q)
			}
		}
		lo, hi := end(new(big.Rat)), end(big.NewRat(1, 1))
		if lo.Cmp(hi) < 0 {
			return lo, hi
		}
	}
}

func timeStageSmallRat(rng *rand.Rand, scale int64) *big.Rat {
	return big.NewRat(rng.Int64N(2*scale+1)-scale, rng.Int64N(7)+1)
}

func requireMappedCellsSame(t *testing.T, want, got []revolveLoftFacesMappedCell) {
	t.Helper()
	require.Len(t, got, len(want))
	for i := range want {
		w, g := want[i], got[i]
		require.Equal(t, w.segment, g.segment)
		require.Equal(t, w.span, g.span)
		require.Equal(t, w.band, g.band)
		require.Equal(t, w.side, g.side)
		require.Equal(t, w.xDirection, g.xDirection)
		require.Zero(t, w.sLo.Cmp(g.sLo), "cell %d sLo", i)
		require.Zero(t, w.sHi.Cmp(g.sHi), "cell %d sHi", i)
		require.Zero(t, w.tLo.Cmp(g.tLo), "cell %d tLo", i)
		require.Zero(t, w.tHi.Cmp(g.tHi), "cell %d tHi", i)
	}
}

func requireTimeStageBracketRootsMatch(t *testing.T) {
	t.Helper()
	rng := rand.New(rand.NewPCG(97, 101))
	var found, missed int
	for trial := range 400 {
		var q, j [3][]*big.Rat
		length, jLength := 3+rng.IntN(5), 2+rng.IntN(4)
		for row := range q {
			amplitude := big.NewRat(rng.Int64N(9)+1, rng.Int64N(4)+1)
			step := big.NewRat(rng.Int64N(5)+1, rng.Int64N(4)+1)
			q[row] = make([]*big.Rat, length)
			for k := range q[row] {
				c := new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(int64(k), 1), step), amplitude)
				q[row][k] = c.Add(c, big.NewRat(rng.Int64N(7)-3, rng.Int64N(9)+4))
			}
			j[row] = make([]*big.Rat, jLength)
			for k := range j[row] {
				switch trial % 3 {
				case 0:
					j[row][k] = big.NewRat(rng.Int64N(5)+1, rng.Int64N(4)+1)
				case 1:
					j[row][k] = big.NewRat(-rng.Int64N(5)-1, rng.Int64N(4)+1)
				default:
					j[row][k] = timeStageSmallRat(rng, 5)
				}
			}
		}
		tLo, tHi := timeStageInterval(rng)
		wantCells, wantOK := legacyRevolveLoftFacesBracketTimeRoot(q, j, 3, 4, 5, tLo, tHi)
		gotCells, gotOK := revolveLoftFacesBracketTimeRoot(
			polynomial.BernsteinRectTimeRestrict(q, tLo, tHi), j, 3, 4, 5, tLo, tHi)
		require.Equal(t, wantOK, gotOK, "trial %d", trial)
		requireMappedCellsSame(t, wantCells, gotCells)
		if gotOK {
			found++
		} else {
			missed++
		}
	}
	require.GreaterOrEqual(t, found, 100)
	require.GreaterOrEqual(t, missed, 20)
}

func requireTimeStageKnotRootCellsMatch(t *testing.T) {
	t.Helper()
	rng := rand.New(rand.NewPCG(131, 137))
	mapAt := func(x, y int64, qv loftFaceRatVec) loftFaceMap {
		zero, one := lfZero(), lfOne()
		return loftFaceMap{
			Constant: loftFaceRatVec{zero, zero, zero},
			Q0:       loftFaceRatVec{big.NewRat(x, 10), big.NewRat(y, 1), zero},
			QU:       loftFaceRatVec{zero, one, zero},
			QV:       qv,
			G0:       one, GU: zero, GV: zero,
		}
	}
	var withCell int
	for trial := range 35 {
		qv := loftFaceRatVec{lfZero(), lfZero(), lfZero()}
		if rng.IntN(2) == 0 {
			qv = loftFaceRatVec{big.NewRat(rng.Int64N(41)-20, 10), big.NewRat(rng.Int64N(41)-20, 10), lfZero()}
		}
		trim := &loftFacesTrimRecord{Maps: []loftFaceMap{
			mapAt(rng.Int64N(9)+1, rng.Int64N(7)-3, qv),
			mapAt(rng.Int64N(9)+1, rng.Int64N(7)-3, qv),
			mapAt(rng.Int64N(9)+1, rng.Int64N(7)-3, qv),
		}}
		band := rng.IntN(2)
		var us [4]int64
		for i := range us {
			us[i] = rng.Int64N(31)
		}
		// Sorting keeps the span monotone in U.
		for i := range us {
			for k := i + 1; k < len(us); k++ {
				if us[k] < us[i] {
					us[i], us[k] = us[k], us[i]
				}
			}
		}
		var span freeform.BezierSpan
		for _, u := range us {
			span = append(span, freeform.RatPoint{U: big.NewRat(u, 30), V: lfZero()})
		}
		start := 0.05 + float64(rng.IntN(45))/100
		end := start + 0.02 + float64(rng.IntN(45))/100
		section := revolveSectionSpan{start: planePoint{U: start, V: 1.5}, end: planePoint{U: end, V: 1.5}}
		for _, atEnd := range []bool{false, true} {
			roots, ok := revolveLoftFacesEndpointFiniteRoots(trim, section, span, band, atEnd)
			if !ok {
				continue
			}
			// The isolated brackets are narrow. Widened copies keep the
			// root inside and make the time stage matter.
			var cases []loftface.EndpointRoot
			for _, root := range roots {
				cases = append(cases, root)
				for _, w := range []*big.Rat{big.NewRat(1, 64), big.NewRat(1, 8)} {
					wide := root
					wide.Time.Lo = loftface.RatMax(lfZero(), lfSub(root.Time.Lo, w))
					wide.Time.Hi = ratMinOf(lfOne(), lfAdd(root.Time.Hi, w))
					cases = append(cases, wide)
				}
			}
			for _, root := range cases {
				want, wantOK := legacyRevolveLoftFacesKnotRootCell(trim, section, span, band, atEnd, root)
				got, gotOK := revolveLoftFacesKnotRootCell(trim, section, span, band, atEnd, root)
				require.Equal(t, wantOK, gotOK, "trial %d atEnd %t", trial, atEnd)
				if !gotOK {
					continue
				}
				withCell++
				require.Equal(t, want.finite, got.finite)
				require.Equal(t, want.outsideSide, got.outsideSide)
				require.Equal(t, want.sDerivative, got.sDerivative)
				require.Zero(t, want.sLo.Cmp(got.sLo))
				require.Zero(t, want.sHi.Cmp(got.sHi))
				require.Zero(t, want.time.Lo.Cmp(got.time.Lo))
				require.Zero(t, want.time.Hi.Cmp(got.time.Hi))
				require.Zero(t, want.x.Lo.Cmp(got.x.Lo))
				require.Zero(t, want.x.Hi.Cmp(got.x.Hi))
			}
		}
	}
	require.GreaterOrEqual(t, withCell, 30)
}

// TestCurvedTimeStageCellsMatchLegacy compares the two routines that now share
// one band-time restriction across their source restrictions with verbatim
// copies of the code that repeated it. The curved fixture tests do not notice
// a wrong source range in the bracket-root search.
func TestCurvedTimeStageCellsMatchLegacy(t *testing.T) {
	t.Run("bracket root", requireTimeStageBracketRootsMatch)
	t.Run("knot root cell", requireTimeStageKnotRootCellsMatch)
}

func ratMinOf(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) < 0 {
		return a
	}
	return b
}
