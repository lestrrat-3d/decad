package crossbore

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type crossIV = proofarith.LazyInterval

func crossPoint(x float64) crossIV          { return proofarith.LazyPoint(proofarith.FloatRat(x)) }
func crossAdd(a, b crossIV) crossIV         { return proofarith.LazyIntervalAdd(a, b) }
func crossSub(a, b crossIV) crossIV         { return proofarith.LazyIntervalSub(a, b) }
func crossMul(a, b crossIV) crossIV         { return proofarith.LazyIntervalMul(a, b) }
func crossSq(a crossIV) crossIV             { return proofarith.LazyIntervalSquare(a) }
func crossQuo(a, b crossIV) (crossIV, bool) { return proofarith.LazyIntervalQuo(a, b) }
func crossSqrt(a crossIV) (crossIV, bool)   { return proofbound.LazyIntervalSqrt(a) }
func crossWidth(a crossIV) float64          { return proofbound.LazyIntervalWidthUp(a) }

type crossDual struct{ v, d crossIV }

func crossConst(v crossIV) crossDual { return crossDual{v: v, d: crossPoint(0)} }
func crossDAdd(a, b crossDual) crossDual {
	return crossDual{v: crossAdd(a.v, b.v), d: crossAdd(a.d, b.d)}
}
func crossDSub(a, b crossDual) crossDual {
	return crossDual{v: crossSub(a.v, b.v), d: crossSub(a.d, b.d)}
}
func crossDMul(a, b crossDual) crossDual {
	return crossDual{v: crossMul(a.v, b.v),
		d: crossAdd(crossMul(a.d, b.v), crossMul(a.v, b.d))}
}
func crossDQuo(a, b crossDual) (crossDual, bool) {
	v, ok := crossQuo(a.v, b.v)
	if !ok {
		return crossDual{}, false
	}
	d, ok := crossQuo(crossSub(crossMul(a.d, b.v), crossMul(a.v, b.d)), crossSq(b.v))
	return crossDual{v: v, d: d}, ok
}
func crossDSqrt(a crossDual) (crossDual, bool) {
	v, ok := crossSqrt(a.v)
	if !ok {
		return crossDual{}, false
	}
	d, ok := crossQuo(a.d, crossMul(crossPoint(2), v))
	return crossDual{v: v, d: d}, ok
}
func crossDSinCos(a crossDual) (crossDual, crossDual, bool) {
	s, c, ok := proofbound.LazyRadSinCosSpan(a.v)
	if !ok {
		return crossDual{}, crossDual{}, false
	}
	return crossDual{v: s, d: crossMul(c, a.d)},
		crossDual{v: c, d: crossSub(crossPoint(0), crossMul(s, a.d))}, true
}

// CellResult is what MirroredCellMaxima returns: the maxima of the coordinate
// width, the derivative speed and the band length over the quarter-turn base
// cells, and the maximum held-sample error over all whole-turn cells.
type CellResult struct {
	Width, SampleError, Speed, BandLength float64
}

// CellExact is one cell's exact part: the coordinate width, the
// derivative speed and the band length, plus the nine coordinate enclosures
// the held-sample comparison reads, in the order outer X, Y, Z−c, bore X, Y,
// Z−c, negative-mouth X, Y, Z−c.
type CellExact struct {
	Width, Speed, BandLength float64
	Coords                   [9]crossIV
}

// CellExactProof proves every point of one angular cell, including
// the ruled band between the two source-cylinder setbacks. The cross
// products of the endpoint curves and their mixed pair are Bernstein
// coefficients for polar progress at every transverse band position. Both
// Jacobian endpoints have one sign, so no transverse band strip folds. Each
// clearance and fold test computes only the interval endpoint it reads and
// decides its sign by comparison. It reads no held vertex;
// CellSampleError compares those with the returned coords.
//
// The cell's intervals keep unreduced rationals (proofarith.LazyRat); a value
// is read in lowest terms where it leaves the proof: a square-root input, the
// sine/cosine span input and width, and the float readers.
func CellExactProof(rec *facetedrecord.CrossCylinderBoreRecord, setback float64,
	i int, angularGap *big.Rat) (CellExact, error) {
	const n = 2 * HalfStations
	turn := big.NewRat(int64(i), n)
	s0, c0 := proofbound.TurnSinCosInterval(turn)
	gapLazy := proofarith.LazyOf(angularGap)
	s := proofarith.LazyIntervalWiden(proofarith.LazyIntervalOf(proofbound.ToProofInterval(s0)), gapLazy)
	c := proofarith.LazyIntervalWiden(proofarith.LazyIntervalOf(proofbound.ToProofInterval(c0)), gapLazy)
	R, r, d := crossPoint(rec.Radius), crossPoint(rec.Bore), crossPoint(setback)
	cd, sd := crossDual{v: c, d: crossSub(crossPoint(0), s)}, crossDual{v: s, d: c}
	Rd, rd, dd := crossConst(R), crossConst(r), crossConst(d)
	xd, ok := crossDSqrt(crossDSub(crossDMul(Rd, Rd),
		crossDMul(crossDMul(rd, rd), crossDMul(cd, cd))))
	if !ok {
		return CellExact{}, fmt.Errorf("%w: the cross-bore seam has no real X", decaderr.ErrUnsupported)
	}
	c2 := crossDMul(cd, cd)
	Qd, ok := crossDSqrt(crossDSub(crossDMul(Rd, Rd),
		crossDMul(crossDMul(rd, rd), crossDMul(c2, c2))))
	if !ok {
		return CellExact{}, fmt.Errorf("%w: the cross-bore normal has no length", decaderr.ErrUnsupported)
	}
	do, ok := crossDQuo(crossDMul(crossDMul(dd, cd), xd), crossDMul(Rd, Qd))
	if !ok {
		return CellExact{}, fmt.Errorf("%w: the outer setback has no finite angle", decaderr.ErrUnsupported)
	}
	sodo, codo, ok := crossDSinCos(do)
	if !ok {
		return CellExact{}, fmt.Errorf("%w: the outer setback has no trig enclosure", decaderr.ErrUnsupported)
	}
	outX := crossDSub(crossDMul(xd, codo), crossDMul(crossDMul(rd, cd), sodo))
	outY := crossDAdd(crossDMul(xd, sodo), crossDMul(crossDMul(rd, cd), codo))
	outZ, ok := crossDQuo(crossDMul(crossDMul(dd, Rd), sd), Qd)
	if !ok {
		return CellExact{}, fmt.Errorf("%w: the outer setback has no Z", decaderr.ErrUnsupported)
	}
	outZ = crossDAdd(crossDMul(rd, sd), outZ)
	db, ok := crossDQuo(crossDMul(crossDMul(dd, cd), sd), Qd)
	if !ok {
		return CellExact{}, fmt.Errorf("%w: the bore setback has no finite angle", decaderr.ErrUnsupported)
	}
	sodb, codb, ok := crossDSinCos(db)
	if !ok {
		return CellExact{}, fmt.Errorf("%w: the bore setback has no trig enclosure", decaderr.ErrUnsupported)
	}
	dOverQ, ok := crossDQuo(dd, Qd)
	if !ok {
		return CellExact{}, fmt.Errorf("%w: the bore setback has no X", decaderr.ErrUnsupported)
	}
	inX := crossDMul(xd, crossDSub(crossConst(crossPoint(1)), dOverQ))
	inY := crossDMul(rd, crossDSub(crossDMul(cd, codb), crossDMul(sd, sodb)))
	inZ := crossDMul(rd, crossDAdd(crossDMul(sd, codb), crossDMul(cd, sodb)))
	diffY, diffZ := crossSub(outY.v, inY.v), crossSub(outZ.v, inZ.v)
	mulLo, mulHi := proofarith.LazyIntervalMulLo, proofarith.LazyIntervalMulHi
	// crossPolar(aY, aZ, bY, bZ).Lo
	polarLo := func(aY, aZ, bY, bZ crossDual) proofarith.LazyRat {
		return proofarith.LazySub(mulLo(aY.v, bZ.d), mulHi(aZ.v, bY.d))
	}
	// crossPolar(aY, aZ, bY, bZ).Lo.Sign() > 0
	polarPositive := func(aY, aZ, bY, bZ crossDual) bool {
		return proofarith.LazyCmp(mulLo(aY.v, bZ.d), mulHi(aZ.v, bY.d)) > 0
	}
	// crossJacobian(aY, aZ, dY, dZ).Hi.Sign() < 0
	jacobianNegative := func(aY, aZ crossDual, dY, dZ crossIV) bool {
		return proofarith.LazyCmp(mulHi(aY.d, dZ), mulLo(aZ.d, dY)) < 0
	}
	outerBorePositive := proofarith.LazyCmp(proofarith.LazyAdd(proofarith.LazyIntervalSquareLo(outY.v),
		proofarith.LazyIntervalSquareLo(outZ.v)), proofarith.LazyIntervalSquareHi(r)) > 0
	innerTargetPositive := proofarith.LazyCmp(proofarith.LazyIntervalSquareLo(R),
		proofarith.LazyAdd(proofarith.LazyIntervalSquareHi(inX.v), proofarith.LazyIntervalSquareHi(inY.v))) > 0
	supportPositive := proofarith.LazyCmp(mulLo(diffY, inY.v), proofarith.LazyNeg(mulLo(diffZ, inZ.v))) > 0
	// polarMid = (P1 + P2)/2 with P1 = crossPolar(inY, inZ, outY, outZ) and
	// P2 = crossPolar(outY, outZ, inY, inZ). Dividing by the positive point 2
	// keeps the sign of P1.Lo + P2.Lo, and never fails: the divisor excludes 0.
	polarMidPositive := proofarith.LazyCmp(polarLo(inY, inZ, outY, outZ),
		proofarith.LazyNeg(polarLo(outY, outZ, inY, inZ))) > 0
	if !outerBorePositive || !innerTargetPositive || !supportPositive ||
		!polarPositive(inY, inZ, inY, inZ) || !polarPositive(outY, outZ, outY, outZ) ||
		!polarMidPositive || !jacobianNegative(inY, inZ, diffY, diffZ) ||
		!jacobianNegative(outY, outZ, diffY, diffZ) {
		return CellExact{}, fmt.Errorf("%w: a cross-bore chamfer cell loses clearance or folds", decaderr.ErrUnsupported)
	}
	negativeX := crossSub(crossPoint(0), xd.v)
	negativeY, negativeZ := crossMul(r, c), crossMul(r, s)
	width := 0.0
	for _, coordinate := range []crossIV{
		outX.v, outY.v, outZ.v, inX.v, inY.v, inZ.v,
		negativeX, negativeY, negativeZ,
	} {
		width = max(width, crossWidth(coordinate))
	}
	speed := 0.0
	for _, dcoord := range []crossIV{outX.d, outY.d, outZ.d, inX.d, inY.d, inZ.d} {
		speed = max(speed, proofbound.LazyIntervalAbsUpperUp(dcoord))
	}
	diffX := crossSub(outX.v, inX.v)
	bandCoord := max(
		proofbound.LazyIntervalAbsUpperUp(diffX),
		proofbound.LazyIntervalAbsUpperUp(diffY),
		proofbound.LazyIntervalAbsUpperUp(diffZ))
	return CellExact{
		Width: width, Speed: proofbound.Radius3D(speed), BandLength: proofbound.Radius3D(bandCoord),
		Coords: [9]crossIV{outX.v, outY.v, outZ.v, inX.v, inY.v, inZ.v, negativeX, negativeY, negativeZ},
	}, nil
}

const (
	// CellParallelMin is the cell count from which the cells are
	// proved on several goroutines.
	CellParallelMin = 256
	// CellGrain is the number of consecutive cells one worker takes
	// at a time.
	CellGrain = 16
)

// crossNeg is the exact negation of an interval: [−Hi, −Lo].
func crossNeg(a crossIV) crossIV {
	return crossIV{Lo: proofarith.LazyNeg(a.Hi), Hi: proofarith.LazyNeg(a.Lo)}
}

// CellSampleError compares one cell's held vertices with coords:
// held holds the outer foot, the bore foot and the negative-mouth point the
// mesh keeps at the cell's angle, and centerZ is the bore axis height c.
// coords must enclose the cell's exact coordinates (X, Y, Z−c of each point)
// once the Y enclosures are negated when flipY and the Z−c enclosures when
// flipZ. Negation and the shift of a Z−c enclosure by c are exact, and
// LazyIntervalFloatError reads the exact difference from the held float, so
// the comparison rounds only its result, upward.
func CellSampleError(held *[3]r3.Vec, centerZ float64, coords *[9]crossIV, flipY, flipZ bool) float64 {
	c := crossPoint(centerZ)
	sampleError := 0.0
	for p, point := range held {
		x, y, z := coords[3*p], coords[3*p+1], coords[3*p+2]
		if flipY {
			y = crossNeg(y)
		}
		if flipZ {
			z = crossNeg(z)
		}
		sampleError = max(sampleError,
			proofbound.LazyIntervalFloatError(x, point.X),
			proofbound.LazyIntervalFloatError(y, point.Y),
			proofbound.LazyIntervalFloatError(crossAdd(z, c), point.Z))
	}
	return proofbound.Radius3D(sampleError)
}

// CellMirror maps cell j of the n = 4,096 whole-turn cells to the
// quarter-turn base cell k in [0, n/4] whose angular range is its
// mirror image, and says which coordinates the mirror negates. With
// θ_j = 2πj/n: j ≤ n/4 is its own base; j ≤ n/2 has θ_j = π − θ_k (Y
// negated); j ≤ 3n/4 has θ_j = π + θ_k (Y and Z−c negated); otherwise
// θ_j = 2π − θ_k (Z−c negated). The relations hold exactly for the turns
// j/n and k/n, so they map the whole range θ_j ± 2π/n onto θ_k ± 2π/n.
func CellMirror(j int) (int, bool, bool) {
	const n = 2 * HalfStations
	const quarter, half = n / 4, n / 2
	switch {
	case j <= quarter:
		return j, false, false
	case j <= half:
		return half - j, true, false
	case j <= half+quarter:
		return j - half, true, true
	default:
		return n - j, false, true
	}
}

// cellEach runs fn(i) for every i in [0, n) and returns the error
// of the lowest failing index, whatever the worker count. With workers below
// 2 it polls ctx every 32 indices; otherwise it polls ctx before the fan-out
// and the calling goroutine polls it while the workers run. fn runs on
// worker goroutines and must not touch ctx or any shared budget.
func cellEach(ctx context.Context, n, workers int, fn func(i int) error) error {
	if workers < 2 {
		for i := range n {
			if i%32 == 0 {
				if err := ctx.Err(); err != nil {
					return err
				}
			}
			if err := fn(i); err != nil {
				return err
			}
		}
		return nil
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	_, err := orderedwork.Each(n, workers, CellGrain, ctx.Err, fn)
	return err
}

// MirroredCellMaxima proves the base cells k in [0, n/4] of the
// n = 4,096 whole-turn cells with exact
// and returns the error of the lowest failing one. It then folds every cell
// j in [0, n), base cells included, with sample(j, base, flipY, flipZ) for
// its CellMirror base and flags, so each cell's own held samples
// are compared with its base cell's mirrored enclosures. width, speed and
// bandLength are the base cells' maxima: a negated interval has the same
// exact width and absolute upper bound, so the readings of base k cover
// cells mapped onto it. The maxima are folded in index order whatever the
// worker count, and a context cancelled before the call returns fails it.
func MirroredCellMaxima(ctx context.Context, workers int,
	exact func(k int) (CellExact, error),
	sample func(j int, base *CellExact, flipY, flipZ bool) float64) (CellResult, error) {
	const n = 2 * HalfStations
	bases := make([]CellExact, n/4+1)
	if err := cellEach(ctx, len(bases), workers, func(k int) error {
		b, err := exact(k)
		bases[k] = b
		return err
	}); err != nil {
		return CellResult{}, err
	}
	samples := make([]float64, n)
	if err := cellEach(ctx, n, workers, func(j int) error {
		k, flipY, flipZ := CellMirror(j)
		samples[j] = sample(j, &bases[k], flipY, flipZ)
		return nil
	}); err != nil {
		return CellResult{}, err
	}
	if err := ctx.Err(); err != nil {
		return CellResult{}, err
	}
	maximum := CellResult{}
	for k := range bases {
		maximum.Width = max(maximum.Width, bases[k].Width)
		maximum.Speed = max(maximum.Speed, bases[k].Speed)
		maximum.BandLength = max(maximum.BandLength, bases[k].BandLength)
	}
	for _, e := range samples {
		maximum.SampleError = max(maximum.SampleError, e)
	}
	return maximum, nil
}
