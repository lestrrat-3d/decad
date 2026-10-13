package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
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
func crossPolar(aY, aZ, bY, bZ crossDual) crossIV {
	return crossSub(crossMul(aY.v, bZ.d), crossMul(aZ.v, bY.d))
}
func crossJacobian(aY, aZ, dY, dZ crossDual) crossIV {
	return crossSub(crossMul(aY.d, dZ.v), crossMul(aZ.d, dY.v))
}

type crossBoreCellResult struct {
	width, sampleError, speed, bandLength float64
}

// crossBoreCellProof proves every point of one angular cell, including the
// ruled band between the two source-cylinder setbacks. The cross products of
// the endpoint curves and their mixed pair are Bernstein coefficients for
// polar progress at every transverse band position. Both Jacobian endpoints
// have one sign, so no transverse band strip folds. The source/held sample
// comparison charges libm and coordinate rounding instead of assuming ulps.
//
// The cell's intervals keep unreduced rationals (proofarith.LazyRat); a value
// is read in lowest terms where it leaves the proof: a square-root input, the
// sine/cosine span input and width, and the float readers.
func crossBoreCellProof(rec *crossCylinderBoreRecord, setback float64,
	i int, angularGap *big.Rat) (crossBoreCellResult, error) {
	const n = 2 * crossBoreHalfStations
	turn := big.NewRat(int64(i), n)
	s0, c0 := proofbound.TurnSinCosInterval(turn)
	gapLazy := proofarith.LazyOf(angularGap)
	s := proofarith.LazyIntervalWiden(proofarith.LazyIntervalOf(proofbound.ToProofInterval(s0)), gapLazy)
	c := proofarith.LazyIntervalWiden(proofarith.LazyIntervalOf(proofbound.ToProofInterval(c0)), gapLazy)
	R, r, d := crossPoint(rec.radius), crossPoint(rec.bore), crossPoint(setback)
	cd, sd := crossDual{v: c, d: crossSub(crossPoint(0), s)}, crossDual{v: s, d: c}
	Rd, rd, dd := crossConst(R), crossConst(r), crossConst(d)
	xd, ok := crossDSqrt(crossDSub(crossDMul(Rd, Rd),
		crossDMul(crossDMul(rd, rd), crossDMul(cd, cd))))
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the cross-bore seam has no real X", ErrUnsupported)
	}
	c2 := crossDMul(cd, cd)
	Qd, ok := crossDSqrt(crossDSub(crossDMul(Rd, Rd),
		crossDMul(crossDMul(rd, rd), crossDMul(c2, c2))))
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the cross-bore normal has no length", ErrUnsupported)
	}
	do, ok := crossDQuo(crossDMul(crossDMul(dd, cd), xd), crossDMul(Rd, Qd))
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the outer setback has no finite angle", ErrUnsupported)
	}
	sodo, codo, ok := crossDSinCos(do)
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the outer setback has no trig enclosure", ErrUnsupported)
	}
	outX := crossDSub(crossDMul(xd, codo), crossDMul(crossDMul(rd, cd), sodo))
	outY := crossDAdd(crossDMul(xd, sodo), crossDMul(crossDMul(rd, cd), codo))
	outZ, ok := crossDQuo(crossDMul(crossDMul(dd, Rd), sd), Qd)
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the outer setback has no Z", ErrUnsupported)
	}
	outZ = crossDAdd(crossDMul(rd, sd), outZ)
	db, ok := crossDQuo(crossDMul(crossDMul(dd, cd), sd), Qd)
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the bore setback has no finite angle", ErrUnsupported)
	}
	sodb, codb, ok := crossDSinCos(db)
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the bore setback has no trig enclosure", ErrUnsupported)
	}
	dOverQ, ok := crossDQuo(dd, Qd)
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the bore setback has no X", ErrUnsupported)
	}
	inX := crossDMul(xd, crossDSub(crossConst(crossPoint(1)), dOverQ))
	inY := crossDMul(rd, crossDSub(crossDMul(cd, codb), crossDMul(sd, sodb)))
	inZ := crossDMul(rd, crossDAdd(crossDMul(sd, codb), crossDMul(cd, sodb)))
	outerBore := crossSub(crossAdd(crossSq(outY.v), crossSq(outZ.v)), crossSq(r))
	innerTarget := crossSub(crossSq(R), crossAdd(crossSq(inX.v), crossSq(inY.v)))
	diffY, diffZ := crossDSub(outY, inY), crossDSub(outZ, inZ)
	support := crossAdd(crossMul(diffY.v, inY.v), crossMul(diffZ.v, inZ.v))
	polarMid, ok := crossQuo(crossAdd(crossPolar(inY, inZ, outY, outZ),
		crossPolar(outY, outZ, inY, inZ)), crossPoint(2))
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the band polar proof is undefined", ErrUnsupported)
	}
	if outerBore.Lo.Sign() <= 0 || innerTarget.Lo.Sign() <= 0 || support.Lo.Sign() <= 0 ||
		crossPolar(inY, inZ, inY, inZ).Lo.Sign() <= 0 ||
		crossPolar(outY, outZ, outY, outZ).Lo.Sign() <= 0 || polarMid.Lo.Sign() <= 0 ||
		crossJacobian(inY, inZ, diffY, diffZ).Hi.Sign() >= 0 ||
		crossJacobian(outY, outZ, diffY, diffZ).Hi.Sign() >= 0 {
		return crossBoreCellResult{}, fmt.Errorf("%w: a cross-bore chamfer cell loses clearance or folds", ErrUnsupported)
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
	theta := 2 * math.Pi * float64(i) / n
	outer, inner := crossBoreFeet(rec, setback, theta)
	co, sn := math.Cos(theta), math.Sin(theta)
	negative := r3.Vec{X: -math.Sqrt(rec.radius*rec.radius - rec.bore*rec.bore*co*co),
		Y: rec.bore * co, Z: rec.bore * sn}
	sampleError := 0.0
	for _, pair := range []struct {
		exact crossIV
		held  float64
	}{
		{outX.v, outer.X}, {outY.v, outer.Y}, {outZ.v, outer.Z - rec.centerZ},
		{inX.v, inner.X}, {inY.v, inner.Y}, {inZ.v, inner.Z - rec.centerZ},
		{negativeX, negative.X}, {negativeY, negative.Y}, {negativeZ, negative.Z},
	} {
		sampleError = max(sampleError, proofbound.LazyIntervalFloatError(pair.exact, pair.held))
	}
	speed := 0.0
	for _, dcoord := range []crossIV{outX.d, outY.d, outZ.d, inX.d, inY.d, inZ.d} {
		speed = max(speed, proofbound.LazyIntervalAbsUpperUp(dcoord))
	}
	diffX := crossSub(outX.v, inX.v)
	bandCoord := max(
		proofbound.LazyIntervalAbsUpperUp(diffX),
		proofbound.LazyIntervalAbsUpperUp(diffY.v),
		proofbound.LazyIntervalAbsUpperUp(diffZ.v))
	return crossBoreCellResult{
		width: width, sampleError: proofbound.Radius3D(sampleError),
		speed: proofbound.Radius3D(speed), bandLength: proofbound.Radius3D(bandCoord),
	}, nil
}

const (
	// crossBoreCellParallelMin is the cell count from which the cells are
	// proved on several goroutines.
	crossBoreCellParallelMin = 256
	// crossBoreCellGrain is the number of consecutive cells one worker takes
	// at a time.
	crossBoreCellGrain = 16
)

// crossBoreCellMaxima folds cell(i) for i in [0, n) into the componentwise
// maximum in index order and returns the error of the lowest failing cell,
// whatever the worker count. With workers below 2 it polls ctx every 32
// cells; otherwise it polls ctx before the fan-out and the calling goroutine
// polls it while the workers run. cell runs on worker goroutines and must
// not touch ctx or any shared budget.
func crossBoreCellMaxima(ctx context.Context, n, workers int,
	cell func(i int) (crossBoreCellResult, error)) (crossBoreCellResult, error) {
	maximum := crossBoreCellResult{}
	fold := func(c crossBoreCellResult) {
		maximum.width = max(maximum.width, c.width)
		maximum.sampleError = max(maximum.sampleError, c.sampleError)
		maximum.speed = max(maximum.speed, c.speed)
		maximum.bandLength = max(maximum.bandLength, c.bandLength)
	}
	if workers < 2 {
		for i := range n {
			if i%32 == 0 {
				if err := ctx.Err(); err != nil {
					return crossBoreCellResult{}, err
				}
			}
			c, err := cell(i)
			if err != nil {
				return crossBoreCellResult{}, err
			}
			fold(c)
		}
		return maximum, nil
	}
	if err := ctx.Err(); err != nil {
		return crossBoreCellResult{}, err
	}
	cells := make([]crossBoreCellResult, n)
	if _, err := orderedwork.Each(n, workers, crossBoreCellGrain, ctx.Err, func(i int) error {
		c, err := cell(i)
		cells[i] = c
		return err
	}); err != nil {
		return crossBoreCellResult{}, err
	}
	for i := range n {
		fold(cells[i])
	}
	return maximum, nil
}

// proveCrossBoreChamfer encloses all source-to-held boundary motion. Its
// volume proof charges the entire positive-mouth collar, where the ideal
// chamfer and mesh can differ arbitrarily, then two radial chord annuli for
// unchanged outer and bore walls. A collar wider than the actual removed
// region is deliberate; it keeps the occupied-volume argument independent
// of the exact band integral. Each source foot is at developed-cylinder
// distance d from its seam point, so its Euclidean distance is at most d;
// the ruled band lies in the resulting positive-mouth collar. The cells are
// proved concurrently and folded in cell order.
func proveCrossBoreChamfer(ctx context.Context, rec *crossCylinderBoreRecord,
	setback float64, mesh *crossBoreMesh) (float64, float64, float64, error) {
	const n = 2 * crossBoreHalfStations
	gap := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper), big.NewRat(n, 1))
	maximum, err := crossBoreCellMaxima(ctx, n, orderedwork.EachWorkers(n, crossBoreCellParallelMin),
		func(i int) (crossBoreCellResult, error) { return crossBoreCellProof(rec, setback, i, gap) })
	if err != nil {
		return 0, 0, 0, err
	}
	// Every vertical wall patch pairs source and held points at the same
	// height fraction. The band pairs them at the same angle and transverse
	// fraction. Two endpoint curves in one cell have coordinate spread at
	// most width; the triangle diagonal adds one further spread. Three box
	// diagonals enclose all such pairings and their rounded sample vertices.
	R, r, H, d := rec.radius, rec.bore, rec.height, setback
	xMinSquared := new(big.Rat).Sub(
		new(big.Rat).Mul(proofarith.FloatRat(R), proofarith.FloatRat(R)),
		new(big.Rat).Mul(proofarith.FloatRat(r), proofarith.FloatRat(r)))
	xMin := proofbound.RatSqrtDown(xMinSquared)
	width := proofbound.RatFloatUp(new(big.Rat).Sub(proofarith.FloatRat(R), proofarith.FloatRat(xMin)))
	width = proofbound.AbsSumUpper(width, proofbound.ProductUpper(2, d), rec.sourceBound)
	span := proofbound.AbsSumUpper(proofbound.ProductUpper(2, r), proofbound.ProductUpper(2, d),
		proofbound.ProductUpper(2, rec.sourceBound))
	collar := proofbound.ProductUpper(width, proofbound.ProductUpper(span, span))
	piUpper := proofbound.RatFloatUp(proofbound.PiUpper)
	// The mesh builder certifies the maximum exact angular gap of every
	// station, including the mouth schedules and the full-turn wrap. It also
	// encloses every held outer-ring vertex against that source-circle
	// parameter, covering both radius and angle-conversion rounding.
	outerChord := proofbound.ProductUpper(R,
		proofbound.DivUpper(proofbound.ProductUpper(mesh.outerAngularGap, mesh.outerAngularGap), 8))
	// The bore wall is a ruled cylinder patch whose far loop rotates by
	// delta_b=d*cos(theta)*sin(theta)/Q. A triangle spans the station gap
	// PLUS that cross-wall twist. |cos*sin|<=1/2 and Q>=xMin, so twice the
	// maximum twist is at most d/xMin. The doubled allowance also covers a
	// possible sign change of delta_b across a station cell.
	boreStep := proofbound.AbsSumUpper(
		proofbound.DivUpper(proofbound.ProductUpper(2, piUpper), n),
		proofbound.DivUpper(d, xMin))
	boreChord := proofbound.ProductUpper(r,
		proofbound.DivUpper(proofbound.ProductUpper(boreStep, boreStep), 8))
	bound := proofbound.AbsSumUpper(
		proofbound.ProductUpper(3, proofbound.Radius3D(maximum.width)),
		maximum.sampleError, proofbound.ProductUpper(4, rec.sourceBound),
		mesh.outerStationError, outerChord, boreChord)
	if !(bound < 0.1) || proofbound.IsNonFinite(bound) {
		return 0, 0, 0, fmt.Errorf("%w: the cross-bore chamfer exceeds its 0.1 mm proof plan", ErrUnsupported)
	}
	outerShell := proofbound.ProductUpper(4*piUpper,
		proofbound.ProductUpper(R, proofbound.ProductUpper(H,
			proofbound.AbsSumUpper(outerChord, mesh.outerStationError, rec.sourceBound))))
	boreShell := proofbound.ProductUpper(8*piUpper,
		proofbound.ProductUpper(R, proofbound.ProductUpper(r,
			proofbound.AbsSumUpper(boreChord, rec.sourceBound))))
	volumeGap := proofbound.AbsSumUpper(collar, outerShell, boreShell)
	// Surface area is bounded by the complete outer cylinder and caps, the
	// complete bore wall, and the band's derivative times its width. Adding
	// the held area bounds their absolute difference without assuming equal
	// areas at a mesh cut.
	sourceArea := proofbound.AbsSumUpper(
		proofbound.ProductUpper(2*piUpper, proofbound.ProductUpper(R, H)),
		proofbound.ProductUpper(2*piUpper, proofbound.ProductUpper(R, R)),
		proofbound.ProductUpper(4*piUpper, proofbound.ProductUpper(R, r)),
		proofbound.ProductUpper(2*piUpper,
			proofbound.ProductUpper(maximum.speed, maximum.bandLength)))
	heldArea := facetproof.MeshAreaUpper(mesh.verts, mesh.tris)
	areaSlack := proofbound.AbsSumUpper(sourceArea, heldArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return 0, 0, 0, fmt.Errorf("%w: the cross-bore chamfer has no finite volume or area proof", ErrUnsupported)
	}
	return bound, volumeGap, areaSlack, nil
}
