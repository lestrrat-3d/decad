package decad

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The crossFull helpers below are crossPolar and crossJacobian as the cell
// proof used them when each clearance and fold test built both interval
// endpoints. crossBoreCellProofFullEnds is that cell proof, kept verbatim as
// the reference the sign-only tests must equal.

func crossFullPolar(aY, aZ, bY, bZ crossDual) crossIV {
	return crossSub(crossMul(aY.v, bZ.d), crossMul(aZ.v, bY.d))
}
func crossFullJacobian(aY, aZ, dY, dZ crossDual) crossIV {
	return crossSub(crossMul(aY.d, dZ.v), crossMul(aZ.d, dY.v))
}

// crossBoreCellProofFullEnds is crossBoreCellProof as it stood before the
// clearance and fold tests computed only the endpoints they read.
func crossBoreCellProofFullEnds(rec *crossCylinderBoreRecord, setback float64,
	i int, angularGap *big.Rat) (crossBoreCellResult, error) {
	const n = 2 * crossBoreHalfStations
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
	polarMid, ok := crossQuo(crossAdd(crossFullPolar(inY, inZ, outY, outZ),
		crossFullPolar(outY, outZ, inY, inZ)), crossPoint(2))
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the band polar proof is undefined", ErrUnsupported)
	}
	if outerBore.Lo.Sign() <= 0 || innerTarget.Lo.Sign() <= 0 || support.Lo.Sign() <= 0 ||
		crossFullPolar(inY, inZ, inY, inZ).Lo.Sign() <= 0 ||
		crossFullPolar(outY, outZ, outY, outZ).Lo.Sign() <= 0 || polarMid.Lo.Sign() <= 0 ||
		crossFullJacobian(inY, inZ, diffY, diffZ).Hi.Sign() >= 0 ||
		crossFullJacobian(outY, outZ, diffY, diffZ).Hi.Sign() >= 0 {
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
	negative := r3.Vec{X: -math.Sqrt(rec.Radius*rec.Radius - rec.Bore*rec.Bore*co*co),
		Y: rec.Bore * co, Z: rec.Bore * sn}
	sampleError := 0.0
	for _, pair := range []struct {
		exact crossIV
		held  float64
	}{
		{outX.v, outer.X}, {outY.v, outer.Y}, {outZ.v, outer.Z - rec.CenterZ},
		{inX.v, inner.X}, {inY.v, inner.Y}, {inZ.v, inner.Z - rec.CenterZ},
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

func TestCrossBoreSignTestsMatchReference(t *testing.T) {
	gap := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper),
		big.NewRat(2*crossBoreHalfStations, 1))
	for _, tc := range []struct {
		name    string
		rec     crossCylinderBoreRecord
		setback float64
		failing bool
	}{
		{"five by one", crossCylinderBoreRecord{Radius: 5, Bore: 1, Height: 10, CenterZ: 5, SourceBound: 1e-9}, 0.5, false},
		{"eight by one and half",
			crossCylinderBoreRecord{Radius: 8, Bore: 1.5, Height: 16, CenterZ: 8, SourceBound: 1e-9}, 0.1875, false},
		{"loses clearance or folds",
			crossCylinderBoreRecord{Radius: 5, Bore: 4.9, Height: 10, CenterZ: 5, SourceBound: 1e-9}, 0.5, true},
		{"no real X", crossCylinderBoreRecord{Radius: 5, Bore: 5, Height: 10, CenterZ: 5, SourceBound: 1e-9}, 0.5, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			const n = 2 * crossBoreHalfStations
			failed := 0
			for i := 0; i < n; i += 8 {
				want, wantErr := crossBoreCellProofFullEnds(&tc.rec, tc.setback, i, gap)
				got, gotErr := crossBoreCellProof(&tc.rec, tc.setback, i, gap)
				if wantErr != nil {
					failed++
					require.EqualError(t, gotErr, wantErr.Error(), "cell %d", i)
					continue
				}
				require.NoError(t, gotErr, "cell %d", i)
				requireCrossBoreBitsEqual(t, want, got)
			}
			if tc.failing {
				require.Positive(t, failed)
				return
			}
			require.Zero(t, failed)
		})
	}
}
