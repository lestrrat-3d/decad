package decad

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The crossRef helpers and crossBoreCellProofReference are the cell proof over
// reduced big.Rat intervals, the form crossBoreCellProof had before it kept
// unreduced rationals. They are the reference its results must equal bit for
// bit.
type crossRefIV = proofbound.RatInterval

func crossRefPoint(x float64) crossRefIV             { return proofbound.PointInterval(proofarith.FloatRat(x)) }
func crossRefAdd(a, b crossRefIV) crossRefIV         { return proofbound.IntervalAdd(a, b) }
func crossRefSub(a, b crossRefIV) crossRefIV         { return proofbound.IntervalSub(a, b) }
func crossRefMul(a, b crossRefIV) crossRefIV         { return proofbound.IntervalMul(a, b) }
func crossRefSq(a crossRefIV) crossRefIV             { return proofbound.IntervalSquare(a) }
func crossRefQuo(a, b crossRefIV) (crossRefIV, bool) { return proofbound.IntervalQuo(a, b) }
func crossRefSqrt(a crossRefIV) (crossRefIV, bool)   { return proofbound.IntervalSqrt(a) }
func crossRefWidth(a crossRefIV) float64 {
	return proofbound.RatFloatUp(new(big.Rat).Sub(a.Hi, a.Lo))
}

type crossRefDual struct{ v, d crossRefIV }

func crossRefConst(v crossRefIV) crossRefDual { return crossRefDual{v: v, d: crossRefPoint(0)} }
func crossRefDAdd(a, b crossRefDual) crossRefDual {
	return crossRefDual{v: crossRefAdd(a.v, b.v), d: crossRefAdd(a.d, b.d)}
}
func crossRefDSub(a, b crossRefDual) crossRefDual {
	return crossRefDual{v: crossRefSub(a.v, b.v), d: crossRefSub(a.d, b.d)}
}
func crossRefDMul(a, b crossRefDual) crossRefDual {
	return crossRefDual{v: crossRefMul(a.v, b.v),
		d: crossRefAdd(crossRefMul(a.d, b.v), crossRefMul(a.v, b.d))}
}
func crossRefDQuo(a, b crossRefDual) (crossRefDual, bool) {
	v, ok := crossRefQuo(a.v, b.v)
	if !ok {
		return crossRefDual{}, false
	}
	d, ok := crossRefQuo(crossRefSub(crossRefMul(a.d, b.v), crossRefMul(a.v, b.d)), crossRefSq(b.v))
	return crossRefDual{v: v, d: d}, ok
}
func crossRefDSqrt(a crossRefDual) (crossRefDual, bool) {
	v, ok := crossRefSqrt(a.v)
	if !ok {
		return crossRefDual{}, false
	}
	d, ok := crossRefQuo(a.d, crossRefMul(crossRefPoint(2), v))
	return crossRefDual{v: v, d: d}, ok
}
func crossRefDSinCos(a crossRefDual) (crossRefDual, crossRefDual, bool) {
	s, c, ok := proofbound.RadSinCosSpan(a.v)
	if !ok {
		return crossRefDual{}, crossRefDual{}, false
	}
	return crossRefDual{v: s, d: crossRefMul(c, a.d)},
		crossRefDual{v: c, d: crossRefSub(crossRefPoint(0), crossRefMul(s, a.d))}, true
}
func crossRefPolar(aY, aZ, bY, bZ crossRefDual) crossRefIV {
	return crossRefSub(crossRefMul(aY.v, bZ.d), crossRefMul(aZ.v, bY.d))
}
func crossRefJacobian(aY, aZ, dY, dZ crossRefDual) crossRefIV {
	return crossRefSub(crossRefMul(aY.d, dZ.v), crossRefMul(aZ.d, dY.v))
}

// crossBoreCellProofReference proves every point of one angular cell, including the
// ruled band between the two source-cylinder setbacks. The cross products of
// the endpoint curves and their mixed pair are Bernstein coefficients for
// polar progress at every transverse band position. Both Jacobian endpoints
// have one sign, so no transverse band strip folds. The source/held sample
// comparison charges libm and coordinate rounding instead of assuming ulps.
func crossBoreCellProofReference(rec *crossCylinderBoreRecord, setback float64,
	i int, angularGap *big.Rat) (crossBoreCellResult, error) {
	const n = 2 * crossBoreHalfStations
	turn := big.NewRat(int64(i), n)
	s, c := proofbound.TurnSinCosInterval(turn)
	s, c = proofbound.IntervalWiden(s, angularGap), proofbound.IntervalWiden(c, angularGap)
	R, r, d := crossRefPoint(rec.Radius), crossRefPoint(rec.Bore), crossRefPoint(setback)
	cd, sd := crossRefDual{v: c, d: crossRefSub(crossRefPoint(0), s)}, crossRefDual{v: s, d: c}
	Rd, rd, dd := crossRefConst(R), crossRefConst(r), crossRefConst(d)
	xd, ok := crossRefDSqrt(crossRefDSub(crossRefDMul(Rd, Rd),
		crossRefDMul(crossRefDMul(rd, rd), crossRefDMul(cd, cd))))
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the cross-bore seam has no real X", ErrUnsupported)
	}
	c2 := crossRefDMul(cd, cd)
	Qd, ok := crossRefDSqrt(crossRefDSub(crossRefDMul(Rd, Rd),
		crossRefDMul(crossRefDMul(rd, rd), crossRefDMul(c2, c2))))
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the cross-bore normal has no length", ErrUnsupported)
	}
	do, ok := crossRefDQuo(crossRefDMul(crossRefDMul(dd, cd), xd), crossRefDMul(Rd, Qd))
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the outer setback has no finite angle", ErrUnsupported)
	}
	sodo, codo, ok := crossRefDSinCos(do)
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the outer setback has no trig enclosure", ErrUnsupported)
	}
	outX := crossRefDSub(crossRefDMul(xd, codo), crossRefDMul(crossRefDMul(rd, cd), sodo))
	outY := crossRefDAdd(crossRefDMul(xd, sodo), crossRefDMul(crossRefDMul(rd, cd), codo))
	outZ, ok := crossRefDQuo(crossRefDMul(crossRefDMul(dd, Rd), sd), Qd)
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the outer setback has no Z", ErrUnsupported)
	}
	outZ = crossRefDAdd(crossRefDMul(rd, sd), outZ)
	db, ok := crossRefDQuo(crossRefDMul(crossRefDMul(dd, cd), sd), Qd)
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the bore setback has no finite angle", ErrUnsupported)
	}
	sodb, codb, ok := crossRefDSinCos(db)
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the bore setback has no trig enclosure", ErrUnsupported)
	}
	dOverQ, ok := crossRefDQuo(dd, Qd)
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the bore setback has no X", ErrUnsupported)
	}
	inX := crossRefDMul(xd, crossRefDSub(crossRefConst(crossRefPoint(1)), dOverQ))
	inY := crossRefDMul(rd, crossRefDSub(crossRefDMul(cd, codb), crossRefDMul(sd, sodb)))
	inZ := crossRefDMul(rd, crossRefDAdd(crossRefDMul(sd, codb), crossRefDMul(cd, sodb)))
	outerBore := crossRefSub(crossRefAdd(crossRefSq(outY.v), crossRefSq(outZ.v)), crossRefSq(r))
	innerTarget := crossRefSub(crossRefSq(R), crossRefAdd(crossRefSq(inX.v), crossRefSq(inY.v)))
	diffY, diffZ := crossRefDSub(outY, inY), crossRefDSub(outZ, inZ)
	support := crossRefAdd(crossRefMul(diffY.v, inY.v), crossRefMul(diffZ.v, inZ.v))
	polarMid, ok := crossRefQuo(crossRefAdd(crossRefPolar(inY, inZ, outY, outZ),
		crossRefPolar(outY, outZ, inY, inZ)), crossRefPoint(2))
	if !ok {
		return crossBoreCellResult{}, fmt.Errorf("%w: the band polar proof is undefined", ErrUnsupported)
	}
	if outerBore.Lo.Sign() <= 0 || innerTarget.Lo.Sign() <= 0 || support.Lo.Sign() <= 0 ||
		crossRefPolar(inY, inZ, inY, inZ).Lo.Sign() <= 0 ||
		crossRefPolar(outY, outZ, outY, outZ).Lo.Sign() <= 0 || polarMid.Lo.Sign() <= 0 ||
		crossRefJacobian(inY, inZ, diffY, diffZ).Hi.Sign() >= 0 ||
		crossRefJacobian(outY, outZ, diffY, diffZ).Hi.Sign() >= 0 {
		return crossBoreCellResult{}, fmt.Errorf("%w: a cross-bore chamfer cell loses clearance or folds", ErrUnsupported)
	}
	negativeX := crossRefSub(crossRefPoint(0), xd.v)
	negativeY, negativeZ := crossRefMul(r, c), crossRefMul(r, s)
	width := 0.0
	for _, coordinate := range []crossRefIV{
		outX.v, outY.v, outZ.v, inX.v, inY.v, inZ.v,
		negativeX, negativeY, negativeZ,
	} {
		width = max(width, crossRefWidth(coordinate))
	}
	theta := 2 * math.Pi * float64(i) / n
	outer, inner := crossBoreFeet(rec, setback, theta)
	co, sn := math.Cos(theta), math.Sin(theta)
	negative := r3.Vec{X: -math.Sqrt(rec.Radius*rec.Radius - rec.Bore*rec.Bore*co*co),
		Y: rec.Bore * co, Z: rec.Bore * sn}
	sampleError := 0.0
	for _, pair := range []struct {
		exact crossRefIV
		held  float64
	}{
		{outX.v, outer.X}, {outY.v, outer.Y}, {outZ.v, outer.Z - rec.CenterZ},
		{inX.v, inner.X}, {inY.v, inner.Y}, {inZ.v, inner.Z - rec.CenterZ},
		{negativeX, negative.X}, {negativeY, negative.Y}, {negativeZ, negative.Z},
	} {
		sampleError = max(sampleError, proofbound.IntervalFloatError(pair.exact, pair.held))
	}
	speed := 0.0
	for _, dcoord := range []crossRefIV{outX.d, outY.d, outZ.d, inX.d, inY.d, inZ.d} {
		speed = max(speed, proofbound.RatFloatUp(proofbound.IntervalAbsUpper(dcoord)))
	}
	diffX := crossRefSub(outX.v, inX.v)
	bandCoord := max(
		proofbound.RatFloatUp(proofbound.IntervalAbsUpper(diffX)),
		proofbound.RatFloatUp(proofbound.IntervalAbsUpper(diffY.v)),
		proofbound.RatFloatUp(proofbound.IntervalAbsUpper(diffZ.v)))
	return crossBoreCellResult{
		width: width, sampleError: proofbound.Radius3D(sampleError),
		speed: proofbound.Radius3D(speed), bandLength: proofbound.Radius3D(bandCoord),
	}, nil
}

// crossBoreSampleCells lists every stride-th cell plus the cells around the
// quarter turns, where cos or sin of the cell angle vanishes.
func crossBoreSampleCells(stride int) []int {
	const n = 2 * crossBoreHalfStations
	cells := map[int]struct{}{}
	for i := 0; i < n; i += stride {
		cells[i] = struct{}{}
	}
	for _, q := range []int{0, n / 4, n / 2, 3 * n / 4, n} {
		for d := -2; d <= 2; d++ {
			cells[(q+d+n)%n] = struct{}{}
		}
	}
	out := make([]int, 0, len(cells))
	for i := range n {
		if _, ok := cells[i]; ok {
			out = append(out, i)
		}
	}
	return out
}

func TestCrossBoreLazyCellsMatchReference(t *testing.T) {
	gap := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper),
		big.NewRat(2*crossBoreHalfStations, 1))
	for _, tc := range []struct {
		name    string
		rec     crossCylinderBoreRecord
		setback float64
		stride  int
	}{
		{"five by one", crossCylinderBoreRecord{Radius: 5, Bore: 1, Height: 10, CenterZ: 5, SourceBound: 1e-9}, 0.5, 8},
		{"eight by one and half",
			crossCylinderBoreRecord{Radius: 8, Bore: 1.5, Height: 16, CenterZ: 8, SourceBound: 1e-9}, 0.1875, 8},
		{"loses clearance or folds",
			crossCylinderBoreRecord{Radius: 5, Bore: 4.9, Height: 10, CenterZ: 5, SourceBound: 1e-9}, 0.5, 16},
		{"no real X", crossCylinderBoreRecord{Radius: 5, Bore: 5, Height: 10, CenterZ: 5, SourceBound: 1e-9}, 0.5, 16},
	} {
		t.Run(tc.name, func(t *testing.T) {
			failed := 0
			for _, i := range crossBoreSampleCells(tc.stride) {
				want, wantErr := crossBoreCellProofReference(&tc.rec, tc.setback, i, gap)
				got, gotErr := crossBoreCellProof(&tc.rec, tc.setback, i, gap)
				if wantErr != nil {
					failed++
					require.EqualError(t, gotErr, wantErr.Error(), "cell %d", i)
					continue
				}
				require.NoError(t, gotErr, "cell %d", i)
				requireCrossBoreBitsEqual(t, want, got)
			}
			if tc.stride == 8 {
				require.Zero(t, failed)
				return
			}
			require.Positive(t, failed)
		})
	}
}

func crossBoreExactRecord(t *testing.T, radius, bore, height, centerZ float64) *crossCylinderBoreRecord {
	t.Helper()
	doc := New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	target, err := Extrude(t.Context(), doc, s, s.Profiles()[0],
		Distance{D: units.Millimeters(height), Dir: Along})
	require.NoError(t, err)
	tw := sketch.NewWorld()
	ts, err := tw.CreateSketch(tw.YZ())
	require.NoError(t, err)
	toolCenter := ts.CreatePoint(0, centerZ)
	ts.Fix(toolCenter)
	ts.CreateCircle(toolCenter, bore)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	tool, err := Extrude(t.Context(), doc, ts, ts.Profiles()[0], Symmetric{D: units.Millimeters(2*radius + 2)})
	require.NoError(t, err)
	rec := recognizeCrossCylinderBore(target, tool)
	require.NotNil(t, rec)
	return rec
}

func TestCrossBoreStationErrorsMatchSerial(t *testing.T) {
	for _, tc := range []struct {
		R, r, H, c, setback float64
	}{
		{5, 1, 10, 5, 0.5},
		{8, 1.5, 16, 8, 0.1875},
	} {
		t.Run(fmt.Sprintf("%gx%g", tc.R, tc.r), func(t *testing.T) {
			rec := crossBoreExactRecord(t, tc.R, tc.r, tc.H, tc.c)
			serial, err := buildCrossBoreChamferMeshWorkers(t.Context(), rec, tc.setback, 1)
			require.NoError(t, err)
			fanned, err := buildCrossBoreChamferMeshWorkers(t.Context(), rec, tc.setback, 4)
			require.NoError(t, err)
			require.Equal(t, serial.verts, fanned.verts)
			require.Equal(t, serial.tris, fanned.tris)
			require.Equal(t, serial.src, fanned.src)
			require.Equal(t, math.Float64bits(serial.outerStationError), math.Float64bits(fanned.outerStationError))

			stations, err := crossBoreStationPlan(rec, tc.setback)
			require.NoError(t, err)
			radius := new(big.Rat).SetFloat64(rec.Radius)
			want := 0.0
			for _, station := range stations {
				x, y := rec.Radius*math.Cos(station.phi), rec.Radius*math.Sin(station.phi)
				upper := station.lower
				if station.lower.Sub(station.upper).Len() >= 1e-13 {
					upper = station.upper
				}
				for _, p := range []r3.Vec{{X: x, Y: y}, station.lower, upper, {X: x, Y: y, Z: rec.Height}} {
					e, ok := crossBoreStationError(p, radius, station.phi)
					require.True(t, ok)
					want = max(want, e)
				}
			}
			require.Equal(t, math.Float64bits(want), math.Float64bits(serial.outerStationError))
		})
	}
}
