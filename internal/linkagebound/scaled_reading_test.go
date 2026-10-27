package linkagebound

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// legacyJoints and legacyApply are ReadPoints as it was written over big.Rat
// interval arithmetic, split where readPoseBounds takes the composed pose;
// legacyReadPointBounds rounds that reading with RoundCorners.
func legacyJoints(frames []motionbound.MotionFrame, params []motionbound.MotionParam, path []int, revolute []bool, below int) ([]jointAt, *motionbound.IdealPose) {
	joints := make([]jointAt, 0, len(path)-below)
	var pose *motionbound.IdealPose
	for _, i := range path[below:] {
		f := frames[i]
		var unit motionbound.IvVec
		for d := range 3 {
			unit[d] = proofbound.IntervalScale(f.Unit, f.Axis[d])
		}
		pivot := motionbound.PointVec(f.Center)
		ideal := f.At(params[i])
		if pose != nil {
			unit = pose.Rot.Apply(unit)
			pivot = ApplyIdeal(*pose, pivot)
			ideal = ideal.Then(*pose)
		}
		joints = append(joints, jointAt{revolute: revolute[i], unit: unit, pivot: pivot})
		pose = &ideal
	}
	return joints, pose
}

func legacyApply(joints []jointAt, pose *motionbound.IdealPose, out Reading) Reading {
	for c := range out.Pos {
		if pose != nil {
			out.Pos[c] = ApplyIdeal(*pose, out.Pos[c])
		}
		out.Vel[c] = make([]motionbound.IvVec, len(joints))
		for n, j := range joints {
			if !j.revolute {
				out.Vel[c][n] = j.unit
				continue
			}
			out.Vel[c][n] = IvCross(j.unit, motionbound.IvVecSub(out.Pos[c], j.pivot))
		}
	}
	return out
}

func legacyReadPointBounds(frames []motionbound.MotionFrame, params []motionbound.MotionParam, path []int, revolute []bool, below int, out Reading) (Bounds, bool) {
	joints, pose := legacyJoints(frames, params, path, revolute, below)
	return RoundCorners(legacyApply(joints, pose, out))
}

// readingCase is one generated input: a joint chain, its parameters, and a
// point reading the generator can rebuild, since the exact path writes it.
type readingCase struct {
	frames   []motionbound.MotionFrame
	params   []motionbound.MotionParam
	path     []int
	revolute []bool
	below    int
	points   func() Reading
}

var readingAxes = []r3.Vec{
	r3.NewVec(1, 1, 1), r3.NewVec(1, -1, 0), r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0),
	r3.NewVec(0.3, -0.7, 1.9), r3.NewVec(-2, 1e-3, 5),
}

func randomFloatRat(rng *rand.Rand, scale float64) *big.Rat {
	return new(big.Rat).SetFloat64((rng.Float64() - 0.5) * scale)
}

// randomRat is a rational of no special form: an odd denominator times a
// power of two.
func randomRat(rng *rand.Rand) *big.Rat {
	return new(big.Rat).SetFrac(big.NewInt(rng.Int64N(1<<40)-1<<39), big.NewInt((rng.Int64N(1<<20)|1)<<rng.IntN(8)))
}

func randomReadingCase(t *testing.T, rng *rand.Rand, pointsOnly bool) readingCase {
	t.Helper()
	joints := 1 + rng.IntN(4)
	rc := readingCase{
		frames: make([]motionbound.MotionFrame, joints), params: make([]motionbound.MotionParam, joints),
		path: make([]int, joints), revolute: make([]bool, joints), below: rng.IntN(joints + 1),
	}
	for i := range joints {
		rc.path[i] = i
		axis := readingAxes[rng.IntN(len(readingAxes))]
		if rng.IntN(3) == 0 {
			axis = r3.NewVec(rng.Float64()-0.5, rng.Float64()-0.5, rng.Float64()-0.5)
		}
		rc.revolute[i] = rng.IntN(3) != 0
		spec := motionbound.Spec{Kind: motionbound.MotionPrismatic, Dir: axis}
		param := motionbound.MotionParam{Turn: new(big.Rat), Base: randomFloatRat(rng, 60)}
		if rc.revolute[i] {
			center := r3.NewVec((rng.Float64()-0.5)*200, (rng.Float64()-0.5)*200, (rng.Float64()-0.5)*50)
			spec = motionbound.Spec{Kind: motionbound.MotionRevolute, Center: center, Axis: axis}
			param = motionbound.MotionParam{Turn: big.NewRat(rng.Int64N(512)-256, 1024), Base: new(big.Rat)}
			if rng.IntN(2) == 0 {
				param.Base = randomFloatRat(rng, 2)
			}
		}
		if rng.IntN(4) == 0 {
			// A dependent's centre: no special form.
			param.Base = randomRat(rng)
		}
		frame, ok := motionbound.NewMotionFrame(spec)
		require.True(t, ok)
		rc.frames[i], rc.params[i] = frame, param
	}
	seed := rng.Uint64()
	rc.points = func() Reading {
		prng := rand.New(rand.NewPCG(seed, 7))
		if prng.IntN(2) == 0 {
			lo := motionbound.RatVec{randomFloatRat(prng, 100), randomFloatRat(prng, 100), randomFloatRat(prng, 100)}
			hi := motionbound.RatVec{
				new(big.Rat).Add(lo[0], big.NewRat(prng.Int64N(400)+1, 4)),
				new(big.Rat).Add(lo[1], big.NewRat(prng.Int64N(400), 4)),
				new(big.Rat).Add(lo[2], big.NewRat(prng.Int64N(400)+1, 4)),
			}
			return StaticBox(lo, hi)
		}
		points := make([]motionbound.RatVec, 1+prng.IntN(12))
		for i := range points {
			points[i] = motionbound.RatVec{randomFloatRat(prng, 300), randomFloatRat(prng, 300), new(big.Rat)}
			if prng.IntN(3) != 0 {
				points[i][2] = randomFloatRat(prng, 30)
			}
		}
		r := StaticPoints(points)
		r.Pad, r.PrismK = randomFloatRat(prng, 1), prng.IntN(5)
		if !pointsOnly && prng.IntN(3) == 0 {
			// An enclosed point that is not exact takes the exact path.
			r.Pos[0][1] = proofbound.IntervalOwned(new(big.Rat).Set(r.Pos[0][1].Lo), new(big.Rat).Add(r.Pos[0][1].Hi, big.NewRat(1, 3)))
		}
		return r
	}
	return rc
}

// requireSameBounds compares two Bounds exactly: every rational by value and
// in its lowest terms, every float bit for bit.
func requireSameBounds(t *testing.T, want Bounds, wantOK bool, got Bounds, gotOK bool, msg string) {
	t.Helper()
	require.Equal(t, wantOK, gotOK, msg)
	if !wantOK {
		return
	}
	same := func(a, b *big.Rat, what string) {
		t.Helper()
		require.Zero(t, a.Num().Cmp(b.Num()), "%s %s: %v != %v", msg, what, a, b)
		require.Zero(t, a.Denom().Cmp(b.Denom()), "%s %s: %v != %v", msg, what, a, b)
	}
	require.Len(t, got.Lo, len(want.Lo), msg)
	require.Len(t, got.Vel, len(want.Vel), msg)
	require.Equal(t, want.PrismK, got.PrismK, msg)
	require.Equal(t, want.Pad == nil, got.Pad == nil, msg)
	if want.Pad != nil {
		same(want.Pad, got.Pad, "pad")
	}
	for c := range want.Lo {
		for d := range 3 {
			same(want.Lo[c][d], got.Lo[c][d], "lo")
			same(want.Hi[c][d], got.Hi[c][d], "hi")
			require.Equal(t, math.Float64bits(want.LoF[c][d]), math.Float64bits(got.LoF[c][d]), msg)
			require.Equal(t, math.Float64bits(want.HiF[c][d]), math.Float64bits(got.HiF[c][d]), msg)
		}
		require.Len(t, got.Vel[c], len(want.Vel[c]), msg)
		for n := range want.Vel[c] {
			for d := range 3 {
				same(want.Vel[c][n][d].Lo, got.Vel[c][n][d].Lo, "vel lo")
				same(want.Vel[c][n][d].Hi, got.Vel[c][n][d].Hi, "vel hi")
			}
		}
	}
}

func TestReadPointBoundsMatchesExactReading(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(2611, 2617))
	for i := range 400 {
		rc := randomReadingCase(t, rng, false)
		want, wantOK := legacyReadPointBounds(rc.frames, rc.params, rc.path, rc.revolute, rc.below, rc.points())
		got, gotOK := ReadPointBounds(rc.frames, rc.params, rc.path, rc.revolute, rc.below, rc.points())
		requireSameBounds(t, want, wantOK, got, gotOK, fmt.Sprintf("case %d", i))
	}
}

// TestReadPoseBoundsIntervalPivot reads a pose whose pivot is an interval,
// which no joint frame builds but readPoseBounds must still read exactly.
func TestReadPoseBoundsIntervalPivot(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(2621, 2633))
	for range 60 {
		rc := randomReadingCase(t, rng, true)
		joints, pose := legacyJoints(rc.frames, rc.params, rc.path, rc.revolute, rc.below)
		if pose == nil {
			continue
		}
		for d := range 3 {
			pose.Pivot[d] = proofbound.IntervalOwned(new(big.Rat).Sub(pose.Pivot[d].Lo, big.NewRat(1, 3)), new(big.Rat).Set(pose.Pivot[d].Hi))
		}
		want, wantOK := RoundCorners(legacyApply(joints, pose, rc.points()))
		got, gotOK := readPoseBounds(joints, pose, rc.points())
		requireSameBounds(t, want, wantOK, got, gotOK, "interval pivot")
	}
}

// TestReadPointBoundsOverflow fails a reading whose position or velocity
// leaves the floats, as RoundCorners fails it.
func TestReadPointBoundsOverflow(t *testing.T) {
	t.Parallel()
	frame, ok := motionbound.NewMotionFrame(motionbound.Spec{Kind: motionbound.MotionRevolute, Center: r3.NewVec(1, 2, 3), Axis: r3.NewVec(1, 1, 1)})
	require.True(t, ok)
	params := []motionbound.MotionParam{{Turn: big.NewRat(1, 8), Base: new(big.Rat)}}
	huge := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 1030))
	for _, p := range []motionbound.RatVec{
		{huge, new(big.Rat), new(big.Rat)},
		{new(big.Rat).SetFloat64(math.MaxFloat64), new(big.Rat).SetFloat64(-math.MaxFloat64), new(big.Rat)},
	} {
		points := func() Reading {
			return StaticPoints([]motionbound.RatVec{{new(big.Rat).Set(p[0]), new(big.Rat).Set(p[1]), new(big.Rat).Set(p[2])}})
		}
		want, wantOK := legacyReadPointBounds([]motionbound.MotionFrame{frame}, params, []int{0}, []bool{true}, 0, points())
		got, gotOK := ReadPointBounds([]motionbound.MotionFrame{frame}, params, []int{0}, []bool{true}, 0, points())
		require.False(t, wantOK)
		requireSameBounds(t, want, wantOK, got, gotOK, "overflow")
	}
	// A position at the largest float about a pivot at its negation: the
	// position rounds, and its offset of twice the largest float turns into a
	// velocity no float holds.
	far, ok := motionbound.NewMotionFrame(motionbound.Spec{Kind: motionbound.MotionRevolute, Center: r3.NewVec(-math.MaxFloat64, 0, 0), Axis: r3.NewVec(0, 0, 1)})
	require.True(t, ok)
	still := []motionbound.MotionParam{{Turn: new(big.Rat), Base: new(big.Rat)}}
	points := func() Reading {
		return StaticPoints([]motionbound.RatVec{{new(big.Rat).SetFloat64(math.MaxFloat64), new(big.Rat), new(big.Rat)}})
	}
	want, wantOK := legacyReadPointBounds([]motionbound.MotionFrame{far}, still, []int{0}, []bool{true}, 0, points())
	got, gotOK := ReadPointBounds([]motionbound.MotionFrame{far}, still, []int{0}, []bool{true}, 0, points())
	require.False(t, wantOK)
	requireSameBounds(t, want, wantOK, got, gotOK, "velocity overflow")
}

// tiltedCases are the tilted slider-crank's readings (apitest's
// TestVerifyLinkageLoopTiltedSlider): its crank alone, one revolute about
// (1, 1, 1) at a turn of the drive, and the crank with the rod below it, read
// at a dependent's centre; each over a body's eight box corners.
func tiltedCases(b *testing.B) map[string]readingCase {
	b.Helper()
	axis := r3.NewVec(1, 1, 1)
	crank, okC := motionbound.NewMotionFrame(motionbound.Spec{Kind: motionbound.MotionRevolute, Center: r3.NewVec(0.1, -0.2, 0.3), Axis: axis})
	rod, okR := motionbound.NewMotionFrame(motionbound.Spec{Kind: motionbound.MotionRevolute, Center: r3.NewVec(17.3, 12.1, -4.9), Axis: axis})
	require.True(b, okC && okR)
	box := func() Reading {
		lo := motionbound.RatVec{new(big.Rat).SetFloat64(18.371), new(big.Rat).SetFloat64(-3.25), new(big.Rat).SetFloat64(4.5)}
		hi := motionbound.RatVec{new(big.Rat).SetFloat64(78.125), new(big.Rat).SetFloat64(7.75), new(big.Rat).SetFloat64(12.0625)}
		return StaticBox(lo, hi)
	}
	crankAt := motionbound.MotionParam{Turn: big.NewRat(37, 1024), Base: new(big.Rat)}
	return map[string]readingCase{
		"crank": {
			frames: []motionbound.MotionFrame{crank}, params: []motionbound.MotionParam{crankAt},
			path: []int{0}, revolute: []bool{true}, points: box,
		},
		"chain": {
			frames: []motionbound.MotionFrame{crank, rod},
			params: []motionbound.MotionParam{crankAt, {Turn: new(big.Rat), Base: new(big.Rat).SetFloat64(-0.7310585786300049)}},
			path:   []int{0, 1}, revolute: []bool{true, true}, points: box,
		},
	}
}

func BenchmarkReadPointBounds(b *testing.B) {
	for _, name := range []string{"crank", "chain"} {
		rc := tiltedCases(b)[name]
		b.Run(name+"/scaled", func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				if _, ok := ReadPointBounds(rc.frames, rc.params, rc.path, rc.revolute, 0, rc.points()); !ok {
					b.Fatal("reading failed")
				}
			}
		})
		b.Run(name+"/legacy", func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				if _, ok := legacyReadPointBounds(rc.frames, rc.params, rc.path, rc.revolute, 0, rc.points()); !ok {
					b.Fatal("reading failed")
				}
			}
		})
	}
}
