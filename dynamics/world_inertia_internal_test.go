package dynamics

import (
	"math"
	"math/rand/v2"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// inertiaMemoTestWorld holds three dynamic bodies: two with different
// off-diagonal inertias, and a singular one whose inversion r3 refuses in
// axis-aligned poses (rounding leaves some rotated copies invertible).
func inertiaMemoTestWorld() *World {
	reading := func(v float64) decad.Measurement {
		return decad.Measurement{Value: units.KilogramSquareMillimeters(v),
			Bound: units.KilogramSquareMillimeters(0), Exactness: decad.Exact}
	}
	body := func(xx, yy, zz, xy, xz, yz float64) worldBody {
		return worldBody{mass: decad.MassProperties{Inertia: decad.InertiaReading{XX: reading(xx), YY: reading(yy),
			ZZ: reading(zz), XY: reading(xy), XZ: reading(xz), YZ: reading(yz)}}}
	}
	return &World{bodies: []worldBody{
		body(3e6, 2e6, 4e6, 1e5, -2e5, 5e4),
		body(1.25e5, 7.5e5, 6.1e5, -3.3e4, 1.7e4, 8.9e4),
		body(1e6, 1e6, 0, 0, 0, 0),
	}, inertia: &worldInertiaMemo{}}
}

// rotatedInertiaUncached is nominalBodies' and kickAngular's rotation
// before the memo: build the body-frame tensor, rotate it into the pose's
// axes, invert it there.
func rotatedInertiaUncached(inertia decad.InertiaReading, pose r3.Transform) (r3.SymmetricTensor,
	r3.SymmetricTensor, error) {
	local, err := r3.NewSymmetricTensor(inertia.XX.Value.Base(), inertia.YY.Value.Base(),
		inertia.ZZ.Value.Base(), inertia.XY.Value.Base(), inertia.XZ.Value.Base(), inertia.YZ.Value.Base())
	if err != nil {
		return r3.SymmetricTensor{}, r3.SymmetricTensor{}, err
	}
	world, err := local.Rotate(pose)
	if err != nil {
		return r3.SymmetricTensor{}, r3.SymmetricTensor{}, err
	}
	inverse, err := world.Inverse()
	if err != nil {
		return r3.SymmetricTensor{}, r3.SymmetricTensor{}, err
	}
	return world, inverse, nil
}

func tensorBits(s r3.SymmetricTensor) [6]uint64 {
	xx, yy, zz, xy, xz, yz := s.Components()
	return [6]uint64{math.Float64bits(xx), math.Float64bits(yy), math.Float64bits(zz),
		math.Float64bits(xy), math.Float64bits(xz), math.Float64bits(yz)}
}

// inertiaMemoTestPoses returns random rotations, each at two translations
// (one basis, so one key), turns sharing one basis vector, and the identity
// basis with a −0 entry, which == cannot tell from the identity.
func inertiaMemoTestPoses(t *testing.T, rng *rand.Rand) []r3.Transform {
	t.Helper()
	var out []r3.Transform
	for range 12 {
		turn, err := r3.Rotation(r3.Vec{X: rng.Float64() - 0.5, Y: rng.Float64() - 0.5, Z: rng.Float64() - 0.5},
			units.Radians(rng.Float64()*2*math.Pi))
		require.NoError(t, err)
		for range 2 {
			pose, err := r3.FromBasis(turn.Basis(), r3.Vec{X: rng.Float64()*100 - 50, Z: rng.Float64() * 20})
			require.NoError(t, err)
			out = append(out, pose)
		}
	}
	// Turns about one axis share that basis vector's bits, so a key that
	// read only some basis vectors would conflate them.
	for _, turn := range []struct {
		axis  r3.Vec
		angle float64
	}{{r3.Vec{X: 1}, 0.3}, {r3.Vec{X: 1}, 1.1}, {r3.Vec{Y: 1}, 0.7}, {r3.Vec{Y: 1}, 2}} {
		pose, err := r3.Rotation(turn.axis, units.Radians(turn.angle))
		require.NoError(t, err)
		out = append(out, pose)
	}
	signed, err := r3.FromBasis(r3.Basis{EX: r3.Vec{X: 1, Y: math.Copysign(0, -1)}, EY: r3.Vec{Y: 1},
		EZ: r3.Vec{Z: 1}}, r3.Vec{})
	require.NoError(t, err)
	require.True(t, signed == r3.Identity(), "premise: == cannot tell the bases apart")
	return append(out, r3.Identity(), signed)
}

// TestWorldInertiaMemoMatchesUncachedRotation reads every body at every
// pose through the memo three times, the second read right after the first
// and the third after every other pose. Each tensor and inverse must equal
// the uncached rotation bit for bit, a refusal must stay a refusal, and only
// the first read of each body and basis may miss.
func TestWorldInertiaMemoMatchesUncachedRotation(t *testing.T) {
	t.Parallel()
	w := inertiaMemoTestWorld()
	poses := inertiaMemoTestPoses(t, rand.New(rand.NewPCG(13, 17)))
	distinct := map[worldInertiaKey]struct{}{}
	refused := 0
	check := func(index int, pose r3.Transform) {
		wantTensor, wantInverse, wantErr := rotatedInertiaUncached(w.bodies[index].mass.Inertia, pose)
		got, err := w.worldInertia(index, pose)
		if wantErr != nil {
			require.Equal(t, wantErr, err, "body %d", index)
			refused++
			return
		}
		require.NoError(t, err, "body %d", index)
		require.Equal(t, tensorBits(wantTensor), tensorBits(got.tensor), "body %d", index)
		require.Equal(t, tensorBits(wantInverse), tensorBits(got.inverse), "body %d", index)
		distinct[newWorldInertiaKey(index, pose)] = struct{}{}
	}
	reads := 0
	for _, pose := range poses {
		for index := range w.bodies {
			check(index, pose)
			check(index, pose)
			reads += 2
		}
	}
	for _, pose := range poses {
		for index := range w.bodies {
			check(index, pose)
			reads++
		}
	}
	require.Positive(t, refused, "premise: the singular body is refused")
	require.Len(t, w.inertia.entries, len(distinct), "a refusal never enters the memo")
	require.Less(t, len(distinct), worldInertiaMemoCap, "premise: nothing is evicted")
	require.Equal(t, uint64(len(distinct)+refused), w.inertia.misses, "one miss per body and basis, and per refusal")
	require.Equal(t, uint64(reads-len(distinct)-refused), w.inertia.hits, "every repeat is served")
	first := 0
	for key := range distinct {
		if key.body == 0 {
			first++
		}
	}
	require.Equal(t, 12+4+2, first, "premise: two translations share a basis; −0 and +0 do not")
}

// TestWorldInertiaBypassesInvalidPose requires a pose r3 refuses to raise
// the same error with or without the memo and to leave the memo untouched.
func TestWorldInertiaBypassesInvalidPose(t *testing.T) {
	t.Parallel()
	w := inertiaMemoTestWorld()
	_, _, want := rotatedInertiaUncached(w.bodies[0].mass.Inertia, r3.Transform{})
	require.Error(t, want, "premise: the zero transform is not a pose")
	for range 2 {
		_, err := w.worldInertia(0, r3.Transform{})
		require.Equal(t, want, err)
	}
	require.Empty(t, w.inertia.entries)
	require.Zero(t, w.inertia.hits+w.inertia.misses)
}

// TestWorldInertiaMemoIsSafeConcurrently reads one World from several
// goroutines at once, as concurrent Step calls may; run under -race it fails
// if the memo is read or written unguarded.
func TestWorldInertiaMemoIsSafeConcurrently(t *testing.T) {
	t.Parallel()
	w := inertiaMemoTestWorld()
	poses := inertiaMemoTestPoses(t, rand.New(rand.NewPCG(19, 23)))
	var wg sync.WaitGroup
	got := make([][]worldInertia, 4)
	for g := range got {
		got[g] = make([]worldInertia, len(poses))
		wg.Go(func() {
			for i, pose := range poses {
				got[g][i], _ = w.worldInertia(1, pose)
			}
		})
	}
	wg.Wait()
	for i, pose := range poses {
		wantTensor, wantInverse, err := rotatedInertiaUncached(w.bodies[1].mass.Inertia, pose)
		require.NoError(t, err)
		for g := range got {
			require.Equal(t, tensorBits(wantTensor), tensorBits(got[g][i].tensor))
			require.Equal(t, tensorBits(wantInverse), tensorBits(got[g][i].inverse))
		}
	}
}

// TestWorldInertiaMemoEvictsOldest fills a memo past its bound: it keeps
// exactly worldInertiaMemoCap tensors, the oldest leaves first, and storing
// a key it holds keeps the first tensor.
func TestWorldInertiaMemoEvictsOldest(t *testing.T) {
	t.Parallel()
	keyAt := func(i int) worldInertiaKey { return worldInertiaKey{body: i} }
	valueAt := func(i int) worldInertia {
		tensor, err := r3.NewSymmetricTensor(float64(i+1), 1, 1, 0, 0, 0)
		require.NoError(t, err)
		return worldInertia{tensor: tensor}
	}
	var memo worldInertiaMemo
	for i := range worldInertiaMemoCap + 3 {
		memo.store(keyAt(i), valueAt(i))
	}
	require.Len(t, memo.entries, worldInertiaMemoCap)
	for i := range 3 {
		_, ok := memo.load(keyAt(i))
		require.False(t, ok, "entry %d is among the oldest", i)
	}
	for _, i := range []int{3, worldInertiaMemoCap, worldInertiaMemoCap + 2} {
		value, ok := memo.load(keyAt(i))
		require.True(t, ok, "entry %d", i)
		require.Equal(t, tensorBits(valueAt(i).tensor), tensorBits(value.tensor))
	}
	memo.store(keyAt(5), valueAt(99))
	value, ok := memo.load(keyAt(5))
	require.True(t, ok)
	require.Equal(t, tensorBits(valueAt(5).tensor), tensorBits(value.tensor))
	require.Len(t, memo.entries, worldInertiaMemoCap)
	memo.store(keyAt(0), valueAt(0))
	_, ok = memo.load(keyAt(3))
	require.False(t, ok, "the next oldest leaves for the next new key")
}
