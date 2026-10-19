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

// inertiaTestWorld holds three dynamic bodies: two with different
// off-diagonal inertias, and a singular one whose inversion r3 refuses in
// axis-aligned poses (rounding leaves some rotated copies invertible).
func inertiaTestWorld() *World {
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

func requireTensorClose(t *testing.T, want, got r3.SymmetricTensor) {
	t.Helper()
	wxx, wyy, wzz, wxy, wxz, wyz := want.Components()
	gxx, gyy, gzz, gxy, gxz, gyz := got.Components()
	wantValues := [...]float64{wxx, wyy, wzz, wxy, wxz, wyz}
	gotValues := [...]float64{gxx, gyy, gzz, gxy, gxz, gyz}
	for i := range wantValues {
		require.InDelta(t, wantValues[i], gotValues[i], 1e-8)
	}
}

func inertiaTestPoses(t *testing.T, rng *rand.Rand) []r3.Transform {
	t.Helper()
	var out []r3.Transform
	for range 12 {
		turn, err := r3.Rotation(r3.Vec{X: rng.Float64() - 0.5, Y: rng.Float64() - 0.5, Z: rng.Float64() - 0.5},
			units.Radians(rng.Float64()*2*math.Pi))
		require.NoError(t, err)
		out = append(out, turn)
	}
	for _, turn := range []struct {
		axis  r3.Vec
		angle float64
	}{{r3.Vec{X: 1}, 0.3}, {r3.Vec{X: 1}, 1.1}, {r3.Vec{Y: 1}, 0.7}, {r3.Vec{Y: 1}, 2}} {
		pose, err := r3.Rotation(turn.axis, units.Radians(turn.angle))
		require.NoError(t, err)
		out = append(out, pose)
	}
	return append(out, r3.Identity())
}

// TestWorldInertiaMatchesRotatedTensor checks rotated inertia readings.
func TestWorldInertiaMatchesRotatedTensor(t *testing.T) {
	t.Parallel()
	w := inertiaTestWorld()
	poses := inertiaTestPoses(t, rand.New(rand.NewPCG(13, 17)))
	refused := 0
	check := func(index int, pose r3.Transform) {
		wantTensor, wantInverse, wantErr := rotatedInertiaUncached(w.bodies[index].mass.Inertia, pose)
		got, err := w.worldInertia(index, pose)
		if wantErr != nil {
			require.Error(t, err, "body %d", index)
			refused++
			return
		}
		require.NoError(t, err, "body %d", index)
		requireTensorClose(t, wantTensor, got.tensor)
		requireTensorClose(t, wantInverse, got.inverse)
	}
	for _, pose := range poses {
		for index := range w.bodies {
			check(index, pose)
			check(index, pose)
		}
	}
	for _, pose := range poses {
		for index := range w.bodies {
			check(index, pose)
		}
	}
	require.Positive(t, refused, "premise: the singular body is refused")
}

// TestWorldInertiaBypassesInvalidPose requires a pose r3 refuses to raise
// the same error with or without the memo and to leave the memo untouched.
func TestWorldInertiaBypassesInvalidPose(t *testing.T) {
	t.Parallel()
	w := inertiaTestWorld()
	_, _, want := rotatedInertiaUncached(w.bodies[0].mass.Inertia, r3.Transform{})
	require.Error(t, want, "premise: the zero transform is not a pose")
	for range 2 {
		_, err := w.worldInertia(0, r3.Transform{})
		require.Equal(t, want, err)
	}
}

// TestWorldInertiaMemoIsSafeConcurrently reads one World from several
// goroutines at once, as concurrent Step calls may; run under -race it fails
// if the memo is read or written unguarded.
func TestWorldInertiaMemoIsSafeConcurrently(t *testing.T) {
	t.Parallel()
	w := inertiaTestWorld()
	poses := inertiaTestPoses(t, rand.New(rand.NewPCG(19, 23)))
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
			requireTensorClose(t, wantTensor, got[g][i].tensor)
			requireTensorClose(t, wantInverse, got[g][i].inverse)
		}
	}
}
