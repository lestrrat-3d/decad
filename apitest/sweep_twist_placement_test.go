package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file checks a placed twisted sweep's volume and area against the solid
// its record denotes: the plane-local sweep carried through the placement's
// held basis B read as exact rationals (agent-docs/sweep-design.md §17.5).
// r3.Transform.Then does not re-orthonormalize, so a long chain of float
// rotations holds a B whose volume and area factors depart from 1 by far more
// than the sweep's own closed-form enclosures.

// twistMapRat reads a float exactly.
func twistMapRat(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }

// twistMapDriftedPose composes n float rotations. Each Then rounds the basis
// again, so the held basis drifts from orthonormal.
func twistMapDriftedPose(t *testing.T, n int) r3.Transform {
	t.Helper()
	step, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Radians(0.1))
	require.NoError(t, err)
	pose := r3.Identity()
	for range n {
		pose, err = pose.Then(step)
		require.NoError(t, err)
	}
	return pose
}

// twistMapFactors reads the exact volume factor |det B| of a placement's held
// basis B and an exact range [lo, hi] holding the squared area factor of every
// plane. A plane with unit normal n maps to area factor |cof(B)·n|, whose
// square nᵀMn with M = cof(B)ᵀcof(B) lies within M's Gershgorin interval.
func twistMapFactors(pose r3.Transform) (*big.Rat, *big.Rat, *big.Rat) {
	basis := pose.Basis()
	var b [3][3]*big.Rat // b[i][k] is row i, column k.
	for k, col := range []r3.Vec{basis.EX, basis.EY, basis.EZ} {
		for i, x := range []float64{col.X, col.Y, col.Z} {
			b[i][k] = twistMapRat(x)
		}
	}
	var cof [3][3]*big.Rat
	for i := range 3 {
		for k := range 3 {
			i1, i2, k1, k2 := (i+1)%3, (i+2)%3, (k+1)%3, (k+2)%3
			cof[i][k] = new(big.Rat).Sub(new(big.Rat).Mul(b[i1][k1], b[i2][k2]),
				new(big.Rat).Mul(b[i1][k2], b[i2][k1]))
		}
	}
	det := new(big.Rat)
	for k := range 3 {
		det.Add(det, new(big.Rat).Mul(b[0][k], cof[0][k]))
	}
	var lo, hi *big.Rat
	for r := range 3 {
		diag, off := new(big.Rat), new(big.Rat)
		for c := range 3 {
			m := new(big.Rat)
			for i := range 3 {
				m.Add(m, new(big.Rat).Mul(cof[i][r], cof[i][c]))
			}
			if r == c {
				diag = m
				continue
			}
			off.Add(off, m.Abs(m))
		}
		rowLo, rowHi := new(big.Rat).Sub(diag, off), new(big.Rat).Add(diag, off)
		if lo == nil || rowLo.Cmp(lo) < 0 {
			lo = rowLo
		}
		if hi == nil || rowHi.Cmp(hi) > 0 {
			hi = rowHi
		}
	}
	return det.Abs(det), lo, hi
}

// requireVolumeAdmits asserts that the published volume can hold the true
// placed volume, which lies in |det B|·[v0 − b0, v0 + b0] for the unplaced
// reading v0 ± b0. A published interval missing that whole range is disproven.
func requireVolumeAdmits(t *testing.T, got, local decad.Measurement, det *big.Rat) {
	t.Helper()
	v, b := twistMapRat(got.Value.Base()), twistMapRat(got.Bound.Base())
	v0, b0 := twistMapRat(local.Value.Base()), twistMapRat(local.Bound.Base())
	trueLo := new(big.Rat).Mul(det, new(big.Rat).Sub(v0, b0))
	trueHi := new(big.Rat).Mul(det, new(big.Rat).Add(v0, b0))
	require.True(t, new(big.Rat).Sub(v, b).Cmp(trueHi) <= 0,
		"the true placed volume lies above %v - %v", got.Value.Base(), got.Bound.Base())
	require.True(t, new(big.Rat).Add(v, b).Cmp(trueLo) >= 0,
		"the true placed volume lies below %v + %v", got.Value.Base(), got.Bound.Base())
}

// requireAreaAdmits asserts that the published area can hold the true placed
// area, whose square lies in [lo·(a0 − b0)², hi·(a0 + b0)²] for the unplaced
// reading a0 ± b0 and the squared area factor range [lo, hi].
func requireAreaAdmits(t *testing.T, what string, got, local decad.Measurement, lo, hi *big.Rat) {
	t.Helper()
	a0, b0 := twistMapRat(local.Value.Base()), twistMapRat(local.Bound.Base())
	localLo, localHi := new(big.Rat).Sub(a0, b0), new(big.Rat).Add(a0, b0)
	trueHi2 := new(big.Rat).Mul(hi, new(big.Rat).Mul(localHi, localHi))
	gotLo := new(big.Rat).Sub(twistMapRat(got.Value.Base()), twistMapRat(got.Bound.Base()))
	gotHi := new(big.Rat).Add(twistMapRat(got.Value.Base()), twistMapRat(got.Bound.Base()))
	require.True(t, gotLo.Sign() <= 0 || new(big.Rat).Mul(gotLo, gotLo).Cmp(trueHi2) <= 0,
		"%s: the true placed area lies above %v - %v", what, got.Value.Base(), got.Bound.Base())
	if localLo.Sign() <= 0 {
		return
	}
	trueLo2 := new(big.Rat).Mul(lo, new(big.Rat).Mul(localLo, localLo))
	require.True(t, new(big.Rat).Mul(gotHi, gotHi).Cmp(trueLo2) >= 0,
		"%s: the true placed area lies below %v + %v", what, got.Value.Base(), got.Bound.Base())
}

// twistMapSquare builds a centred square of half-width half on the world XY
// sketch plane, the plane every twisted sweep admits.
func twistMapSquare(t *testing.T, half float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-half, -half, half, half)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// twistMapCase is one twisted sweep of a profile on the world XY sketch
// plane. exactVolume, when nonzero, is the straight path's exact product of
// profile area and height.
type twistMapCase struct {
	name        string
	profile     func(t *testing.T) (*sketch.Sketch, *sketch.Profile)
	segments    []decad.PathSegment
	degrees     float64
	exactVolume float64
}

// twistMapCases covers the three twisted-sweep builders: one straight Loft
// (an angle up to one radian), stationed Lofts (a larger angle, or a profile
// with Arc corners), and a line followed by a cardinal quarter arc.
func twistMapCases() []twistMapCase {
	square := func(half float64) func(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
		return func(t *testing.T) (*sketch.Sketch, *sketch.Profile) { return twistMapSquare(t, half) }
	}
	rounded := func(t *testing.T) (*sketch.Sketch, *sketch.Profile) { return roundedTwistProfile(t, 0) }
	line := func(h float64) []decad.PathSegment {
		return []decad.PathSegment{decad.LineTo{End: r3.NewVec(0, 0, h)}}
	}
	return []twistMapCase{
		{"straight", square(50), line(2), 30, 20000},
		{"stationed", square(50), line(2), 90, 20000},
		{"rounded", rounded, line(5), 30, 0},
		{"lineArc", square(2), []decad.PathSegment{decad.LineTo{End: r3.NewVec(0, 0, 10)},
			decad.ArcThrough{Through: r3.NewVec(20, 0, 50), End: r3.NewVec(50, 0, 60)}}, 5, 0},
	}
}

func twistMapSweep(t *testing.T, c twistMapCase) *decad.Body {
	t.Helper()
	s, p := c.profile(t)
	path, err := decad.NewPath(r3.Vec{}, c.segments...)
	require.NoError(t, err)
	body, err := decad.Sweep(t.Context(), decad.New(), s, p, path, decad.WithSweepTwist(units.Degrees(c.degrees)))
	require.NoError(t, err)
	return body
}

func TestSweepTwistChargesPlacementMap(t *testing.T) {
	t.Parallel()
	pose := twistMapDriftedPose(t, 5000)
	det, lo, hi := twistMapFactors(pose)
	for _, c := range twistMapCases() {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			body := twistMapSweep(t, c)
			placed, err := decad.Transform(t.Context(), body, pose)
			require.NoError(t, err)
			v0, err := body.Volume()
			require.NoError(t, err)
			v, err := placed.Volume()
			require.NoError(t, err)
			requireVolumeAdmits(t, v, v0, det)
			a0, err := body.Area()
			require.NoError(t, err)
			a, err := placed.Area()
			require.NoError(t, err)
			requireAreaAdmits(t, "body", a, a0, lo, hi)
			// Every face's published area must hold its own true placed area,
			// whose unplaced reading the same face of the unplaced body gives.
			local := map[string]decad.Measurement{}
			for _, f := range body.Faces() {
				m, err := f.Area()
				require.NoError(t, err)
				local[f.Origins()[0].Role] = m
			}
			require.Len(t, placed.Faces(), len(local))
			for _, f := range placed.Faces() {
				role := f.Origins()[0].Role
				m, err := f.Area()
				require.NoError(t, err)
				require.Contains(t, local, role)
				requireAreaAdmits(t, role, m, local[role], lo, hi)
			}
		})
	}
}

func TestSweepTwistExactPlacementKeepsReadings(t *testing.T) {
	t.Parallel()
	// A translation holds the identity basis exactly, so the map charge is
	// zero and every reading keeps its unplaced value and bound bit for bit.
	move, err := r3.Translation(r3.NewVec(7, -3, 11))
	require.NoError(t, err)
	for _, c := range twistMapCases() {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			body := twistMapSweep(t, c)
			placed, err := decad.Transform(t.Context(), body, move)
			require.NoError(t, err)
			v0, err := body.Volume()
			require.NoError(t, err)
			v, err := placed.Volume()
			require.NoError(t, err)
			require.Equal(t, v0, v)
			a0, err := body.Area()
			require.NoError(t, err)
			a, err := placed.Area()
			require.NoError(t, err)
			require.Equal(t, a0, a)
			if c.exactVolume != 0 {
				// A straight polygon twist's volume is the exact product of
				// the profile area and the path height.
				require.Equal(t, decad.Exact, v.Exactness)
				require.InDelta(t, c.exactVolume, v.Value.Base(), 0)
			}
		})
	}
}
