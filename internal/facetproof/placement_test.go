package facetproof_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The Dyadic affine image spells the same homogeneous point as the big.Rat
// image meshbool.ExactRigidImage builds, and XptDenotes agrees with the sign of
// CoordDistance.
func TestExactAffineImageMatchesRigidImage(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(5, 9))
	for i := range 1500 {
		axis := r3.Vec{X: r.Float64() - 0.5, Y: r.Float64() - 0.5, Z: r.Float64() - 0.5}
		rot, err := r3.Rotation(axis, units.Degrees(r.Float64()*360))
		require.NoError(t, err)
		tr := r3.Vec{X: (r.Float64() - 0.5) * 20, Y: (r.Float64() - 0.5) * 20, Z: (r.Float64() - 0.5) * 20}
		if i%7 == 0 {
			tr = r3.Vec{}
		}
		if i%11 == 0 {
			rot = r3.Identity()
		}
		shift, err := r3.Translation(tr)
		require.NoError(t, err)
		xf, err := shift.Then(rot)
		require.NoError(t, err)
		b := xf.Basis()
		scale := math.Ldexp(1, r.IntN(20)-10)
		p := r3.Vec{X: (r.Float64() - 0.5) * scale, Y: (r.Float64() - 0.5) * scale, Z: (r.Float64() - 0.5) * scale}
		if i%5 == 0 {
			p.Y = 0
		}
		want := meshbool.ExactRigidImage(b, xf.Translation(), p)
		got := proof.ExactAffineImage(b.EX, b.EY, b.EZ, xf.Translation(), p)
		require.Equal(t, 0, want.X.Cmp(got.X))
		require.Equal(t, 0, want.Y.Cmp(got.Y))
		require.Equal(t, 0, want.Z.Cmp(got.Z))
		require.Equal(t, 0, want.W.Cmp(got.W))
		held := got.Vec()
		require.Equal(t, meshbool.CoordDistance(got, held).Sign() == 0, proof.XptDenotes(got, held))
		require.Equal(t, meshbool.CoordDistance(got, p).Sign() == 0, proof.XptDenotes(got, p))
	}
}
