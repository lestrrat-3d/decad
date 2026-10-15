package decad

import (
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCompositeChainSweepAreaSlackEnclosesTheAnalyticSheet(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a, z := s.CreatePoint(0, 0), s.CreatePoint(4, 0)
	s.Fix(a)
	s.Fix(z)
	s.CreateLine(a, z)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	path, err := NewPath(r3.Vec{}, LineTo{End: r3.Vec{Z: 20}},
		ArcThrough{Through: r3.Vec{X: 5, Z: 25}, End: r3.Vec{X: 10, Z: 20}})
	require.NoError(t, err)
	sheet, err := Sweep(t.Context(), New(), s, s.Chains()[0], path)
	require.NoError(t, err)
	all, err := sheet.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.Greater(t, all.n.AreaSlack, 0.0)
	require.Less(t, all.n.AreaSlack, 0.5)

	// The exact held planar area is the sum of dyadic determinants. The true
	// sheet is an 80 mm² rectangle plus a half annulus of area 12π mm².
	held := new(big.Rat)
	for _, tri := range all.n.Triangles {
		a, b, c := all.n.Vertices[tri[0]], all.n.Vertices[tri[1]], all.n.Vertices[tri[2]]
		ux := new(big.Rat).Sub(proofarith.FloatRat(b.X), proofarith.FloatRat(a.X))
		uz := new(big.Rat).Sub(proofarith.FloatRat(b.Z), proofarith.FloatRat(a.Z))
		vx := new(big.Rat).Sub(proofarith.FloatRat(c.X), proofarith.FloatRat(a.X))
		vz := new(big.Rat).Sub(proofarith.FloatRat(c.Z), proofarith.FloatRat(a.Z))
		cross := new(big.Rat).Sub(new(big.Rat).Mul(ux, vz), new(big.Rat).Mul(uz, vx))
		held.Add(held, new(big.Rat).Quo(cross.Abs(cross), big.NewRat(2, 1)))
	}
	trueLo := new(big.Rat).Add(big.NewRat(80, 1), new(big.Rat).Mul(big.NewRat(12, 1), proofbound.PiLower))
	trueHi := new(big.Rat).Add(big.NewRat(80, 1), new(big.Rat).Mul(big.NewRat(12, 1), proofbound.PiUpper))
	dLo := new(big.Rat).Abs(new(big.Rat).Sub(trueLo, held))
	dHi := new(big.Rat).Abs(new(big.Rat).Sub(trueHi, held))
	require.GreaterOrEqual(t, proofarith.FloatRat(all.n.AreaSlack).Cmp(proofbound.RatMax(dLo, dHi)), 0)

	none, err := sheet.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyNone))
	require.NoError(t, err)
	require.Equal(t, all.n.Vertices, none.n.Vertices)
	require.Equal(t, all.n.Triangles, none.n.Triangles)
	require.Equal(t, all.n.Source, none.n.Source)
	require.Zero(t, none.n.AreaSlack)
	require.False(t, none.n.BoundaryOK)
	require.False(t, none.n.SymDiffOK)
}
