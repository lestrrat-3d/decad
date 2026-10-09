package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/multibody-dynamics-design.md §13 PR 14's ladder half: it
// calls each step of §8.5's tolerance ladder directly, so the steps stay
// observable whichever arm MassProperties dispatches a fixture to.
//
// Each widening leg heldMeshMassProperties adds was deleted in turn and the
// fixture named went red, then restored:
//
//   - V by E: TestMeshMassLadderNarrowsSphereTensor, the k = 8 mass misses
//     4πr³ρ/3 (the inscribed mesh holds less volume than the ball).
//   - P_i by R_i·E: TestMeshMassFirstMomentLegCoversQuarterTurn, the k = 8
//     center misses 4r/3π on the curved axes; the sphere's k = 8 center also
//     leaves its bound, since its mesh is not symmetric about the anchor.
//   - Q_ij by R_i·R_j·E: TestMeshMassLadderNarrowsSphereTensor, the k = 8
//     diagonal components miss 2Mr²/5.
//
// The two inputs to R_i, the body box's own Bound and the mesh's own vertex
// extent, are not exercised: every fixture here has an exact box (Bound 0)
// and a mesh inside it, so deleting either leaves R_i unchanged.

// meshLadderDensity is dyadic, so ρ enters every expected value exactly.
var meshLadderDensity = units.KilogramsPerCubicMillimeter(1.0 / 1024)

var meshLadderPi, _ = new(big.Rat).SetString("3.14159265358979323846264338327950288419716939937510582097494459230781640628620899")

// meshLadderRevolve revolves the profile build draws on the XY plane about
// the world X axis by extent.
func meshLadderRevolve(t *testing.T, extent AngularExtent, build func(s *sketch.Sketch)) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	build(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().Revolve(s, s.Profiles()[0], SketchLine{End: Point2{U: 1}}, extent)
	require.NoError(t, err)
	return body
}

// meshLadderStep reads the ladder's k-th step for body.
func meshLadderStep(t *testing.T, body *Body, k int) (MassProperties, error) {
	t.Helper()
	diameter := body.bounds.Max.Sub(body.bounds.Min).Len()
	return meshMassPropertiesAt(t.Context(), body, math.Ldexp(diameter, -k), meshLadderDensity)
}

func requireMeshReadingCovers(t *testing.T, reading Measurement, exact *big.Rat) {
	t.Helper()
	deviation := new(big.Rat).Sub(exact, new(big.Rat).SetFloat64(reading.Value.Base()))
	require.LessOrEqual(t, deviation.Abs(deviation).Cmp(new(big.Rat).SetFloat64(reading.Bound.Base())), 0)
}

// TestMeshMassLadderNarrowsSphereTensor walks a radius-8 ball through the
// ladder. Every step a revolve mesh reaches (k = 8, 9, 10) encloses
// M = 4πr³ρ/3 and I = 2Mr²/5, and each step's mass and six tensor bounds are
// strictly narrower than the step before: E halves with the tolerance. From
// k = 11 the revolve's own facet-contact audit refuses the mesh at the fixed
// facet-pair ceiling, which ends the ladder. The ladder itself returns the
// first step, the one whose interval first proves positive.
func TestMeshMassLadderNarrowsSphereTensor(t *testing.T) {
	ball := meshLadderRevolve(t, FullRevolution{}, func(s *sketch.Sketch) {
		o := s.CreatePoint(-8, 0)
		s.Fix(o)
		end := s.CreatePoint(8, 0)
		s.CreateLine(o, end)
		s.CreateArc(s.CreatePoint(0, 0), end, o)
	})
	rho := new(big.Rat).SetFloat64(meshLadderDensity.Base())
	mass := new(big.Rat).Mul(rho, new(big.Rat).Mul(meshLadderPi, big.NewRat(2048, 3)))
	inertia := new(big.Rat).Mul(mass, big.NewRat(128, 5))
	zero := new(big.Rat)

	var previous MassProperties
	for k := meshLadderFirst; k <= meshLadderLast; k++ {
		got, err := meshLadderStep(t, ball, k)
		if k > 10 {
			require.ErrorIs(t, err, ErrUnsupported, "k=%d", k)
			require.NotErrorIs(t, err, errMassIntervalUnproved, "k=%d is refused by the tessellation, not by the interval", k)
			continue
		}
		require.NoError(t, err, "k=%d", k)
		requireMeshReadingCovers(t, got.Mass, mass)
		for _, diagonal := range []Measurement{got.Inertia.XX, got.Inertia.YY, got.Inertia.ZZ} {
			requireMeshReadingCovers(t, diagonal, inertia)
		}
		for _, mixed := range []Measurement{got.Inertia.XY, got.Inertia.XZ, got.Inertia.YZ} {
			requireMeshReadingCovers(t, mixed, zero)
		}
		for _, held := range []float64{got.Center.Value.X, got.Center.Value.Y, got.Center.Value.Z} {
			require.LessOrEqual(t, math.Abs(held), got.Center.Bound.Base())
		}
		if k > meshLadderFirst {
			for i, pair := range [][2]Measurement{
				{previous.Mass, got.Mass},
				{previous.Inertia.XX, got.Inertia.XX}, {previous.Inertia.YY, got.Inertia.YY},
				{previous.Inertia.ZZ, got.Inertia.ZZ}, {previous.Inertia.XY, got.Inertia.XY},
				{previous.Inertia.XZ, got.Inertia.XZ}, {previous.Inertia.YZ, got.Inertia.YZ},
			} {
				require.Less(t, pair[1].Bound.Base(), pair[0].Bound.Base(), "k=%d reading %d", k, i)
			}
		}
		previous = got
	}

	first, err := meshLadderStep(t, ball, meshLadderFirst)
	require.NoError(t, err)
	ladder, err := verifiedMeshMassProperties(t.Context(), ball, meshLadderDensity)
	require.NoError(t, err)
	require.Equal(t, first, ladder)
}

// TestMeshMassLadderRefinesUntilPositive is a thin torus (R 10, r 1) whose
// k = 8 tensor interval does not prove positive: the ladder goes on to k = 9
// and returns that step.
func TestMeshMassLadderRefinesUntilPositive(t *testing.T) {
	torus := meshLadderRevolve(t, FullRevolution{}, func(s *sketch.Sketch) {
		center := s.CreatePoint(0, 10)
		s.Fix(center)
		s.CreateCircle(center, 1)
	})
	_, err := meshLadderStep(t, torus, meshLadderFirst)
	require.ErrorIs(t, err, errMassIntervalUnproved)
	second, err := meshLadderStep(t, torus, meshLadderFirst+1)
	require.NoError(t, err)
	ladder, err := verifiedMeshMassProperties(t.Context(), torus, meshLadderDensity)
	require.NoError(t, err)
	require.Equal(t, second, ladder)

	// M = 2π²Rr²ρ, axial I = M(R² + 3r²/4), transverse M(R²/2 + 5r²/8).
	rho := new(big.Rat).SetFloat64(meshLadderDensity.Base())
	mass := new(big.Rat).Mul(rho, new(big.Rat).Mul(new(big.Rat).Mul(meshLadderPi, meshLadderPi), big.NewRat(20, 1)))
	requireMeshReadingCovers(t, second.Mass, mass)
	requireMeshReadingCovers(t, second.Inertia.XX, new(big.Rat).Mul(mass, big.NewRat(403, 4)))
	requireMeshReadingCovers(t, second.Inertia.YY, new(big.Rat).Mul(mass, big.NewRat(405, 8)))
	requireMeshReadingCovers(t, second.Inertia.ZZ, new(big.Rat).Mul(mass, big.NewRat(405, 8)))
}

// TestMeshMassFirstMomentLegCoversQuarterTurn is the first-moment leg's
// fixture: a quarter turn of the r 8, length 10 rectangle, whose center sits
// off the anchor at (5, 4r/3π, 4r/3π) while the inscribed mesh's center sits
// nearer the axis.
func TestMeshMassFirstMomentLegCoversQuarterTurn(t *testing.T) {
	quarter := meshLadderRevolve(t, AngleExtent{A: units.Degrees(90), Dir: Along}, func(s *sketch.Sketch) {
		rect := s.CreateRectangle(0, 0, 10, 8)
		s.Fix(rect.A)
	})
	got, err := meshLadderStep(t, quarter, meshLadderFirst)
	require.NoError(t, err)
	offset := new(big.Rat).Quo(big.NewRat(32, 3), meshLadderPi)
	for i, want := range []*big.Rat{big.NewRat(5, 1), offset, offset} {
		held := []float64{got.Center.Value.X, got.Center.Value.Y, got.Center.Value.Z}[i]
		deviation := new(big.Rat).Sub(want, new(big.Rat).SetFloat64(held))
		require.LessOrEqual(t, deviation.Abs(deviation).Cmp(new(big.Rat).SetFloat64(got.Center.Bound.Base())), 0)
	}
}

// TestMeshMassReadsARevolveTheAnalyticPathRefuses gives the quarter turn a
// section displacement the analytic revolve path does not charge: that path
// refuses, and MassProperties reads the body off its verified mesh instead,
// still enclosing the closed form.
func TestMeshMassReadsARevolveTheAnalyticPathRefuses(t *testing.T) {
	quarter := meshLadderRevolve(t, AngleExtent{A: units.Degrees(90), Dir: Along}, func(s *sketch.Sketch) {
		rect := s.CreateRectangle(0, 0, 10, 8)
		s.Fix(rect.A)
	})
	rp, ok := quarter.payload.(revolvePayload)
	require.True(t, ok)
	rp.sectionDelta = 1.0 / 1024
	displaced := &Body{
		doc: quarter.doc, origin: quarter.origin, lumps: quarter.lumps,
		volume: quarter.volume, area: quarter.area, centroid: quarter.centroid, bounds: quarter.bounds,
		solid: quarter.solid, kind: quarter.kind, payload: rp,
	}
	_, err := massmoment.RevolveProperties(t.Context(), massRevolveRecord(rp), displaced.centroid, meshLadderDensity)
	require.ErrorIs(t, err, ErrUnsupported, "premise: the analytic path refuses the displacement")

	got, err := displaced.MassProperties(t.Context(), meshLadderDensity)
	require.NoError(t, err)
	ladder, err := verifiedMeshMassProperties(t.Context(), displaced, meshLadderDensity)
	require.NoError(t, err)
	require.Equal(t, ladder, got)
	rho := new(big.Rat).SetFloat64(meshLadderDensity.Base())
	requireMeshReadingCovers(t, got.Mass, new(big.Rat).Mul(rho, new(big.Rat).Mul(meshLadderPi, big.NewRat(160, 1))))
}
