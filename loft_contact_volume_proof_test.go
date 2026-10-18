package decad

import (
	"context"
	"errors"
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

// The real boundary producer carries the source partition from its own call.
// This helper exposes the same live payloads for adversarial shell tests.
func loftVolumeRealCertificates(t *testing.T, first, second *Body) (
	prismPayload, loftPayload, loftOutputBoundaryProof, *proofbound.WorkBudget,
) {
	t.Helper()
	operation := proofbound.NewWorkBudget(t.Context())
	boundary, admitted, err := certifyLoftOutputBoundary(t.Context(), first, second, operation)
	require.NoError(t, err)
	require.True(t, admitted)
	if pp, ok := first.n.Payload.(prismPayload); ok {
		lp, ok := second.n.Payload.(loftPayload)
		require.True(t, ok)
		return pp, lp, boundary, operation
	}
	lp, ok := first.n.Payload.(loftPayload)
	require.True(t, ok)
	pp, ok := second.n.Payload.(prismPayload)
	require.True(t, ok)
	return pp, lp, boundary, operation
}

func TestLoftVolumeComposeRealVaryingTabBothOrders(t *testing.T) {
	for _, tc := range []struct {
		name         string
		lower, upper [4]float64
	}{
		{"right x", [4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}},
		{"upper y", [4]float64{4, 8, 6, 12}, [4]float64{4, 8, 6, 13}},
		{"left x", [4]float64{-2, 4, 2, 6}, [4]float64{-3, 4, 2, 6}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			prism, loft := loftPartitionRealBodies(t, tc.lower, tc.upper, 0, 4)
			for _, pair := range [][2]*Body{{prism, loft}, {loft, prism}} {
				pp, lp, boundary, operation := loftVolumeRealCertificates(t, pair[0], pair[1])
				proof, admitted, err := loftVolumeCompose(t.Context(), operation, pp, lp, boundary)
				require.NoError(t, err)
				require.True(t, admitted)
				require.Equal(t, [3]string{"0", "4", "5"}, [3]string{
					proof.boundary.source.capLevels[0].RatString(),
					proof.boundary.source.capLevels[1].RatString(),
					proof.boundary.source.capLevels[2].RatString(),
				})
				require.Equal(t, 0, proof.sourceVolume.Cmp(big.NewRat(520, 1)))
				require.True(t, proof.symDiffUpper >= 0 && proof.symDiffUpper < 520,
					"bound=%g", proof.symDiffUpper)
				require.True(t, proof.totalOutputVisits > proof.boundary.workVisits &&
					proof.totalOutputVisits < loftVolumeOutputSteps,
					"output visits=%d", proof.totalOutputVisits)
				require.LessOrEqual(t, proof.totalOutputVisits, uint64(loftVolumeOutputSteps))
			}
		})
	}
}

func loftVolumePolygonBody(t *testing.T, world *sketch.World, base, upper *sketch.Plane,
	coords [4][2]float64) *Body {
	t.Helper()
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{base, upper} {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		var points [4]*sketch.Point
		for j, xy := range coords {
			points[j] = s.CreatePoint(xy[0], xy[1])
		}
		for j := range points {
			s.CreateLine(points[j], points[(j+1)%len(points)])
		}
		s.Fix(points[0])
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	loft, err := Loft(t.Context(), New(), sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	return loft
}

func TestLoftVolumeSourceRefusesNonbinaryJunction(t *testing.T) {
	// The second right-wall crossing is (10, 32/5), which has no exact
	// binary64 representation and therefore has no admitted Sketch witness.
	parameter := new(big.Rat).Quo(big.NewRat(13-10, 1), big.NewRat(13-8, 1))
	junction := new(big.Rat).Sub(big.NewRat(7, 1), parameter)
	require.Equal(t, "3/5", parameter.RatString())
	require.Equal(t, "32/5", junction.RatString())
	_, exact := junction.Float64()
	require.False(t, exact)

	world := sketch.NewWorld()
	upper, err := world.CreateOffsetPlane(world.XY(), 4)
	require.NoError(t, err)
	loft := loftVolumePolygonBody(t, world, world.XY(), upper,
		[4][2]float64{{8, 4}, {12, 4}, {13, 7}, {8, 6}})
	prism := internalBoxBody(t, New(), 0, 0, 10, 10, 5)
	for _, tc := range []struct {
		name          string
		first, second *Body
	}{{loftOperandPrismFirst, prism, loft}, {loftOperandLoftFirst, loft, prism}} {
		t.Run(tc.name, func(t *testing.T) {
			_, admitted, err := certifyLoftSourcePartition(t.Context(), tc.first, tc.second,
				proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			require.False(t, admitted)
		})
	}
}

func TestLoftOutputBoundaryRefusesTinyTranslatedSeam(t *testing.T) {
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{Y: math.Ldexp(1, -52)}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	base, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	upper, err := world.CreateOffsetPlane(base, 4)
	require.NoError(t, err)
	loft := loftVolumePolygonBody(t, world, base, upper,
		[4][2]float64{{8, 4}, {12, 4}, {12, 7}, {8, 6}})
	s, err := world.CreateSketch(base)
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	prism, err := Extrude(t.Context(), New(), s, s.Profiles()[0],
		Distance{D: units.Millimeters(5), Dir: Along})
	require.NoError(t, err)
	wantWorldY := new(big.Rat).Add(big.NewRat(13, 2),
		new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 52)))
	heldY, exact := wantWorldY.Float64()
	require.False(t, exact)
	require.Equal(t, 6.5, heldY)
	for _, tc := range []struct {
		name          string
		first, second *Body
	}{{loftOperandPrismFirst, prism, loft}, {loftOperandLoftFirst, loft, prism}} {
		t.Run(tc.name, func(t *testing.T) {
			source, admitted, sourceErr := certifyLoftSourcePartition(t.Context(),
				tc.first, tc.second, proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, sourceErr)
			require.True(t, admitted)
			require.Positive(t, source.sourceVisits)
			for _, cap := range source.prepared {
				require.NotNil(t, cap.scene)
				require.Positive(t, len(cap.raw))
			}

			boundary, accepted, boundaryErr := certifyLoftOutputBoundary(t.Context(),
				tc.first, tc.second, proofbound.NewWorkBudget(t.Context()))
			require.ErrorIs(t, boundaryErr, ErrUnsupported)
			require.ErrorContains(t, boundaryErr, "private loft crossing vertex exceeds the exact-bit gate")
			require.False(t, accepted)
			require.Zero(t, boundary.workVisits)
			require.Zero(t, len(boundary.shell.vertices))
			require.Zero(t, len(boundary.shell.triangles))
			require.True(t, boundary.heldAudit == nil)
		})
	}
}

func TestLoftVolumeSourceRefusesRealTriangleFractionalSeam(t *testing.T) {
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{Y: math.Ldexp(1, -52)}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	base, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	upper, err := world.CreateOffsetPlane(base, 4)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{base, upper} {
		s, createErr := world.CreateSketch(plane)
		require.NoError(t, createErr)
		var points [3]*sketch.Point
		for j, xy := range [3][2]float64{{8, 4}, {12, 4}, {12, 9}} {
			points[j] = s.CreatePoint(xy[0], xy[1])
		}
		for j := range points {
			s.CreateLine(points[j], points[(j+1)%len(points)])
		}
		s.Fix(points[0])
		_, createErr = s.Solve(t.Context())
		require.NoError(t, createErr)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	loft, err := Loft(t.Context(), New(), sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	s, err := world.CreateSketch(base)
	require.NoError(t, err)
	var prismPoints [3]*sketch.Point
	for i, xy := range [3][2]float64{{0, 0}, {10, 0}, {10, 10}} {
		prismPoints[i] = s.CreatePoint(xy[0], xy[1])
	}
	for i := range prismPoints {
		s.CreateLine(prismPoints[i], prismPoints[(i+1)%len(prismPoints)])
	}
	s.Fix(prismPoints[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	prism, err := Extrude(t.Context(), New(), s, s.Profiles()[0],
		Distance{D: units.Millimeters(5), Dir: Along})
	require.NoError(t, err)
	for _, tc := range []struct {
		name          string
		first, second *Body
	}{{loftOperandPrismFirst, prism, loft}, {loftOperandLoftFirst, loft, prism}} {
		t.Run(tc.name, func(t *testing.T) {
			parent := proofbound.NewWorkBudget(t.Context())
			var attempts uint64
			operation := &proofbound.WorkBudget{
				StepFn: func() error {
					attempts++
					return parent.Step()
				},
				ErrFn: parent.Err,
			}
			source, admitted, sourceErr := certifyLoftSourcePartition(
				t.Context(), tc.first, tc.second, operation)
			require.ErrorIs(t, sourceErr, proofbound.ErrWallWorkBudget)
			require.False(t, admitted)
			// The source wall accepts 4,096 steps before refusing this attempt.
			require.Equal(t, uint64(4097), attempts)
			t.Logf("source visits=4096 refused attempt=%d", attempts)
			require.Zero(t, source.sourceVisits)
			require.Nil(t, source.walls)
			require.Nil(t, source.prepared[0].scene)
			require.Nil(t, source.prepared[1].scene)

			canceled, cancel := context.WithCancel(t.Context())
			cancel()
			partial, accepted, cancelErr := certifyLoftSourcePartition(canceled,
				tc.first, tc.second, proofbound.NewWorkBudget(canceled))
			require.ErrorIs(t, cancelErr, context.Canceled)
			require.False(t, accepted)
			require.Zero(t, partial.sourceVisits)
			require.Nil(t, partial.walls)
		})
	}
}

func TestLoftVolumeComposeRefusesUnprovedShells(t *testing.T) {
	prism, loft := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	pp, lp, boundary, operation := loftVolumeRealCertificates(t, prism, loft)
	for _, tc := range []struct {
		name   string
		change func(*loftOutputShell)
	}{
		{"missing cap", func(s *loftOutputShell) {
			for i, face := range s.faces {
				if face.kind == 0 {
					s.triangles = append(s.triangles[:i:i], s.triangles[i+1:]...)
					s.faces = append(s.faces[:i:i], s.faces[i+1:]...)
					s.triangleBounds = append(s.triangleBounds[:i:i], s.triangleBounds[i+1:]...)
					return
				}
			}
		}},
		{"missing side", func(s *loftOutputShell) {
			for i, face := range s.faces {
				if face.kind == 1 {
					s.triangles = append(s.triangles[:i:i], s.triangles[i+1:]...)
					s.faces = append(s.faces[:i:i], s.faces[i+1:]...)
					s.triangleBounds = append(s.triangleBounds[:i:i], s.triangleBounds[i+1:]...)
					return
				}
			}
		}},
		{"wrong cap owner", func(s *loftOutputShell) {
			for i, face := range s.faces {
				if face.kind == 0 && face.level == 1 {
					s.faces[i].owner = 0
					return
				}
			}
		}},
		{"reversed triangle", func(s *loftOutputShell) {
			s.triangles[0][0], s.triangles[0][1] = s.triangles[0][1], s.triangles[0][0]
		}},
		{"lost correspondence", func(s *loftOutputShell) {
			s.vertices[0].world, s.vertices[1].world = s.vertices[1].world, s.vertices[0].world
		}},
		{"nonfinite held coordinate", func(s *loftOutputShell) {
			s.vertices[0].held.X = math.NaN()
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			changed := boundary
			changed.shell.vertices = append([]loftOutputVertex(nil), boundary.shell.vertices...)
			changed.shell.triangles = append([][3]int(nil), boundary.shell.triangles...)
			changed.shell.faces = append([]loftOutputFace(nil), boundary.shell.faces...)
			changed.shell.triangleBounds = append([]float64(nil), boundary.shell.triangleBounds...)
			tc.change(&changed.shell)
			_, admitted, err := loftVolumeCompose(t.Context(),
				operation, pp, lp, changed)
			require.NoError(t, err)
			require.False(t, admitted)
		})
	}
	invalid := boundary
	invalid.workVisits = 0
	_, admitted, err := loftVolumeCompose(t.Context(), operation,
		pp, lp, invalid)
	require.NoError(t, err)
	require.False(t, admitted)

	wrongSource := boundary
	wrongSource.source.capLevels[1] = big.NewRat(3, 1)
	_, admitted, err = loftVolumeCompose(t.Context(), operation,
		pp, lp, wrongSource)
	require.NoError(t, err)
	require.False(t, admitted)
}

func TestLoftVolumeComposeBudgetAndCancellation(t *testing.T) {
	prism, loft := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	pp, lp, boundary, operation := loftVolumeRealCertificates(t, prism, loft)
	_, admitted, err := loftVolumeCompose(t.Context(), nil, pp, lp, boundary)
	require.NoError(t, err)
	require.False(t, admitted)

	nearlyFull := boundary
	nearlyFull.workVisits = loftVolumeOutputSteps - 1
	_, admitted, err = loftVolumeCompose(t.Context(), operation,
		pp, lp, nearlyFull)
	require.False(t, admitted)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)

	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	steps := 0
	operation = &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 10 {
				cancel()
			}
			return ctx.Err()
		},
		ErrFn: ctx.Err,
	}
	_, admitted, err = loftVolumeCompose(ctx, operation, pp, lp, boundary)
	require.False(t, admitted)
	require.True(t, errors.Is(err, context.Canceled))
	require.GreaterOrEqual(t, steps, 10)
}

func TestLoftVolumeComposeKeepsBoundaryOperationBudget(t *testing.T) {
	prism, loft := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	var used uint64
	ceiling := ^uint64(0)
	operation := &proofbound.WorkBudget{
		StepFn: func() error {
			if used >= ceiling {
				return proofbound.ErrWallWorkBudget
			}
			used++
			return nil
		},
		ErrFn: func() error {
			if used >= ceiling {
				return proofbound.ErrWallWorkBudget
			}
			return nil
		},
	}
	boundary, admitted, err := certifyLoftOutputBoundary(t.Context(), prism, loft, operation)
	require.NoError(t, err)
	require.True(t, admitted)
	require.Positive(t, boundary.workVisits)

	// Spend the final parent step after the real boundary has returned.
	ceiling = used + 1
	require.NoError(t, operation.Step())
	require.ErrorIs(t, operation.Err(), proofbound.ErrWallWorkBudget)
	pp, ok := prism.n.Payload.(prismPayload)
	require.True(t, ok)
	lp, ok := loft.n.Payload.(loftPayload)
	require.True(t, ok)
	_, admitted, err = loftVolumeCompose(t.Context(), operation, pp, lp, boundary)
	require.False(t, admitted)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	require.Equal(t, ceiling, used)
}

// A tetrahedron with its apex moved outward by 1/2 has exact symmetric
// difference 1/12. This small independent calculation tests the geometric
// hull cover without treating the difference of signed volumes as a general
// proof for arbitrary shells.
func TestLoftVolumeHullCoverIndependentTetra(t *testing.T) {
	base := [4][3]*big.Rat{
		{big.NewRat(0, 1), big.NewRat(0, 1), big.NewRat(0, 1)},
		{big.NewRat(1, 1), big.NewRat(0, 1), big.NewRat(0, 1)},
		{big.NewRat(0, 1), big.NewRat(1, 1), big.NewRat(0, 1)},
		{big.NewRat(0, 1), big.NewRat(0, 1), big.NewRat(1, 1)},
	}
	held := base
	held[3][2] = big.NewRat(3, 2)
	bound := new(big.Rat)
	for _, tri := range [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}} {
		var points [6][3]*big.Rat
		for i, index := range tri {
			points[i] = base[index]
			points[i+3] = held[index]
		}
		term, admitted, err := loftVolumeHullBound(proofbound.NewWallWorkBudget(65_536), points)
		require.NoError(t, err)
		require.True(t, admitted)
		require.True(t, term.Sign() >= 0)
		bound.Add(bound, term)
	}
	require.True(t, bound.Cmp(big.NewRat(1, 12)) >= 0, "bound=%s", bound.RatString())
}

func TestLoftVolumeHullZeroWidthAndMidWorkCancellation(t *testing.T) {
	triangle := [3][3]*big.Rat{
		{big.NewRat(0, 1), big.NewRat(0, 1), big.NewRat(0, 1)},
		{big.NewRat(1, 1), big.NewRat(0, 1), big.NewRat(0, 1)},
		{big.NewRat(0, 1), big.NewRat(1, 1), big.NewRat(0, 1)},
	}
	var points [6][3]*big.Rat
	for i := range triangle {
		points[i], points[i+3] = triangle[i], triangle[i]
	}
	volume, admitted, err := loftVolumeHullBound(proofbound.NewWallWorkBudget(65_536), points)
	require.NoError(t, err)
	require.True(t, admitted)
	require.Zero(t, volume.Sign())

	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	steps := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 25 {
				cancel()
			}
			return ctx.Err()
		},
		ErrFn: ctx.Err,
	}
	_, admitted, err = loftVolumeHullBound(budget, points)
	require.False(t, admitted)
	require.True(t, errors.Is(err, context.Canceled))
	require.GreaterOrEqual(t, steps, 25)
}

func TestLoftVolumeRationalCeiling(t *testing.T) {
	tooLarge := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), loftVolumeComposerRatBits))
	require.False(t, loftVolumeRatFits(tooLarge))
	_, rounded := loftVolumeRoundUp(tooLarge)
	require.False(t, rounded)
	require.False(t, loftVolumeSumFits(big.NewRat(1, 1), tooLarge))
	require.False(t, loftVolumeCompareFits(big.NewRat(1, 1), tooLarge))
	require.True(t, loftVolumeRatFits(big.NewRat(1, 3)))
	require.True(t, loftVolumeSumFits(big.NewRat(1, 3), big.NewRat(1, 6)))
	require.True(t, loftVolumeCompareFits(big.NewRat(1, 3), big.NewRat(1, 6)))

	heldSevenTenths := proofarith.FloatRat(0.7)
	nearNumerator := new(big.Int).Lsh(heldSevenTenths.Num(),
		uint(8191-(heldSevenTenths.Denom().BitLen()-1)))
	nearNumerator.Add(nearNumerator, big.NewInt(1))
	nearFloat := new(big.Rat).SetFrac(nearNumerator,
		new(big.Int).Lsh(big.NewInt(1), 8191))
	require.True(t, loftVolumeRatFits(nearFloat))
	_, rounded = loftVolumeRoundUp(nearFloat)
	require.False(t, rounded)
	deepDenominator := new(big.Rat).SetFrac(big.NewInt(1),
		new(big.Int).Mul(big.NewInt(3), new(big.Int).Lsh(big.NewInt(1), 8138)))
	require.True(t, loftVolumeRatFits(deepDenominator))
	_, rounded = loftVolumeRoundUp(deepDenominator)
	require.False(t, rounded)

	for _, value := range []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 3), proofarith.FloatRat(0.7)} {
		got, ok := loftVolumeRoundUp(value)
		require.True(t, ok)
		require.Equal(t, math.Float64bits(proofbound.RatFloatUp(value)), math.Float64bits(got))
	}
	extremeDyadic := new(big.Rat).SetFrac(big.NewInt(1),
		new(big.Int).Lsh(big.NewInt(1), 8191))
	got, ok := loftVolumeRoundUp(extremeDyadic)
	require.True(t, ok)
	require.Equal(t, math.SmallestNonzeroFloat64, got)
}
