package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
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
					proof.boundary.source.CapLevels[0].RatString(),
					proof.boundary.source.CapLevels[1].RatString(),
					proof.boundary.source.CapLevels[2].RatString(),
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
			require.Positive(t, source.SourceVisits)
			for _, cap := range source.Prepared {
				require.NotNil(t, cap.Scene)
				require.Positive(t, len(cap.Raw))
			}

			boundary, accepted, boundaryErr := certifyLoftOutputBoundary(t.Context(),
				tc.first, tc.second, proofbound.NewWorkBudget(t.Context()))
			require.ErrorIs(t, boundaryErr, ErrUnsupported)
			require.ErrorContains(t, boundaryErr, "private loft crossing vertex exceeds the exact-bit gate")
			require.False(t, accepted)
			require.Zero(t, boundary.workVisits)
			require.Zero(t, len(boundary.shell.Vertices))
			require.Zero(t, len(boundary.shell.Triangles))
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
			require.Zero(t, source.SourceVisits)
			require.Nil(t, source.Walls)
			require.Nil(t, source.Prepared[0].Scene)
			require.Nil(t, source.Prepared[1].Scene)

			canceled, cancel := context.WithCancel(t.Context())
			cancel()
			partial, accepted, cancelErr := certifyLoftSourcePartition(canceled,
				tc.first, tc.second, proofbound.NewWorkBudget(canceled))
			require.ErrorIs(t, cancelErr, context.Canceled)
			require.False(t, accepted)
			require.Zero(t, partial.SourceVisits)
			require.Nil(t, partial.Walls)
		})
	}
}

func TestLoftVolumeComposeRefusesUnprovedShells(t *testing.T) {
	prism, loft := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	pp, lp, boundary, operation := loftVolumeRealCertificates(t, prism, loft)
	for _, tc := range []struct {
		name   string
		change func(*loftcontact.OutputShell)
	}{
		{"missing cap", func(s *loftcontact.OutputShell) {
			for i, face := range s.Faces {
				if face.Kind == 0 {
					s.Triangles = append(s.Triangles[:i:i], s.Triangles[i+1:]...)
					s.Faces = append(s.Faces[:i:i], s.Faces[i+1:]...)
					s.TriangleBounds = append(s.TriangleBounds[:i:i], s.TriangleBounds[i+1:]...)
					return
				}
			}
		}},
		{"missing side", func(s *loftcontact.OutputShell) {
			for i, face := range s.Faces {
				if face.Kind == 1 {
					s.Triangles = append(s.Triangles[:i:i], s.Triangles[i+1:]...)
					s.Faces = append(s.Faces[:i:i], s.Faces[i+1:]...)
					s.TriangleBounds = append(s.TriangleBounds[:i:i], s.TriangleBounds[i+1:]...)
					return
				}
			}
		}},
		{"wrong cap owner", func(s *loftcontact.OutputShell) {
			for i, face := range s.Faces {
				if face.Kind == 0 && face.Level == 1 {
					s.Faces[i].Owner = 0
					return
				}
			}
		}},
		{"reversed triangle", func(s *loftcontact.OutputShell) {
			s.Triangles[0][0], s.Triangles[0][1] = s.Triangles[0][1], s.Triangles[0][0]
		}},
		{"lost correspondence", func(s *loftcontact.OutputShell) {
			s.Vertices[0].World, s.Vertices[1].World = s.Vertices[1].World, s.Vertices[0].World
		}},
		{"nonfinite held coordinate", func(s *loftcontact.OutputShell) {
			s.Vertices[0].Held.X = math.NaN()
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			changed := boundary
			changed.shell.Vertices = append([]loftcontact.OutputVertex(nil), boundary.shell.Vertices...)
			changed.shell.Triangles = append([][3]int(nil), boundary.shell.Triangles...)
			changed.shell.Faces = append([]loftcontact.OutputFace(nil), boundary.shell.Faces...)
			changed.shell.TriangleBounds = append([]float64(nil), boundary.shell.TriangleBounds...)
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
	wrongSource.source.CapLevels[1] = big.NewRat(3, 1)
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
		term, admitted, err := loftcontact.VolumeHullBound(proofbound.NewWallWorkBudget(65_536), points)
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
	volume, admitted, err := loftcontact.VolumeHullBound(proofbound.NewWallWorkBudget(65_536), points)
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
	_, admitted, err = loftcontact.VolumeHullBound(budget, points)
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

func TestLoftVolumeSourceEqualityRealAndRefusals(t *testing.T) {
	for _, tc := range []struct {
		name         string
		lower, upper [4]float64
	}{
		{"right_x", [4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}},
		{"upper_y", [4]float64{4, 8, 6, 12}, [4]float64{4, 8, 6, 13}},
		{"left_x", [4]float64{-2, 4, 2, 6}, [4]float64{-3, 4, 2, 6}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			prism, loft := loftPartitionRealBodies(t, tc.lower, tc.upper, 0, 4)
			pp, lp := prism.n.Payload.(prismPayload), loft.n.Payload.(loftPayload)
			for order, pair := range [][2]*Body{{prism, loft}, {loft, prism}} {
				name := loftOperandPrismFirst
				if order == 1 {
					name = loftOperandLoftFirst
				}
				t.Run(name, func(t *testing.T) {
					operation := proofbound.NewWorkBudget(t.Context())
					source, admitted, err := certifyLoftSourcePartition(t.Context(), pair[0], pair[1], operation)
					require.NoError(t, err)
					require.True(t, admitted)
					proof, admitted, err := certifyLoftOutputBoundary(t.Context(), pair[0], pair[1], operation)
					require.NoError(t, err)
					require.True(t, admitted)
					const outputWorkLimit uint64 = 65_536
					require.Less(t, proof.workVisits, outputWorkLimit)
					steps := 0
					counted := &proofbound.WorkBudget{
						StepFn: func() error { steps++; return operation.Step() },
						ErrFn:  operation.Err,
					}
					valid, err := loftVolumeSourceEquality(t.Context(),
						proofbound.NewWallWorkBudgetWithOperation(outputWorkLimit-proof.workVisits, counted),
						pp, lp, source, proof.shell)
					require.NoError(t, err)
					require.True(t, valid)
					require.LessOrEqual(t, proof.workVisits+uint64(steps), outputWorkLimit)
					t.Logf("output work visits=%d source-equality steps=%d", proof.workVisits, steps)
					if tc.name == "right_x" && order == 0 {
						loftVolumeSourceAdversarialCases(t, pp, lp, source, proof.shell)
					}
				})
			}
		})
	}
}

func loftVolumeSourceAdversarialCases(t *testing.T, pp prismPayload, lp loftPayload,
	source loftcontact.SourcePartition, original loftcontact.OutputShell) {
	t.Helper()
	clone := func() loftcontact.OutputShell {
		s := original
		s.Vertices = append([]loftcontact.OutputVertex(nil), original.Vertices...)
		s.Triangles = append([][3]int(nil), original.Triangles...)
		s.Faces = append([]loftcontact.OutputFace(nil), original.Faces...)
		s.TriangleBounds = append([]float64(nil), original.TriangleBounds...)
		return s
	}
	remove := func(s *loftcontact.OutputShell, kind int) {
		for i, face := range s.Faces {
			if face.Kind != kind {
				continue
			}
			s.Triangles = append(s.Triangles[:i:i], s.Triangles[i+1:]...)
			s.Faces = append(s.Faces[:i:i], s.Faces[i+1:]...)
			s.TriangleBounds = append(s.TriangleBounds[:i:i], s.TriangleBounds[i+1:]...)
			return
		}
	}
	for _, tc := range []struct {
		name   string
		change func(*loftcontact.OutputShell)
	}{
		{"missing_cap", func(s *loftcontact.OutputShell) { remove(s, 0) }},
		{"missing_side", func(s *loftcontact.OutputShell) { remove(s, 1) }},
		{"doubled_patch", func(s *loftcontact.OutputShell) {
			s.Triangles = append(s.Triangles, s.Triangles[0])
			s.Faces = append(s.Faces, s.Faces[0])
			s.TriangleBounds = append(s.TriangleBounds, s.TriangleBounds[0])
		}},
		{"internal_owner", func(s *loftcontact.OutputShell) { s.Faces[0].Owner = 2 }},
		{"reversed_orientation", func(s *loftcontact.OutputShell) {
			s.Triangles[0][1], s.Triangles[0][2] = s.Triangles[0][2], s.Triangles[0][1]
		}},
		{"changed_exact_vertex", func(s *loftcontact.OutputShell) {
			s.Vertices[0].Point.U = new(big.Rat).Add(s.Vertices[0].Point.U, big.NewRat(1, 1))
		}},
		{"changed_world_vertex", func(s *loftcontact.OutputShell) {
			s.Vertices[0].World[0] = new(big.Rat).Add(s.Vertices[0].World[0], big.NewRat(1, 1))
		}},
		{"oversized_rational", func(s *loftcontact.OutputShell) {
			s.Vertices[0].World[0] = new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 8193))
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			shell := clone()
			tc.change(&shell)
			valid, err := loftVolumeSourceEquality(t.Context(),
				proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
				pp, lp, source, shell)
			require.NoError(t, err)
			require.False(t, valid)
		})
	}
	t.Run("wrong_partition_wall", func(t *testing.T) {
		bad := source
		bad.Walls = append([]int8(nil), source.Walls...)
		bad.Walls[0] = 2
		valid, err := loftVolumeSourceEquality(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			pp, lp, bad, original)
		require.NoError(t, err)
		require.False(t, valid)
	})
	t.Run("wrong_cap_level", func(t *testing.T) {
		bad := source
		bad.CapLevels[1] = big.NewRat(3, 1)
		valid, err := loftVolumeSourceEquality(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			pp, lp, bad, original)
		require.NoError(t, err)
		require.False(t, valid)
	})
	t.Run("oversized_cap_level", func(t *testing.T) {
		bad := source
		bad.CapLevels[1] = new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 8193))
		valid, err := loftVolumeSourceEquality(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			pp, lp, bad, original)
		require.NoError(t, err)
		require.False(t, valid)
	})
	t.Run("cap_comparison_product_limit", func(t *testing.T) {
		bad := source
		bad.CapLevels[1] = new(big.Rat).SetFrac(
			big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 8190))
		valid, err := loftVolumeSourceEquality(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			pp, lp, bad, original)
		require.NoError(t, err)
		require.False(t, valid)
	})
	t.Run("pairwise_comparison_bounds", func(t *testing.T) {
		comparison, bounded := loftVolumeSourceCmp(big.NewRat(3, 2), big.NewRat(6, 4))
		require.True(t, bounded)
		require.Zero(t, comparison)

		large := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 8190))
		tiny := new(big.Rat).SetFrac(
			big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 8190))
		require.True(t, loftVolumeSourceRatOK(large))
		require.True(t, loftVolumeSourceRatOK(tiny))
		_, bounded = loftVolumeSourceCmp(large, big.NewRat(1, 4))
		require.False(t, bounded)
		_, bounded = loftVolumeSourceCmp(big.NewRat(4, 1), tiny)
		require.False(t, bounded)
		_, bounded = loftVolumeSourceCmp(tiny, big.NewRat(4, 1))
		require.False(t, bounded)

		atLimit := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 8188))
		comparison, bounded = loftVolumeSourceCmp(atLimit, big.NewRat(1, 4))
		require.True(t, bounded)
		require.Positive(t, comparison)
		comparison, bounded = loftVolumeSourceCmp(big.NewRat(1, 4), atLimit)
		require.True(t, bounded)
		require.Negative(t, comparison)
		atDenominatorLimit := new(big.Rat).SetFrac(
			big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 8188))
		comparison, bounded = loftVolumeSourceCmp(big.NewRat(4, 1), atDenominatorLimit)
		require.True(t, bounded)
		require.Positive(t, comparison)
		_, bounded = loftVolumeSourceCmp(nil, big.NewRat(1, 1))
		require.False(t, bounded)
	})
	t.Run("wrong_cap_owner", func(t *testing.T) {
		bad := source
		bad.CapOwners[0][0]++
		valid, err := loftVolumeSourceEquality(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			pp, lp, bad, original)
		require.NoError(t, err)
		require.False(t, valid)
	})
	t.Run("lost_seam", func(t *testing.T) {
		bad := source
		bad.Seams[0] = loftcontact.LineSeam{}
		valid, err := loftVolumeSourceEquality(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(65_536, proofbound.NewWorkBudget(t.Context())),
			pp, lp, bad, original)
		require.NoError(t, err)
		require.False(t, valid)
	})
	t.Run("shared_work_limit", func(t *testing.T) {
		valid, err := loftVolumeSourceEquality(t.Context(),
			proofbound.NewWallWorkBudgetWithOperation(1, proofbound.NewWorkBudget(t.Context())),
			pp, lp, source, original)
		require.False(t, valid)
		require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	})
	t.Run("canceled_during_source_check", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		defer cancel()
		steps := 0
		operation := &proofbound.WorkBudget{
			StepFn: func() error {
				steps++
				if steps == 50 {
					cancel()
				}
				return ctx.Err()
			},
			ErrFn: ctx.Err,
		}
		valid, err := loftVolumeSourceEquality(ctx,
			proofbound.NewWallWorkBudgetWithOperation(65_536, operation),
			pp, lp, source, original)
		require.False(t, valid)
		require.True(t, errors.Is(err, context.Canceled))
		require.Equal(t, 50, steps)
	})
}

func TestLoftBoundedProfilesRealRightXPrismFirst(t *testing.T) {
	prism, loft := loftBounded520Operands(t)
	operation := proofbound.NewWorkBudget(t.Context())
	boundary, admitted, err := certifyLoftOutputBoundary(t.Context(), prism, loft, operation)
	require.NoError(t, err)
	require.True(t, admitted)
	checkLoftBoundedCapCertificates(t, boundary.source)
	pp := prism.n.Payload.(prismPayload)
	lp := loft.n.Payload.(loftPayload)
	proof, admitted, err := loftVolumeCompose(t.Context(), operation, pp, lp, boundary)
	require.NoError(t, err)
	require.True(t, admitted)
	require.Zero(t, proof.sourceVolume.Cmp(big.NewRat(520, 1)))
	require.Less(t, proof.totalOutputVisits, uint64(loftVolumeOutputSteps))
	t.Logf("totalOutputVisits=%d", proof.totalOutputVisits)
}

func TestLoftBoundedProfilesRealRightXLoftFirst(t *testing.T) {
	prism, loft := loftBounded520Operands(t)
	operation := proofbound.NewWorkBudget(t.Context())
	boundary, admitted, err := certifyLoftOutputBoundary(t.Context(), loft, prism, operation)
	require.NoError(t, err)
	require.True(t, admitted)
	checkLoftBoundedCapCertificates(t, boundary.source)
	pp := prism.n.Payload.(prismPayload)
	lp := loft.n.Payload.(loftPayload)
	proof, admitted, err := loftVolumeCompose(t.Context(), operation, pp, lp, boundary)
	require.NoError(t, err)
	require.True(t, admitted)
	require.Zero(t, proof.sourceVolume.Cmp(big.NewRat(520, 1)))
	require.Less(t, proof.totalOutputVisits, uint64(loftVolumeOutputSteps))
	t.Logf("totalOutputVisits=%d", proof.totalOutputVisits)
}

func loftBounded520Operands(t *testing.T) (*Body, *Body) {
	t.Helper()
	world := sketch.NewWorld()
	base := world.XY()
	upper, err := world.CreateOffsetPlane(base, 4)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, coords := range [2][4]float64{
		{8, 4, 12, 6}, {8, 4, 13, 6},
	} {
		plane := base
		if i == 1 {
			plane = upper
		}
		s, createErr := world.CreateSketch(plane)
		require.NoError(t, createErr)
		rect := s.CreateRectangle(coords[0], coords[1], coords[2], coords[3])
		s.Fix(rect.A)
		_, createErr = s.Solve(t.Context())
		require.NoError(t, createErr)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	d := New()
	loft, err := Loft(t.Context(), d, sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	prismSketch, err := world.CreateSketch(base)
	require.NoError(t, err)
	rect := prismSketch.CreateRectangle(0, 0, 10, 10)
	prismSketch.Fix(rect.A)
	_, err = prismSketch.Solve(t.Context())
	require.NoError(t, err)
	prism, err := Extrude(t.Context(), d, prismSketch, prismSketch.Profiles()[0],
		Distance{D: units.Millimeters(5), Dir: Along})
	require.NoError(t, err)
	return prism, loft
}

func checkLoftBoundedCapCertificates(t *testing.T, source loftcontact.SourcePartition) {
	t.Helper()
	require.Greater(t, source.SourceVisits, uint64(0))
	require.LessOrEqual(t, source.SourceVisits, uint64(4_096))
	require.True(t, source.Prepared[0].Scene != source.Prepared[1].Scene)
	for capIndex, cap := range source.Prepared {
		require.True(t, cap.Scene != nil)
		require.Equal(t, capIndex, cap.CapIndex)
		require.True(t, cap.Level == source.CapLevels[capIndex])
		require.Equal(t, 3, len(cap.Raw))
		for _, cell := range cap.Raw {
			require.True(t, cell != nil)
			require.True(t, cell.Sketch() == cap.Scene)
			require.False(t, cell.IsStale())
		}
	}
	t.Logf("sourceVisits=%d", source.SourceVisits)
}

func TestLoftBoundedProfilesGroundingCancelsInFlight(t *testing.T) {
	world := sketch.NewWorld()
	scene, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	for i := range 8 {
		scene.CreatePoint(float64(i), 0)
	}
	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	steps := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 3 {
				cancel()
			}
			return ctx.Err()
		},
		ErrFn: ctx.Err,
	}
	err = groundLoftPartitionScene(budget, scene)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 3, steps)
	for i, point := range scene.Points() {
		require.Equal(t, i < 2, point.IsFixed(), "point %d", i)
	}
}

func TestLoftContactTwoLineCapsRealTrapezoid524(t *testing.T) {
	world := sketch.NewWorld()
	upper, err := world.CreateOffsetPlane(world.XY(), 4)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for cap, corners := range [2][4][2]float64{
		{{8, 4}, {12, 4}, {13, 6}, {8, 6}},
		{{8, 4}, {13, 4}, {14, 6}, {8, 6}},
	} {
		plane := world.XY()
		if cap == 1 {
			plane = upper
		}
		s, createErr := world.CreateSketch(plane)
		require.NoError(t, createErr)
		var points [4]*sketch.Point
		for i, xy := range corners {
			points[i] = s.CreatePoint(xy[0], xy[1])
			s.Fix(points[i])
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
		_, solveErr := s.Solve(t.Context())
		require.NoError(t, solveErr)
		found := s.Profiles()
		require.Equal(t, 1, len(found))
		require.True(t, found[0].Valid)
		sections[cap], profiles[cap] = s, found[0]
	}

	doc := New()
	prism := internalBoxBody(t, doc, 0, 0, 10, 10, 5)
	loft, err := Loft(t.Context(), doc, sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	operation := proofbound.NewWorkBudget(t.Context())
	boundary, admitted, err := certifyLoftOutputBoundary(
		t.Context(), prism, loft, operation)
	require.NoError(t, err)
	require.True(t, admitted)
	require.Greater(t, len(boundary.shell.Vertices), 0)
	require.Greater(t, len(boundary.shell.Triangles), 0)

	source := boundary.source
	require.Greater(t, source.SourceVisits, uint64(0))
	require.Less(t, source.SourceVisits, uint64(4096))
	require.Equal(t, [3]string{"0", "4", "5"}, [3]string{
		source.CapLevels[0].RatString(), source.CapLevels[1].RatString(),
		source.CapLevels[2].RatString(),
	})
	require.Equal(t, [2][3]int{{2, 1, 1}, {0, 1, 0}}, source.CapOwners)
	require.Equal(t, [2][2]sectionrecord.Point2{
		{{U: 10, V: 4}, {U: 10, V: 6}},
		{{U: 10, V: 4}, {U: 10, V: 6}},
	}, source.Junction)
	for cap, prepared := range source.Prepared {
		require.NotNil(t, prepared.Scene)
		require.Equal(t, 8, len(prepared.Tags))
		require.Equal(t, 8, len(prepared.Scene.Entities()))
		require.Equal(t, 3, len(prepared.Raw))
		junctions := map[[2]float64]struct{}{}
		for _, cell := range prepared.Raw {
			require.True(t, prepared.Scene == cell.Sketch())
			require.False(t, cell.IsStale())
			require.True(t, cell.Valid)
			require.Zero(t, len(cell.Holes))
			for i, left := range cell.Outer {
				right := cell.Outer[(i+1)%len(cell.Outer)]
				a, okA := prepared.Tags[left.Entity]
				b, okB := prepared.Tags[right.Entity]
				if !okA || !okB || a.IsB == b.IsB {
					continue
				}
				point, certified := prepared.Scene.CertifyLineLineJunction(left, right)
				if certified {
					junctions[point] = struct{}{}
				}
			}
		}
		require.Equal(t, 2, len(junctions), "cap %d junctions", cap)
		_, hasLower := junctions[[2]float64{10, 4}]
		_, hasUpper := junctions[[2]float64{10, 6}]
		require.True(t, hasLower, "cap %d lower junction", cap)
		require.True(t, hasUpper, "cap %d upper junction", cap)
	}

	pp := prism.n.Payload.(prismPayload)
	lp := loft.n.Payload.(loftPayload)
	proof, admitted, err := loftVolumeCompose(t.Context(), operation, pp, lp, boundary)
	require.NoError(t, err)
	require.True(t, admitted)
	require.Zero(t, proof.sourceVolume.Cmp(big.NewRat(524, 1)))
	require.False(t, math.IsNaN(proof.symDiffUpper))
	require.False(t, math.IsInf(proof.symDiffUpper, 0))
	require.GreaterOrEqual(t, proof.symDiffUpper, float64(0))
	require.Less(t, proof.symDiffUpper, float64(524))
	require.Greater(t, boundary.workVisits, uint64(0))
	require.Greater(t, proof.totalOutputVisits, boundary.workVisits)
	require.LessOrEqual(t, proof.totalOutputVisits, uint64(65_536))
	t.Logf("source visits=%d output visits=%d facets=%d", source.SourceVisits,
		proof.totalOutputVisits, len(boundary.shell.Triangles))
}

func TestLoftContactTwoLineCapsNarrowTranslated524(t *testing.T) {
	world := sketch.NewWorld()
	epsilon := math.Ldexp(1, -52)
	originY := 1 + epsilon
	frame, err := r3.NewFrame(r3.Vec{Y: originY}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	base, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	upper, err := world.CreateOffsetPlane(base, 1)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for cap, corners := range [2][4][2]float64{
		{{119, 0}, {163 - 896*epsilon, 0}, {163 - 896*epsilon, 1}, {119, 1}},
		{{119, 0}, {165 - 1024*epsilon, 0}, {165 - 1024*epsilon, 1}, {119, 1}},
	} {
		plane := base
		if cap == 1 {
			plane = upper
		}
		s, createErr := world.CreateSketch(plane)
		require.NoError(t, createErr)
		var points [4]*sketch.Point
		for i, xy := range corners {
			points[i] = s.CreatePoint(xy[0], xy[1])
			s.Fix(points[i])
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
		_, solveErr := s.Solve(t.Context())
		require.NoError(t, solveErr)
		found := s.Profiles()
		require.Equal(t, 1, len(found))
		require.True(t, found[0].Valid)
		sections[cap], profiles[cap] = s, found[0]
	}
	doc := New()
	prismSketch, err := world.CreateSketch(base)
	require.NoError(t, err)
	rect := prismSketch.CreateRectangle(0, -epsilon, 120, 1+epsilon)
	prismSketch.Fix(rect.A)
	_, err = prismSketch.Solve(t.Context())
	require.NoError(t, err)
	prismProfiles := prismSketch.Profiles()
	require.Equal(t, 1, len(prismProfiles))
	require.True(t, prismProfiles[0].Valid)
	prism, err := Extrude(t.Context(), doc, prismSketch, prismProfiles[0],
		Distance{D: units.Millimeters(4), Dir: Along})
	require.NoError(t, err)
	loft, err := Loft(t.Context(), doc, sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	pp, lp := prism.n.Payload.(prismPayload), loft.n.Payload.(loftPayload)
	require.True(t, lp.AuthenticatedReconstruction)
	require.NotNil(t, lp.SourceIdentity)
	require.Equal(t, pp.Frame, lp.Frame0)
	wantWorldY := new(big.Rat).Add(big.NewRat(2, 1), big.NewRat(1, 1<<52))
	for _, tc := range []struct {
		name          string
		first, second *Body
	}{{loftOperandPrismFirst, prism, loft}, {loftOperandLoftFirst, loft, prism}} {
		t.Run(tc.name, func(t *testing.T) {
			operation := proofbound.NewWorkBudget(t.Context())
			boundary, admitted, boundaryErr := certifyLoftOutputBoundary(
				t.Context(), tc.first, tc.second, operation)
			require.NoError(t, boundaryErr, "real boundary")
			require.True(t, admitted, "real boundary")
			require.Less(t, boundary.source.SourceVisits, uint64(4096))
			t.Logf("source visits=%d boundary output visits=%d",
				boundary.source.SourceVisits, boundary.workVisits)
			proof, admitted, composeErr := loftVolumeCompose(t.Context(), operation, pp, lp, boundary)
			require.NoError(t, composeErr, "real composer")
			require.True(t, admitted, "real composer")
			require.Zero(t, proof.sourceVolume.Cmp(big.NewRat(524, 1)))
			displaced := false
			for _, vertex := range proof.boundary.shell.Vertices {
				if vertex.Point.U.Cmp(big.NewRat(120, 1)) != 0 ||
					vertex.Point.V.Cmp(big.NewRat(1, 1)) != 0 {
					continue
				}
				require.Equal(t, wantWorldY.RatString(), vertex.World[1].RatString())
				require.Equal(t, float64(2), vertex.Held.Y)
				require.Positive(t, vertex.Bound)
				displaced = true
			}
			require.True(t, displaced)
			require.False(t, math.IsNaN(proof.symDiffUpper))
			require.False(t, math.IsInf(proof.symDiffUpper, 0))
			require.Greater(t, proof.symDiffUpper, float64(0))
			require.Less(t, proof.symDiffUpper, float64(524))
			require.LessOrEqual(t, proof.totalOutputVisits, uint64(loftVolumeOutputSteps))
			t.Logf("source visits=%d output visits=%d facets=%d bound=%g",
				boundary.source.SourceVisits, proof.totalOutputVisits,
				len(boundary.shell.Triangles), proof.symDiffUpper)
		})
	}
}

func TestLoftVolumeComposeRealDisplacedNonjunction(t *testing.T) {
	world := sketch.NewWorld()
	epsilon := math.Ldexp(1, -52)
	epsilonRat := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 52))
	seamWorld := new(big.Rat).Add(big.NewRat(1, 1), epsilonRat)
	seamHeld, exact := seamWorld.Float64()
	require.True(t, exact)
	require.Equal(t, 1+epsilon, seamHeld)
	wantDisplaced := new(big.Rat).Add(big.NewRat(5, 2), epsilonRat)
	displacedHeld, exact := wantDisplaced.Float64()
	require.False(t, exact)
	require.Equal(t, float64(2.5), displacedHeld)
	frame, err := r3.NewFrame(r3.Vec{X: 1 + epsilon}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	base, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	upper, err := world.CreateOffsetPlane(base, 4)
	require.NoError(t, err)

	doc := New()
	prismSketch, err := world.CreateSketch(base)
	require.NoError(t, err)
	prismRect := prismSketch.CreateRectangle(-9, 0, 0, 10)
	prismSketch.Fix(prismRect.A)
	_, err = prismSketch.Solve(t.Context())
	require.NoError(t, err)
	prism, err := Extrude(t.Context(), doc, prismSketch, prismSketch.Profiles()[0],
		Distance{D: units.Millimeters(5), Dir: Along})
	require.NoError(t, err)

	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{base, upper} {
		s, createErr := world.CreateSketch(plane)
		require.NoError(t, createErr)
		right := 1.0
		if i == 1 {
			right = 1.5
		}
		rect := s.CreateRectangle(-0.5, 4, right, 6)
		s.Fix(rect.A)
		_, createErr = s.Solve(t.Context())
		require.NoError(t, createErr)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	loft, err := Loft(t.Context(), doc, sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)

	pp, lp := prism.n.Payload.(prismPayload), loft.n.Payload.(loftPayload)
	for _, tc := range []struct {
		name          string
		first, second *Body
	}{
		{loftOperandPrismFirst, prism, loft},
		{loftOperandLoftFirst, loft, prism},
	} {
		t.Run(tc.name, func(t *testing.T) {
			operation := proofbound.NewWorkBudget(t.Context())
			boundary, admitted, err := certifyLoftOutputBoundary(
				t.Context(), tc.first, tc.second, operation)
			require.NoError(t, err, "boundary")
			require.True(t, admitted, "boundary")
			proof, admitted, err := loftVolumeCompose(t.Context(), operation, pp, lp, boundary)
			require.NoError(t, err, "composer")
			require.True(t, admitted, "composer")
			require.Zero(t, proof.sourceVolume.Cmp(big.NewRat(460, 1)))
			displaced := false
			for _, vertex := range boundary.shell.Vertices {
				if vertex.Point.U.Cmp(big.NewRat(3, 2)) == 0 &&
					vertex.Point.V.Cmp(big.NewRat(6, 1)) == 0 {
					require.Equal(t, wantDisplaced.RatString(), vertex.World[0].RatString())
					require.Equal(t, displacedHeld, vertex.Held.X)
					require.Positive(t, vertex.Bound)
					displaced = true
				}
			}
			require.True(t, displaced, "nonjunction displacement")
			require.Greater(t, proof.symDiffUpper, float64(0))
			require.Less(t, proof.symDiffUpper, float64(460))
			require.Positive(t, boundary.source.SourceVisits)
			require.LessOrEqual(t, boundary.source.SourceVisits, uint64(4096))
			require.LessOrEqual(t, proof.totalOutputVisits, uint64(loftVolumeOutputSteps))
			require.LessOrEqual(t, len(boundary.shell.Vertices), 512)
			require.LessOrEqual(t, len(boundary.shell.Triangles), 512)
			t.Logf("source visits=%d output visits=%d facets=%d bound=%g",
				boundary.source.SourceVisits, proof.totalOutputVisits,
				len(boundary.shell.Triangles), proof.symDiffUpper)
		})
	}

	t.Run("charged refusal", func(t *testing.T) {
		operation := proofbound.NewWorkBudget(t.Context())
		boundary, admitted, err := certifyLoftOutputBoundary(t.Context(), prism, loft, operation)
		require.NoError(t, err)
		require.True(t, admitted)
		limited := proofbound.NewWallWorkBudgetWithOperation(16, operation)
		proof, admitted, err := loftVolumeCompose(t.Context(), limited, pp, lp, boundary)
		require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
		require.False(t, admitted)
		require.Zero(t, proof.totalOutputVisits)
	})

	t.Run("inflight cancellation", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		defer cancel()
		parent := proofbound.NewWorkBudget(ctx)
		composing := false
		var composeVisits uint64
		operation := &proofbound.WorkBudget{
			StepFn: func() error {
				if composing {
					composeVisits++
					if composeVisits == 32 {
						cancel()
					}
				}
				return parent.Step()
			},
			ErrFn: parent.Err,
		}
		boundary, admitted, err := certifyLoftOutputBoundary(ctx, prism, loft, operation)
		require.NoError(t, err)
		require.True(t, admitted)
		composing = true
		proof, admitted, err := loftVolumeCompose(ctx, operation, pp, lp, boundary)
		require.ErrorIs(t, err, context.Canceled)
		require.False(t, admitted)
		require.Zero(t, proof.totalOutputVisits)
		require.Equal(t, uint64(32), composeVisits)
	})
}
