package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestLoftContactOutputBoundaryRealVaryingTab(t *testing.T) {
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
				proof, admitted, err := certifyLoftOutputBoundary(t.Context(), pair[0], pair[1],
					proofbound.NewWorkBudget(t.Context()))
				require.NoError(t, err)
				require.True(t, admitted)
				require.NotEmpty(t, proof.shell.triangles)
				require.NotEmpty(t, proof.shell.vertices)
				require.Positive(t, proof.workVisits)
				require.LessOrEqual(t, proof.workVisits, uint64(65_536))
				require.LessOrEqual(t, len(proof.shell.triangles), 512)
				require.LessOrEqual(t, len(proof.shell.vertices), 512)
				if tc.name == "right x" {
					require.Equal(t, [3]string{"104", "6", "100"},
						loftOutputTestCapAreas(proof.shell))
				}
				t.Logf("work visits=%d facets=%d vertices=%d bound=%g",
					proof.workVisits, len(proof.shell.triangles),
					len(proof.shell.vertices), proof.shell.maxBound)
			}
		})
	}
}

func loftOutputTestCapAreas(shell loftOutputShell) [3]string {
	var areas [3]*big.Rat
	for i := range areas {
		areas[i] = new(big.Rat)
	}
	for i, tri := range shell.triangles {
		face := shell.faces[i]
		if face.kind != 0 {
			continue
		}
		a, b, c := shell.vertices[tri[0]].point,
			shell.vertices[tri[1]].point, shell.vertices[tri[2]].point
		double := loftPartitionCross(loftPartitionPoint{a.u, a.v},
			loftPartitionPoint{b.u, b.v}, loftPartitionPoint{c.u, c.v})
		areas[face.level].Add(areas[face.level], new(big.Rat).Abs(double))
	}
	var result [3]string
	for i := range result {
		result[i] = areas[i].Quo(areas[i], big.NewRat(2, 1)).RatString()
	}
	return result
}

func TestLoftContactOutputBoundarySourceRefusals(t *testing.T) {
	prism, loft := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	source, admitted, err := certifyLoftSourcePartition(t.Context(), prism, loft,
		proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, admitted)
	proof, admitted, err := certifyLoftOutputBoundary(t.Context(), prism, loft,
		proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, admitted)
	for _, tc := range []struct {
		name   string
		change func(*loftOutputShell)
	}{
		{"missing cap", func(s *loftOutputShell) {
			for i, face := range s.faces {
				if face.kind == 0 && face.level == 1 {
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
	} {
		t.Run(tc.name, func(t *testing.T) {
			shell := proof.shell
			shell.triangles = append([][3]int(nil), proof.shell.triangles...)
			shell.faces = append([]loftOutputFace(nil), proof.shell.faces...)
			shell.triangleBounds = append([]float64(nil), proof.shell.triangleBounds...)
			tc.change(&shell)
			resolved, err := loftOutputAuditSource(proofbound.NewWorkBudget(t.Context()), shell, source)
			require.NoError(t, err)
			require.False(t, resolved)
		})
	}
	polygon := []loftPartitionPoint{
		{big.NewRat(0, 1), big.NewRat(0, 1)},
		{big.NewRat(2, 1), big.NewRat(0, 1)},
		{big.NewRat(2, 1), big.NewRat(2, 1)},
		{big.NewRat(0, 1), big.NewRat(2, 1)},
	}
	resolved, err := loftOutputCertifyCapTiling(proofbound.NewWorkBudget(t.Context()), polygon,
		[][3]int{{0, 1, 2}, {0, 1, 2}})
	require.NoError(t, err)
	require.False(t, resolved)
	resolved, err = loftOutputCertifyCapTiling(proofbound.NewWorkBudget(t.Context()), polygon,
		[][3]int{{0, 1, 2}})
	require.NoError(t, err)
	require.False(t, resolved)
	for _, triangle := range [][3]int{{-1, 1, 2}, {0, 1, len(polygon)}} {
		resolved, err = loftOutputCertifyCapTiling(proofbound.NewWorkBudget(t.Context()), polygon,
			[][3]int{triangle})
		require.NoError(t, err)
		require.False(t, resolved)
	}
}

func TestLoftContactOutputBoundaryLimitsAndCancellation(t *testing.T) {
	prism, loft := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	pp, lp := prism.n.Payload.(prismPayload), loft.n.Payload.(loftPayload)
	source, admitted, err := certifyLoftSourcePartition(t.Context(), prism, loft,
		proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, admitted)
	readLines := func(loop sectionrecord.LoopRecord) []sectionrecord.LineSeg {
		lines, ok, err := loftPartitionLines(proofbound.NewWorkBudget(t.Context()), loop)
		require.NoError(t, err)
		require.True(t, ok)
		return lines
	}
	_, err = loftOutputCap(t.Context(), proofbound.NewWallWorkBudgetWithOperation(1,
		proofbound.NewWorkBudget(t.Context())), pp, readLines(pp.Profile.Outer),
		readLines(lp.Profile0.Outer), readLines(lp.Profile1.Outer), source,
		new(loftOutputShell), 0)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	proof, admitted, err := certifyLoftOutputBoundary(t.Context(), prism, loft,
		proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, admitted)
	shell := proof.shell
	shell.triangles = make([][3]int, 513)
	resolved, err := loftOutputAuditSource(proofbound.NewWorkBudget(t.Context()), shell, source)
	require.NoError(t, err)
	require.False(t, resolved)
	full := loftOutputNewShell()
	full.vertices = make([]loftOutputVertex, 512)
	_, admitted = full.vertex(pp.Frame, loftOutputPoint{
		u: big.NewRat(1, 1), v: big.NewRat(1, 1), z: big.NewRat(0, 1)},
		loftOutputFace{})
	require.False(t, admitted)

	count := 0
	probe := &proofbound.WorkBudget{StepFn: func() error { count++; return nil }, ErrFn: func() error { return nil }}
	_, admitted, err = certifyLoftSourcePartition(t.Context(), prism, loft, probe)
	require.NoError(t, err)
	require.True(t, admitted)
	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	steps := 0
	deadline := count + 20
	operation := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == deadline {
				cancel()
			}
			return ctx.Err()
		},
		ErrFn: ctx.Err,
	}
	_, admitted, err = certifyLoftOutputBoundary(ctx, prism, loft, operation)
	require.False(t, admitted)
	require.True(t, errors.Is(err, context.Canceled))
	t.Logf("source steps=%d, canceled after %d shared operation steps", count, steps)
}

func TestLoftContactOutputBoundaryMiddleRegionContact(t *testing.T) {
	prism, loft := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	proof, admitted, err := certifyLoftOutputBoundary(t.Context(), prism, loft,
		proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, admitted)
	for _, tc := range []struct {
		name string
		xy   [3][3]int64
		want bool
	}{
		{"plane crossing outside cap", [3][3]int64{{1, 1, 3}, {2, 1, 5}, {1, 2, 4}}, true},
		{"improper crossing inside cap", [3][3]int64{{11, 4, 3}, {11, 6, 5}, {12, 5, 4}}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			shell := proof.shell
			shell.vertices = append([]loftOutputVertex(nil), proof.shell.vertices...)
			shell.triangles = append([][3]int(nil), proof.shell.triangles...)
			shell.faces = append([]loftOutputFace(nil), proof.shell.faces...)
			start := len(shell.vertices)
			for _, p := range tc.xy {
				shell.vertices = append(shell.vertices, loftOutputVertex{point: loftOutputPoint{
					u: big.NewRat(p[0], 1), v: big.NewRat(p[1], 1), z: big.NewRat(p[2], 1)}})
			}
			shell.triangles = append(shell.triangles, [3]int{start, start + 1, start + 2})
			shell.faces = append(shell.faces, loftOutputFace{kind: 1, owner: 0})
			directed := make(map[[2]int]int)
			for fi, tri := range shell.triangles {
				for k := range 3 {
					directed[[2]int{tri[k], tri[(k+1)%3]}] = fi
				}
			}
			resolved, err := loftOutputMiddleContact(proofbound.NewWorkBudget(t.Context()),
				shell, big.NewRat(4, 1), directed)
			require.NoError(t, err)
			require.Equal(t, tc.want, resolved)
		})
	}
}

func TestLoftContactOutputBoundaryPlanarityAndVertexRefusals(t *testing.T) {
	low := sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 0, V: 0},
		End: sectionrecord.Point2{U: 1, V: 0}, TStart: 0, TEnd: 1}
	high := sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 0, V: 0},
		End: sectionrecord.Point2{U: 1, V: 1}, TStart: 0, TEnd: 1}
	require.False(t, loftOutputPlanarWall(low, high, big.NewRat(4, 1)))
	prism, _ := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	frame := prism.n.Payload.(prismPayload).Frame
	shell := loftOutputNewShell()
	face := loftOutputFace{kind: 0}
	_, admitted := shell.vertex(frame, loftOutputPoint{
		u: big.NewRat(1, 1), v: big.NewRat(1, 1), z: big.NewRat(0, 1)}, face)
	require.True(t, admitted)
	u := new(big.Rat).Add(big.NewRat(1, 1),
		new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 1000)))
	_, admitted = shell.vertex(frame, loftOutputPoint{
		u: u, v: big.NewRat(1, 1), z: big.NewRat(0, 1)}, face)
	require.False(t, admitted)
	_, _, bound, admitted := loftOutputWorld(frame, loftOutputPoint{
		u: big.NewRat(1, 3), v: big.NewRat(0, 1), z: big.NewRat(0, 1)})
	require.True(t, admitted)
	require.Positive(t, bound)
}

func TestLoftOutputWorldRefusesOversizedReplayArithmetic(t *testing.T) {
	prism, _ := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	frame := prism.n.Payload.(prismPayload).Frame
	largeDenominator := new(big.Int).Lsh(big.NewInt(1), 8000)
	world, _, _, admitted := loftOutputWorld(frame, loftOutputPoint{
		u: new(big.Rat).SetFrac(big.NewInt(1), largeDenominator),
		v: big.NewRat(0, 1), z: big.NewRat(0, 1),
	})
	_ = world
	require.False(t, admitted)
	// Float64 aligns the numerator before dividing. This input fits the
	// rational operands but would allocate 8,195 bits during alignment.
	alignmentDenominator := new(big.Int).Lsh(big.NewInt(1), 8140)
	world, _, _, admitted = loftOutputWorld(frame, loftOutputPoint{
		u: new(big.Rat).SetFrac(big.NewInt(1), alignmentDenominator),
		v: big.NewRat(0, 1), z: big.NewRat(0, 1),
	})
	_ = world
	require.False(t, admitted)

	largeNumerator := new(big.Int).Lsh(big.NewInt(1), 8192)
	shell := loftOutputNewShell()
	_, admitted = shell.vertex(frame, loftOutputPoint{
		u: new(big.Rat).SetInt(largeNumerator),
		v: big.NewRat(0, 1), z: big.NewRat(0, 1),
	}, loftOutputFace{kind: 0})
	require.False(t, admitted)
}

func TestLoftReplayFloat64RefusesOversizedAlignment(t *testing.T) {
	denominator := new(big.Int).Lsh(big.NewInt(1), 8140)
	arithmetic := &loftVolumeArithmetic{ok: true}
	_, exact := arithmetic.float64(new(big.Rat).SetFrac(big.NewInt(1), denominator))
	require.False(t, exact)
	require.False(t, arithmetic.ok)

	// The last permitted numerator alignment is exactly 8,192 bits.
	denominator = new(big.Int).Lsh(big.NewInt(1), 8137)
	arithmetic = &loftVolumeArithmetic{ok: true}
	_, _ = arithmetic.float64(new(big.Rat).SetFrac(big.NewInt(1), denominator))
	require.True(t, arithmetic.ok)
}

func TestLoftOutputCrossZeroRefusesOversizedReplayArithmetic(t *testing.T) {
	denominator := new(big.Int).Lsh(big.NewInt(1), 8000)
	tiny := new(big.Rat).SetFrac(big.NewInt(1), denominator)
	zero := big.NewRat(0, 1)
	points := [3]loftOutputPoint{
		{u: zero, v: zero, z: zero},
		{u: tiny, v: zero, z: zero},
		{u: zero, v: tiny, z: zero},
	}
	// The exact triangle has nonzero area; the unsafe cross product refuses.
	require.True(t, loftOutputCrossZero(points))
}

func TestLoftOutputOrientationRefusesOversizedReplayArithmetic(t *testing.T) {
	zero, one := big.NewRat(0, 1), big.NewRat(1, 1)
	large := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 8192))
	low := sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 0, V: 0},
		End: sectionrecord.Point2{U: 1, V: 0}}
	high := low
	require.False(t, loftOutputPlanarWall(low, high, large))
	require.False(t, loftOutputNormalsAgree(
		loftOutputVertex{world: [3]*big.Rat{large, zero, zero}},
		loftOutputVertex{world: [3]*big.Rat{one, zero, zero}},
		loftOutputVertex{world: [3]*big.Rat{zero, one, zero}},
	))
}

func TestLoftOutputTileCapRefusesOversizedReturnedPoint(t *testing.T) {
	prism, _ := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	pp := prism.n.Payload.(prismPayload)
	zero, one := big.NewRat(0, 1), big.NewRat(1, 1)
	large := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 8192))
	polygon := []loftPartitionPoint{
		{u: zero, v: zero}, {u: large, v: zero}, {u: zero, v: one},
	}
	shell := loftOutputNewShell()
	admitted, err := loftOutputTileCap(t.Context(), proofbound.NewWorkBudget(t.Context()),
		pp, &shell, polygon, zero, loftOutputFace{kind: 0}, false)
	require.NoError(t, err)
	require.False(t, admitted)
	require.Empty(t, shell.triangles)
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 8140))
	polygon = []loftPartitionPoint{
		{u: zero, v: zero}, {u: tiny, v: zero}, {u: zero, v: one},
	}
	admitted, err = loftOutputTileCap(t.Context(), proofbound.NewWorkBudget(t.Context()),
		pp, &shell, polygon, zero, loftOutputFace{kind: 0}, false)
	require.NoError(t, err)
	require.False(t, admitted)
	require.Empty(t, shell.triangles)
}

func TestLoftOutputSideRefusesOversizedLineInterpolation(t *testing.T) {
	prism, _ := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	frame := prism.n.Payload.(prismPayload).Frame
	line := sectionrecord.LineSeg{
		Start: sectionrecord.Point2{U: 1, V: 0},
		End:   sectionrecord.Point2{U: math.Nextafter(1, 2), V: 0},
	}
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 8150))
	shell := loftOutputNewShell()
	admitted, err := loftOutputLineQuad(proofbound.NewWorkBudget(t.Context()), frame,
		&shell, line, line, loftOutputRat(0), tiny, loftOutputRat(0), tiny,
		loftOutputRat(0), loftOutputRat(1), loftOutputFace{kind: 1})
	require.NoError(t, err)
	require.False(t, admitted)
	require.Empty(t, shell.triangles)
}

func TestLoftOutputMiddleTraceRefusesOversizedHeightDifference(t *testing.T) {
	first := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 5000))
	second := new(big.Rat).SetFrac(big.NewInt(-1), new(big.Int).Lsh(big.NewInt(1), 5001))
	shell := loftOutputShell{vertices: []loftOutputVertex{
		{point: loftOutputPoint{u: loftOutputRat(0), v: loftOutputRat(0), z: first}},
		{point: loftOutputPoint{u: loftOutputRat(1), v: loftOutputRat(0), z: second}},
		{point: loftOutputPoint{u: loftOutputRat(0), v: loftOutputRat(1), z: first}},
	}}
	arithmetic := &loftVolumeArithmetic{ok: true}
	trace, _, err := loftOutputPlaneTrace(proofbound.NewWorkBudget(t.Context()), shell,
		[3]int{0, 1, 2}, loftOutputRat(0), arithmetic)
	require.NoError(t, err)
	require.False(t, arithmetic.ok)
	require.Empty(t, trace)
}

func TestLoftOutputCapRefusesOversizedAreaProduct(t *testing.T) {
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 5000))
	zero := loftOutputRat(0)
	polygon := []loftPartitionPoint{
		{u: zero, v: zero}, {u: tiny, v: zero}, {u: zero, v: tiny},
	}
	admitted, err := loftOutputCertifyCapTiling(proofbound.NewWorkBudget(t.Context()),
		polygon, [][3]int{{0, 1, 2}})
	require.NoError(t, err)
	require.False(t, admitted)
}

func loftOutputAdversarialPoint(u, v int64) loftPartitionPoint {
	return loftPartitionPoint{u: big.NewRat(u, 1), v: big.NewRat(v, 1)}
}

func TestLoftOutputCapConcaveEscapingDiagonal(t *testing.T) {
	p := loftOutputAdversarialPoint
	polygon := []loftPartitionPoint{p(0, 0), p(4, 0), p(4, 1), p(1, 1), p(1, 4), p(0, 4)}
	budget := proofbound.NewWorkBudget(t.Context())

	// Both endpoints lie on the boundary, but the diagonal crosses the missing corner.
	inside, err := loftOutputEdgeInside(budget, polygon[2], polygon[4], polygon)
	require.NoError(t, err)
	require.False(t, inside)

	tri := [3]loftPartitionPoint{polygon[1], polygon[2], polygon[4]}
	inside, err = loftOutputTriangleInside(budget, polygon, tri)
	require.NoError(t, err)
	require.False(t, inside)

	inside, err = loftOutputCertifyCapTiling(budget, polygon, [][3]int{{1, 2, 4}})
	require.NoError(t, err)
	require.False(t, inside)
}

func TestLoftOutputCapEqualAreaOverlapAndGap(t *testing.T) {
	p := loftOutputAdversarialPoint
	polygon := []loftPartitionPoint{p(0, 0), p(4, 0), p(4, 2), p(4, 4), p(0, 4), p(0, 2)}
	tris := [][3]int{{0, 1, 3}, {0, 2, 4}}
	budget := proofbound.NewWorkBudget(t.Context())

	// The two distinct triangles each cover half the polygon's area, but overlap
	// near the center and leave another region uncovered.
	require.Zero(t, new(big.Rat).Sub(
		new(big.Rat).Add(loftPartitionCross(polygon[0], polygon[1], polygon[3]),
			loftPartitionCross(polygon[0], polygon[2], polygon[4])),
		new(big.Rat).Mul(loftOutputPolygonArea(polygon), big.NewRat(2, 1))).Sign())
	for _, tri := range tris {
		inside, err := loftOutputTriangleInside(budget, polygon,
			[3]loftPartitionPoint{polygon[tri[0]], polygon[tri[1]], polygon[tri[2]]})
		require.NoError(t, err)
		require.True(t, inside)
	}
	valid, err := loftOutputCertifyCapTiling(budget, polygon, tris)
	require.NoError(t, err)
	require.False(t, valid)
}

func TestLoftOutputCapCollinearBoundarySplit(t *testing.T) {
	p := loftOutputAdversarialPoint
	polygon := []loftPartitionPoint{p(0, 0), p(2, 0), p(4, 0), p(4, 4), p(0, 4)}
	budget := proofbound.NewWorkBudget(t.Context())

	inside, err := loftOutputEdgeInside(budget, polygon[0], polygon[2], polygon)
	require.NoError(t, err)
	require.True(t, inside)

	// The first tiling matches both collinear boundary segments. The second
	// covers the same area but has an unsplit edge over the boundary vertex.
	valid, err := loftOutputCertifyCapTiling(budget, polygon,
		[][3]int{{4, 0, 1}, {4, 1, 2}, {4, 2, 3}})
	require.NoError(t, err)
	require.True(t, valid)
	valid, err = loftOutputCertifyCapTiling(budget, polygon,
		[][3]int{{0, 2, 3}, {0, 3, 4}})
	require.NoError(t, err)
	require.False(t, valid)
}

func TestLoftOutputCapPredicatesCancelInFlight(t *testing.T) {
	p := loftOutputAdversarialPoint
	polygon := []loftPartitionPoint{p(0, 0), p(4, 0), p(4, 4), p(0, 4)}
	for _, tc := range []struct {
		name string
		call func(*proofbound.WorkBudget) (bool, error)
	}{
		{"edge", func(b *proofbound.WorkBudget) (bool, error) {
			return loftOutputEdgeInside(b, polygon[0], polygon[2], polygon)
		}},
		{"triangle", func(b *proofbound.WorkBudget) (bool, error) {
			return loftOutputTriangleInside(b, polygon,
				[3]loftPartitionPoint{polygon[0], polygon[1], polygon[2]})
		}},
		{"tiling", func(b *proofbound.WorkBudget) (bool, error) {
			return loftOutputCertifyCapTiling(b, polygon, [][3]int{{0, 1, 2}, {0, 2, 3}})
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
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
			valid, err := tc.call(budget)
			require.ErrorIs(t, err, context.Canceled)
			require.False(t, valid)
			require.Equal(t, 3, steps)
		})
	}
}
