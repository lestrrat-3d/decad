package decad

import (
	"context"
	"errors"
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
