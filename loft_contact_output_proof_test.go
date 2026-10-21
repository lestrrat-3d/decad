package decad

import (
	"context"
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftdirect"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
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
				proof, admitted, err := loftdirect.CertifyLoftOutputBoundary(t.Context(), bodyNode(pair[0]), bodyNode(pair[1]),
					proofbound.NewWorkBudget(t.Context()))
				require.NoError(t, err)
				require.True(t, admitted)
				require.NotEmpty(t, proof.Shell.Triangles)
				require.NotEmpty(t, proof.Shell.Vertices)
				require.Positive(t, proof.WorkVisits)
				require.LessOrEqual(t, proof.WorkVisits, uint64(65_536))
				require.LessOrEqual(t, len(proof.Shell.Triangles), 512)
				require.LessOrEqual(t, len(proof.Shell.Vertices), 512)
				if tc.name == "right x" {
					require.Equal(t, [3]string{"104", "6", "100"},
						loftOutputTestCapAreas(proof.Shell))
				}
				t.Logf("work visits=%d facets=%d vertices=%d bound=%g",
					proof.WorkVisits, len(proof.Shell.Triangles),
					len(proof.Shell.Vertices), proof.Shell.MaxBound)
			}
		})
	}
}

func loftOutputTestCapAreas(shell loftcontact.OutputShell) [3]string {
	var areas [3]*big.Rat
	for i := range areas {
		areas[i] = new(big.Rat)
	}
	for i, tri := range shell.Triangles {
		face := shell.Faces[i]
		if face.Kind != 0 {
			continue
		}
		a, b, c := shell.Vertices[tri[0]].Point,
			shell.Vertices[tri[1]].Point, shell.Vertices[tri[2]].Point
		checked := &loftcontact.Arithmetic{OK: true}
		double := checked.PointCross(loftcontact.Point{U: a.U, V: a.V},
			loftcontact.Point{U: b.U, V: b.V}, loftcontact.Point{U: c.U, V: c.V})
		areas[face.Level].Add(areas[face.Level], new(big.Rat).Abs(double))
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
	source, admitted, err := loftdirect.CertifyLoftSourcePartition(t.Context(), bodyNode(prism), bodyNode(loft),
		proofbound.NewWorkBudget(t.Context()), false)
	require.NoError(t, err)
	require.True(t, admitted)
	proof, admitted, err := loftdirect.CertifyLoftOutputBoundary(t.Context(), bodyNode(prism), bodyNode(loft),
		proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, admitted)
	for _, tc := range []struct {
		name   string
		change func(*loftcontact.OutputShell)
	}{
		{"missing cap", func(s *loftcontact.OutputShell) {
			for i, face := range s.Faces {
				if face.Kind == 0 && face.Level == 1 {
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
	} {
		t.Run(tc.name, func(t *testing.T) {
			shell := proof.Shell
			shell.Triangles = append([][3]int(nil), proof.Shell.Triangles...)
			shell.Faces = append([]loftcontact.OutputFace(nil), proof.Shell.Faces...)
			shell.TriangleBounds = append([]float64(nil), proof.Shell.TriangleBounds...)
			tc.change(&shell)
			resolved, err := loftcontact.AuditOutputSource(proofbound.NewWorkBudget(t.Context()), shell, source)
			require.NoError(t, err)
			require.False(t, resolved)
		})
	}
	polygon := []loftcontact.Point{
		{U: big.NewRat(0, 1), V: big.NewRat(0, 1)},
		{U: big.NewRat(2, 1), V: big.NewRat(0, 1)},
		{U: big.NewRat(2, 1), V: big.NewRat(2, 1)},
		{U: big.NewRat(0, 1), V: big.NewRat(2, 1)},
	}
	resolved, err := loftcontact.CertifyCapTiling(proofbound.NewWorkBudget(t.Context()), polygon,
		[][3]int{{0, 1, 2}, {0, 1, 2}})
	require.NoError(t, err)
	require.False(t, resolved)
	resolved, err = loftcontact.CertifyCapTiling(proofbound.NewWorkBudget(t.Context()), polygon,
		[][3]int{{0, 1, 2}})
	require.NoError(t, err)
	require.False(t, resolved)
	for _, triangle := range [][3]int{{-1, 1, 2}, {0, 1, len(polygon)}} {
		resolved, err = loftcontact.CertifyCapTiling(proofbound.NewWorkBudget(t.Context()), polygon,
			[][3]int{triangle})
		require.NoError(t, err)
		require.False(t, resolved)
	}
}

func TestLoftContactOutputBoundaryLimitsAndCancellation(t *testing.T) {
	prism, loft := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	pp, lp := prism.n.Payload.(prismPayload), loft.n.Payload.(loftPayload)
	source, admitted, err := loftdirect.CertifyLoftSourcePartition(t.Context(), bodyNode(prism), bodyNode(loft),
		proofbound.NewWorkBudget(t.Context()), false)
	require.NoError(t, err)
	require.True(t, admitted)
	readLines := func(loop sectionrecord.LoopRecord) []sectionrecord.LineSeg {
		lines, ok, err := loftcontact.PartitionLines(proofbound.NewWorkBudget(t.Context()), loop)
		require.NoError(t, err)
		require.True(t, ok)
		return lines
	}
	_, err = loftcontact.OutputCap(t.Context(), proofbound.NewWallWorkBudgetWithOperation(1,
		proofbound.NewWorkBudget(t.Context())), pp.Frame, readLines(pp.Profile.Outer),
		readLines(lp.Profile0.Outer), readLines(lp.Profile1.Outer), source,
		new(loftcontact.OutputShell), 0)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	proof, admitted, err := loftdirect.CertifyLoftOutputBoundary(t.Context(), bodyNode(prism), bodyNode(loft),
		proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, admitted)
	shell := proof.Shell
	shell.Triangles = make([][3]int, 513)
	resolved, err := loftcontact.AuditOutputSource(proofbound.NewWorkBudget(t.Context()), shell, source)
	require.NoError(t, err)
	require.False(t, resolved)
	full := loftcontact.NewOutputShell()
	full.Vertices = make([]loftcontact.OutputVertex, 512)
	_, admitted = full.Vertex(pp.Frame, loftcontact.OutputPoint{
		U: big.NewRat(1, 1), V: big.NewRat(1, 1), Z: big.NewRat(0, 1)},
		loftcontact.OutputFace{})
	require.False(t, admitted)

	count := 0
	probe := &proofbound.WorkBudget{StepFn: func() error { count++; return nil }, ErrFn: func() error { return nil }}
	_, admitted, err = loftdirect.CertifyLoftSourcePartition(t.Context(), bodyNode(prism), bodyNode(loft), probe, false)
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
	_, admitted, err = loftdirect.CertifyLoftOutputBoundary(ctx, bodyNode(prism), bodyNode(loft), operation)
	require.False(t, admitted)
	require.True(t, errors.Is(err, context.Canceled))
	t.Logf("source steps=%d, canceled after %d shared operation steps", count, steps)
}

func TestLoftContactOutputBoundaryMiddleRegionContact(t *testing.T) {
	prism, loft := loftPartitionRealBodies(t,
		[4]float64{8, 4, 12, 6}, [4]float64{8, 4, 13, 6}, 0, 4)
	proof, admitted, err := loftdirect.CertifyLoftOutputBoundary(t.Context(), bodyNode(prism), bodyNode(loft),
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
			shell := proof.Shell
			shell.Vertices = append([]loftcontact.OutputVertex(nil), proof.Shell.Vertices...)
			shell.Triangles = append([][3]int(nil), proof.Shell.Triangles...)
			shell.Faces = append([]loftcontact.OutputFace(nil), proof.Shell.Faces...)
			start := len(shell.Vertices)
			for _, p := range tc.xy {
				shell.Vertices = append(shell.Vertices, loftcontact.OutputVertex{Point: loftcontact.OutputPoint{
					U: big.NewRat(p[0], 1), V: big.NewRat(p[1], 1), Z: big.NewRat(p[2], 1)}})
			}
			shell.Triangles = append(shell.Triangles, [3]int{start, start + 1, start + 2})
			shell.Faces = append(shell.Faces, loftcontact.OutputFace{Kind: 1, Owner: 0})
			directed := make(map[[2]int]int)
			for fi, tri := range shell.Triangles {
				for k := range 3 {
					directed[[2]int{tri[k], tri[(k+1)%3]}] = fi
				}
			}
			resolved, err := loftcontact.OutputMiddleContact(proofbound.NewWorkBudget(t.Context()),
				shell, big.NewRat(4, 1), directed)
			require.NoError(t, err)
			require.Equal(t, tc.want, resolved)
		})
	}
}
