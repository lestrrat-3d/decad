package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

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
				proof, admitted, err := certifyLoftOutputBoundary(t.Context(), pair[0], pair[1],
					proofbound.NewWorkBudget(t.Context()))
				require.NoError(t, err)
				require.True(t, admitted)
				require.NotEmpty(t, proof.shell.Triangles)
				require.NotEmpty(t, proof.shell.Vertices)
				require.Positive(t, proof.workVisits)
				require.LessOrEqual(t, proof.workVisits, uint64(65_536))
				require.LessOrEqual(t, len(proof.shell.Triangles), 512)
				require.LessOrEqual(t, len(proof.shell.Vertices), 512)
				if tc.name == "right x" {
					require.Equal(t, [3]string{"104", "6", "100"},
						loftOutputTestCapAreas(proof.shell))
				}
				t.Logf("work visits=%d facets=%d vertices=%d bound=%g",
					proof.workVisits, len(proof.shell.Triangles),
					len(proof.shell.Vertices), proof.shell.MaxBound)
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
			shell := proof.shell
			shell.Triangles = append([][3]int(nil), proof.shell.Triangles...)
			shell.Faces = append([]loftcontact.OutputFace(nil), proof.shell.Faces...)
			shell.TriangleBounds = append([]float64(nil), proof.shell.TriangleBounds...)
			tc.change(&shell)
			resolved, err := loftOutputAuditSource(proofbound.NewWorkBudget(t.Context()), shell, source)
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
	_, err = loftcontact.OutputCap(t.Context(), proofbound.NewWallWorkBudgetWithOperation(1,
		proofbound.NewWorkBudget(t.Context())), pp.Frame, readLines(pp.Profile.Outer),
		readLines(lp.Profile0.Outer), readLines(lp.Profile1.Outer), source,
		new(loftcontact.OutputShell), 0)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	proof, admitted, err := certifyLoftOutputBoundary(t.Context(), prism, loft,
		proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, admitted)
	shell := proof.shell
	shell.Triangles = make([][3]int, 513)
	resolved, err := loftOutputAuditSource(proofbound.NewWorkBudget(t.Context()), shell, source)
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
			shell.Vertices = append([]loftcontact.OutputVertex(nil), proof.shell.Vertices...)
			shell.Triangles = append([][3]int(nil), proof.shell.Triangles...)
			shell.Faces = append([]loftcontact.OutputFace(nil), proof.shell.Faces...)
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
	shell := loftcontact.NewOutputShell()
	face := loftcontact.OutputFace{Kind: 0}
	_, admitted := shell.Vertex(frame, loftcontact.OutputPoint{
		U: big.NewRat(1, 1), V: big.NewRat(1, 1), Z: big.NewRat(0, 1)}, face)
	require.True(t, admitted)
	u := new(big.Rat).Add(big.NewRat(1, 1),
		new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 1000)))
	_, admitted = shell.Vertex(frame, loftcontact.OutputPoint{
		U: u, V: big.NewRat(1, 1), Z: big.NewRat(0, 1)}, face)
	require.False(t, admitted)
	_, _, bound, admitted := loftcontact.OutputWorld(frame, loftcontact.OutputPoint{
		U: big.NewRat(1, 3), V: big.NewRat(0, 1), Z: big.NewRat(0, 1)})
	require.True(t, admitted)
	require.Positive(t, bound)
}

func TestLoftOutputOrientationRefusesOversizedReplayArithmetic(t *testing.T) {
	zero, one := big.NewRat(0, 1), big.NewRat(1, 1)
	large := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 8192))
	low := sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 0, V: 0},
		End: sectionrecord.Point2{U: 1, V: 0}}
	high := low
	require.False(t, loftOutputPlanarWall(low, high, large))
	require.False(t, loftOutputNormalsAgree(
		loftcontact.OutputVertex{World: [3]*big.Rat{large, zero, zero}},
		loftcontact.OutputVertex{World: [3]*big.Rat{one, zero, zero}},
		loftcontact.OutputVertex{World: [3]*big.Rat{zero, one, zero}},
	))
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
	shell := loftcontact.NewOutputShell()
	admitted, err := loftOutputLineQuad(proofbound.NewWorkBudget(t.Context()), frame,
		&shell, line, line, big.NewRat(0, 1), tiny, big.NewRat(0, 1), tiny,
		big.NewRat(0, 1), big.NewRat(1, 1), loftcontact.OutputFace{Kind: 1})
	require.NoError(t, err)
	require.False(t, admitted)
	require.Empty(t, shell.Triangles)
}

func TestLoftOutputMiddleTraceRefusesOversizedHeightDifference(t *testing.T) {
	first := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 5000))
	second := new(big.Rat).SetFrac(big.NewInt(-1), new(big.Int).Lsh(big.NewInt(1), 5001))
	shell := loftcontact.OutputShell{Vertices: []loftcontact.OutputVertex{
		{Point: loftcontact.OutputPoint{U: big.NewRat(0, 1), V: big.NewRat(0, 1), Z: first}},
		{Point: loftcontact.OutputPoint{U: big.NewRat(1, 1), V: big.NewRat(0, 1), Z: second}},
		{Point: loftcontact.OutputPoint{U: big.NewRat(0, 1), V: big.NewRat(1, 1), Z: first}},
	}}
	arithmetic := &loftcontact.Arithmetic{OK: true}
	trace, _, err := loftOutputPlaneTrace(proofbound.NewWorkBudget(t.Context()), shell,
		[3]int{0, 1, 2}, big.NewRat(0, 1), arithmetic)
	require.NoError(t, err)
	require.False(t, arithmetic.OK)
	require.Empty(t, trace)
}
