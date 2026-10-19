package loftcontact_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestLoftContactOutputBoundaryPlanarityAndVertexRefusals(t *testing.T) {
	low := sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 0, V: 0},
		End: sectionrecord.Point2{U: 1, V: 0}, TStart: 0, TEnd: 1}
	high := sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 0, V: 0},
		End: sectionrecord.Point2{U: 1, V: 1}, TStart: 0, TEnd: 1}
	require.False(t, loftcontact.OutputPlanarWall(low, high, big.NewRat(4, 1)))
	require.True(t, loftcontact.OutputPlanarWall(low, low, big.NewRat(4, 1)))
	frame := xyFrame(t)
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
	require.False(t, loftcontact.OutputPlanarWall(low, high, large))
	require.False(t, loftcontact.OutputNormalsAgree(
		loftcontact.OutputVertex{World: [3]*big.Rat{large, zero, zero}},
		loftcontact.OutputVertex{World: [3]*big.Rat{one, zero, zero}},
		loftcontact.OutputVertex{World: [3]*big.Rat{zero, one, zero}},
	))
	// The same triangle with a held copy agrees once its sizes fit.
	require.True(t, loftcontact.OutputNormalsAgree(
		loftcontact.OutputVertex{World: [3]*big.Rat{zero, zero, zero}},
		loftcontact.OutputVertex{World: [3]*big.Rat{one, zero, zero}, Held: xyFrame(t).U()},
		loftcontact.OutputVertex{World: [3]*big.Rat{zero, one, zero}, Held: xyFrame(t).V()},
	))
}

func TestLoftOutputSideRefusesOversizedLineInterpolation(t *testing.T) {
	frame := xyFrame(t)
	line := sectionrecord.LineSeg{
		Start: sectionrecord.Point2{U: 1, V: 0},
		End:   sectionrecord.Point2{U: math.Nextafter(1, 2), V: 0},
	}
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 8150))
	shell := loftcontact.NewOutputShell()
	admitted, err := loftcontact.OutputLineQuad(proofbound.NewWorkBudget(t.Context()), frame,
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
	trace, _, err := loftcontact.OutputPlaneTrace(proofbound.NewWorkBudget(t.Context()), shell,
		[3]int{0, 1, 2}, big.NewRat(0, 1), arithmetic)
	require.NoError(t, err)
	require.False(t, arithmetic.OK)
	require.Empty(t, trace)
}
