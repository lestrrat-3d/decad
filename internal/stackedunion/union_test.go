package stackedunion_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/stackedunion"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func rectRecord(u0, v0, u1, v1 float64) momentinput.Profile {
	corners := []sectionrecord.Point2{{U: u0, V: v0}, {U: u1, V: v0}, {U: u1, V: v1}, {U: u0, V: v1}}
	segments := make([]sectionrecord.CurveSegment, len(corners))
	for i, corner := range corners {
		segments[i] = sectionrecord.LineSeg{Start: corner, End: corners[(i+1)%len(corners)], TEnd: 1}
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segments}}
}

// TestJoinBrepStackCarriesDenotation unions a plate over [-3, 2] with a boss
// over [-3, 0.2 in] that overhangs the plate's edge. The interface crosses,
// so the join builds an A1 BRep, and its stack's top level keeps the rational
// the boss's top denotes.
func TestJoinBrepStackCarriesDenotation(t *testing.T) {
	t.Parallel()
	r := new(big.Rat).Mul(new(big.Rat).SetFloat64(0.2), new(big.Rat).SetFloat64(25.4))
	held, _ := r.Float64()
	delta := proofarith.RationalFloatError(r, held)
	require.Positive(t, delta, "0.2 in is a displaced level")
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	plate := prismrecord.Payload{Profile: rectRecord(0, 0, 20, 20), Frame: frame, Z0: -3, Z1: 2, Xform: r3.Identity()}
	boss := prismrecord.Payload{Profile: rectRecord(15, 5, 25, 15), Frame: frame, Z0: -3,
		Z1: held, Z1Delta: delta, Z1Denoted: prismplacement.DenoteLevel(r, held, delta), Xform: r3.Identity()}
	va, ok := stackedunion.OperandFromPayload(plate)
	require.True(t, ok)
	vb, ok := stackedunion.OperandFromPayload(boss)
	require.True(t, ok)
	shift := new(big.Rat)
	levels, ok := stackedrecord.UnionLevels(va.Slabs, vb.Slabs, shift)
	require.True(t, ok)
	require.Len(t, levels, 3)
	reexpress, err := prismcells.NewReexpression(va.Proxy.PlacementOperand(), vb.Proxy.PlacementOperand())
	require.NoError(t, err)
	payload, ok, err := stackedunion.Join(t.Context(), proofbound.NewWorkBudget(t.Context()), va, vb,
		reexpress, levels, shift, false)
	require.NoError(t, err)
	require.True(t, ok)
	brep, isBrep := payload.(breprecord.Payload)
	require.True(t, isBrep, "the crossing interface builds an A1 BRep")
	require.NotNil(t, brep.Stack)
	slabs := brep.Stack.Slabs
	require.Len(t, slabs, 2)
	got := slabs[1].DenotedZ1()
	require.NotNil(t, got, "the stack's top keeps the boss's denotation")
	require.Zero(t, got.Cmp(r))
	below := slabs[0].DenotedZ1()
	require.NotNil(t, below)
	require.Zero(t, below.Cmp(big.NewRat(2, 1)), "the plate's top denotes 2 exactly")
}
