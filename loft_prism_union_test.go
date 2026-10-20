package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismboolean"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func axialLineLoft(t *testing.T, doc *Document, x0, y0, x1, y1, height float64) *Body {
	t.Helper()
	world := sketch.NewWorld()
	top, err := world.CreateOffsetPlane(world.XY(), height)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{world.XY(), top} {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		rect := s.CreateRectangle(x0, y0, x1, y1)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	body, err := Loft(t.Context(), doc, sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	return body
}

func TestAxialLoftPrismUnionRectangularTab(t *testing.T) {
	doc := New()
	blank := internalBoxBody(t, doc, 0, 0, 10, 10, 4)
	tab := axialLineLoft(t, doc, 8, 4, 12, 6, 4)
	joined, err := Union(t.Context(), blank, tab)
	require.NoError(t, err)
	_, analytic := joined.n.Payload.(prismPayload)
	require.True(t, analytic, "the certified source route should return an analytic prism")
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-416), volume.Bound.Base())
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.NotEqual(t, blank.n.Origin.Producer, joined.n.Origin.Producer)
	require.NotEqual(t, tab.n.Origin.Producer, joined.n.Origin.Producer)
	capRoles := map[string]int{}
	for _, face := range joined.Faces() {
		for _, origin := range face.n.Origins {
			if origin.Producer == joined.n.Origin.Producer {
				capRoles[origin.Role]++
			}
		}
	}
	require.Equal(t, 1, capRoles["capStart"])
	require.Equal(t, 1, capRoles["capEnd"])
	bounds, err := joined.Bounds()
	require.NoError(t, err)
	require.LessOrEqual(t, bounds.Min.Sub(r3.Vec{}).Len(), bounds.Bound.Base())
	require.LessOrEqual(t, bounds.Max.Sub(r3.Vec{X: 12, Y: 10, Z: 4}).Len(), bounds.Bound.Base())
}

func TestAxialLoftPrismUnionIndependentUnequalTabs(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: loftOperandPrismFirst, true: loftOperandLoftFirst}[reverse], func(t *testing.T) {
			doc := New()
			blank := internalBoxBody(t, doc, 0, 0, 10, 10, 4)
			left := axialLineLoft(t, doc, -3, 7, 2, 9, 4)
			right := axialLineLoft(t, doc, 8, 4, 12, 6, 4)
			leftSource := left.n.Payload.(loftPayload).SourceIdentity
			rightSource := right.n.Payload.(loftPayload).SourceIdentity
			require.NotSame(t, leftSource, rightSource)
			var first *Body
			var err error
			if reverse {
				first, err = Union(t.Context(), right, blank)
			} else {
				first, err = Union(t.Context(), blank, right)
			}
			require.NoError(t, err)
			joined, err := Union(t.Context(), first, left)
			require.NoError(t, err)
			_, analytic := joined.n.Payload.(prismPayload)
			require.True(t, analytic)
			volume, err := joined.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-440), volume.Bound.Base())
		})
	}
}

func TestAxialLoftPrismUnionManyIndependentTabs(t *testing.T) {
	for _, order := range []struct {
		name    string
		reverse bool
	}{{name: "left_right"}, {name: "right_left", reverse: true}} {
		t.Run(order.name, func(t *testing.T) {
			doc := New()
			blank := internalBoxBody(t, doc, 0, 0, 10, 10, 4)
			left := axialLineLoft(t, doc, -3, 7, 2, 9, 4)
			right := axialLineLoft(t, doc, 8, 4, 12, 6, 4)
			if order.reverse {
				left, right = right, left
			}
			operands := []*Body{blank, left, right}
			_, admitted, err := prismboolean.StraightLoftUnionMany(t.Context(), envOf(documentNode(doc), doc.nextProducerID()),
				bodyNodes(operands))
			require.NoError(t, err)
			require.True(t, admitted, "the independent tabs should use one prism scene")
			joined, err := Union(t.Context(), operands[0], operands[1:]...)
			require.NoError(t, err)
			_, analytic := joined.n.Payload.(prismPayload)
			require.True(t, analytic)
			volume, err := joined.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-440), volume.Bound.Base())
		})
	}
}

func TestAxialLoftPrismUnionTangentDoesNotAdmit(t *testing.T) {
	doc := New()
	blank := internalBoxBody(t, doc, 0, 0, 10, 10, 4)
	tangent := axialLineLoft(t, doc, 10, 4, 12, 6, 4)
	_, admitted, err := prismboolean.TryAxialLoftUnion(t.Context(), meshbool.OpUnion, blank.n.Payload, tangent.n.Payload)
	require.NoError(t, err)
	require.False(t, admitted)
	valid := axialLineLoft(t, doc, 8, 4, 12, 6, 4)
	_, admitted, err = prismboolean.StraightLoftUnionMany(t.Context(), envOf(documentNode(doc), doc.nextProducerID()),
		bodyNodes([]*Body{blank, valid, tangent}))
	require.NoError(t, err)
	require.False(t, admitted)
}

func TestAxialLoftPrismUnionRefusesVaryingSource(t *testing.T) {
	world := sketch.NewWorld()
	top, err := world.CreateOffsetPlane(world.XY(), 4)
	require.NoError(t, err)
	sections := [2]*sketch.Plane{world.XY(), top}
	var sketches [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range sections {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		x1 := 12.0
		if i == 1 {
			x1 = 13
		}
		rect := s.CreateRectangle(8, 4, x1, 6)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		sketches[i], profiles[i] = s, s.Profiles()[0]
	}
	body, err := Loft(t.Context(), New(), sketches[0], profiles[0], sketches[1], profiles[1])
	require.NoError(t, err)
	_, admitted, err := loftrecord.ExactAxialPrism(t.Context(), body.n.Payload.(loftPayload))
	require.NoError(t, err)
	require.False(t, admitted)
}

func TestAxialLoftPrismUnionRefusesUnequalCapsAndCancellation(t *testing.T) {
	doc := New()
	blank := internalBoxBody(t, doc, 0, 0, 10, 10, 4)
	tab := axialLineLoft(t, doc, 8, 4, 12, 6, 5)
	_, admitted, err := prismboolean.TryAxialLoftUnion(t.Context(), meshbool.OpUnion, blank.n.Payload, tab.n.Payload)
	require.NoError(t, err)
	require.False(t, admitted)

	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, admitted, err = prismboolean.TryAxialLoftUnion(canceled, meshbool.OpUnion, blank.n.Payload, tab.n.Payload)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, admitted)
}

// The straight-source Loft route answers false, with no error, unless the
// first operand is a prism, every other operand is a prism or an exact axial
// Loft, at least one is a Loft, and each Loft overlaps the first prism.
func TestStraightLoftPrismUnionManyGates(t *testing.T) {
	doc := New()
	blank := internalBoxBody(t, doc, 0, 0, 10, 10, 4)
	other := internalBoxBody(t, doc, 20, 0, 30, 10, 4)
	loft := axialLineLoft(t, doc, 8, 4, 12, 6, 4)
	far := axialLineLoft(t, doc, 40, 4, 44, 6, 4)
	run := func(operands ...*Body) (bool, error) {
		t.Helper()
		_, ok, err := prismboolean.StraightLoftUnionMany(t.Context(), envOf(documentNode(doc), doc.nextProducerID()),
			bodyNodes(operands))
		return ok, err
	}
	for name, operands := range map[string][]*Body{
		"first operand is a loft":      {loft, blank},
		"another operand is neither":   {blank, {n: topo.Body{}}},
		"no loft among the operands":   {blank, other},
		"the loft misses the prism":    {blank, far},
		"one loft overlaps, one not":   {blank, loft, far},
		"the far loft comes first now": {blank, far, loft},
	} {
		ok, err := run(operands...)
		require.NoError(t, err, name)
		require.False(t, ok, name)
	}
	ok, err := run(blank, loft)
	require.NoError(t, err)
	require.True(t, ok, "an overlapping straight loft with a prism is the class")

	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	_, ok, err = prismboolean.StraightLoftUnionMany(cancelled, envOf(documentNode(doc), doc.nextProducerID()),
		bodyNodes([]*Body{blank, loft}))
	require.True(t, ok)
	require.ErrorIs(t, err, context.Canceled)
}
