package parts_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func requireVec(t *testing.T, want, got r3.Vec, msg string) {
	t.Helper()
	require.InDelta(t, want.X, got.X, 1e-6, msg+" x")
	require.InDelta(t, want.Y, got.Y, 1e-6, msg+" y")
	require.InDelta(t, want.Z, got.Z, 1e-6, msg+" z")
}

func TestExplodePinAndPlate(t *testing.T) {
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: namePlate, Kind: parts.Manufactured, Body: boredPlate(t, doc)},
		parts.Part{Name: namePin, Kind: parts.Manufactured, Body: cylinder(t, doc, 10, 3, -2, 17)})
	mustJoin(t, s, parts.Joint{Kind: parts.PinInBore, A: namePin, B: namePlate, Motion: parts.Turns})

	ex, err := s.Explode(t.Context(), parts.WithExplodeGround(namePlate))
	require.NoError(t, err)
	require.Equal(t, namePlate, ex.Ground)
	pin := partByName(t, ex, namePin)
	// The pin clears the plate in 12 mm up and 15 mm down; it goes up, 10 mm farther.
	requireVec(t, r3.Vec{Z: 22}, pin.Offset, "pin offset")
	requireVec(t, r3.Vec{Z: 22}, pin.Placement.Apply(r3.Vec{}), "pin placement")
	require.Equal(t, parts.AxisFromFaces, pin.Pull)
	require.Equal(t, 1, pin.Depth)
	// The anchor is the exploded box centre: the pin spans z -2..15.
	requireVec(t, r3.Vec{X: 10, Y: 10, Z: 6.5 + 22}, pin.Anchor, "pin anchor")
	plate := partByName(t, ex, namePlate)
	require.Equal(t, 0, plate.Depth)
	require.Equal(t, parts.PullNone, plate.Pull)
	requireVec(t, r3.Vec{}, plate.Offset, "plate offset")

	// The leader joins the engagement midpoint (z 0..10 overlap) on each part.
	require.Len(t, ex.Leaders, 1)
	require.Equal(t, "pin~plate", ex.Leaders[0].Joint)
	requireVec(t, r3.Vec{X: 10, Y: 10, Z: 5 + 22}, ex.Leaders[0].From, "leader from")
	requireVec(t, r3.Vec{X: 10, Y: 10, Z: 5}, ex.Leaders[0].To, "leader to")
	require.Empty(t, ex.Overlaps)

	// A larger gap moves the pin farther; a zero gap leaves it just clear.
	ex, err = s.Explode(t.Context(), parts.WithExplodeGround(namePlate), parts.WithExplodeGap(units.Millimeters(0)))
	require.NoError(t, err)
	requireVec(t, r3.Vec{Z: 12}, partByName(t, ex, namePin).Offset, "zero gap")
	// The default ground is the manufactured part of largest volume: the plate.
	ex, err = s.Explode(t.Context())
	require.NoError(t, err)
	require.Equal(t, namePlate, ex.Ground)
}

// lapSet is a 20 x 20 x 5 base with a same-size top plate lapped on it and two
// pins through both, at x = 5 and x = 15.
func lapSet(t *testing.T) (*parts.Set, *decad.Document) {
	t.Helper()
	doc := decad.New()
	bore := func(z0 float64) *decad.Body {
		b := block(t, doc, 0, 0, 20, 20, z0, 5)
		for _, x := range []float64{5, 15} {
			var err error
			b, err = decad.Cut(t.Context(), b, cylinder(t, doc, x, 3.2, z0-1, 7))
			require.NoError(t, err)
		}
		return b
	}
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameBase, Kind: parts.Manufactured, Body: bore(0)},
		parts.Part{Name: nameTop, Kind: parts.Manufactured, Body: bore(5)},
		parts.Part{Name: "pin1", Kind: parts.Manufactured, Body: cylinder(t, doc, 5, 3, -2, 14)},
		parts.Part{Name: "pin2", Kind: parts.Manufactured, Body: cylinder(t, doc, 15, 3, -2, 14)})
	mustJoin(t, s,
		parts.Joint{Name: nameLap, Kind: parts.FaceContact, A: nameTop, B: nameBase},
		parts.Joint{Name: "pin1~base", Kind: parts.PinInBore, A: "pin1", B: nameBase, Motion: parts.Turns},
		parts.Joint{Name: "pin2~base", Kind: parts.PinInBore, A: "pin2", B: nameBase, Motion: parts.Turns},
		parts.Joint{Name: "pin1~top", Kind: parts.PinInBore, A: "pin1", B: nameTop, Motion: parts.Turns},
		parts.Joint{Name: "pin2~top", Kind: parts.PinInBore, A: "pin2", B: nameTop, Motion: parts.Turns})
	return s, doc
}

func TestExplodeLap(t *testing.T) {
	s, _ := lapSet(t)
	ex, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBase))
	require.NoError(t, err)

	// The top plate lifts along the contact normal, away from the base: it
	// already clears the base, so only the gap remains.
	top := partByName(t, ex, nameTop)
	requireVec(t, r3.Vec{Z: 10}, top.Offset, nameTop)
	require.Equal(t, parts.FaceNormal, top.Pull)
	// The top plate now sits at z 15..20, so a pin would clear it only after
	// 22 mm up; down it clears the base in 12 mm. It drops 12 mm plus the gap.
	for _, name := range []string{"pin1", "pin2"} {
		p := partByName(t, ex, name)
		require.Equal(t, parts.AxisFromFaces, p.Pull, name)
		require.Equal(t, 1, p.Depth, name)
		requireVec(t, r3.Vec{Z: -22}, p.Offset, name)
	}
	require.Empty(t, ex.Overlaps)
	require.Len(t, ex.Leaders, 5)
}

func TestExplodeBlockedSense(t *testing.T) {
	// A pin through a plate: up and down clear the plate by the same travel,
	// so the positive sense wins. A cap in the pin's upward path blocks it,
	// and the pin takes the downward sense even though it travels as far.
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: namePlate, Kind: parts.Manufactured, Body: boredPlate(t, doc)},
		parts.Part{Name: namePin, Kind: parts.Manufactured, Body: cylinder(t, doc, 10, 3, 0, 10)},
		parts.Part{Name: "cap", Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 20, 20, 25, 2)})
	mustJoin(t, s, parts.Joint{Kind: parts.PinInBore, A: namePin, B: namePlate})
	ex, err := s.Explode(t.Context(), parts.WithExplodeGround(namePlate))
	require.NoError(t, err)
	requireVec(t, r3.Vec{Z: 20}, partByName(t, ex, namePin).Offset, "free upward")

	// Lower the cap into the pin's path: the pin needs 10 mm up, the cap sits 2 mm above the plate.
	doc2 := decad.New()
	s2 := parts.NewSet(doc2)
	mustAdd(t, s2,
		parts.Part{Name: namePlate, Kind: parts.Manufactured, Body: boredPlate(t, doc2)},
		parts.Part{Name: namePin, Kind: parts.Manufactured, Body: cylinder(t, doc2, 10, 3, 0, 10)},
		parts.Part{Name: "cap", Kind: parts.Manufactured, Body: block(t, doc2, 0, 0, 20, 20, 12, 2)})
	mustJoin(t, s2, parts.Joint{Kind: parts.PinInBore, A: namePin, B: namePlate})
	ex2, err := s2.Explode(t.Context(), parts.WithExplodeGround(namePlate))
	require.NoError(t, err)
	requireVec(t, r3.Vec{Z: -20}, partByName(t, ex2, namePin).Offset, "blocked upward")
}

func TestExplodeUnreached(t *testing.T) {
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: "a", Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 20, 20, 0, 10)},
		parts.Part{Name: "b", Kind: parts.Manufactured, Body: block(t, doc, 40, 0, 50, 10, 0, 5)},
		parts.Part{Name: nameStrap, Kind: parts.Purchased, Quantity: 2})
	ex, err := s.Explode(t.Context(), parts.WithExplodeGround("a"))
	require.NoError(t, err)
	require.Len(t, ex.Parts, 2, "a body-less part is omitted")
	b := partByName(t, ex, "b")
	require.Equal(t, -1, b.Depth)
	requireVec(t, r3.Vec{}, b.Offset, "unreached")
	require.Equal(t, parts.PullNone, b.Pull)

	// Contact joints fall back to the line between box centres.
	mustJoin(t, s, parts.Joint{Kind: parts.Contact, A: "b", B: "a"}, parts.Joint{Kind: parts.Tie, A: nameStrap, B: "a"})
	ex, err = s.Explode(t.Context(), parts.WithExplodeGround("a"))
	require.NoError(t, err)
	b = partByName(t, ex, "b")
	require.Equal(t, parts.BoxCentres, b.Pull)
	require.Equal(t, 1, b.Depth)
	// b sits at +X of a, 20 mm clear after zero travel... travel is 30-... a spans x 0..20, b starts at 40,
	// so it already clears and only the gap remains along the centre line.
	require.Greater(t, b.Offset.X, 9.0)
	require.Len(t, ex.Leaders, 1, "a tie to a body-less part draws no leader")

	_, err = s.Explode(t.Context(), parts.WithExplodeGround(nameStrap))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = s.Explode(t.Context(), nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = s.Explode(t.Context(), parts.WithExplodeGap(units.Degrees(1)))
	require.ErrorIs(t, err, decad.ErrUnitKind)
}

func TestExplodeFacetedBoreFallsBack(t *testing.T) {
	// A plate imported as a mesh has Faceted faces: no axis is read from it,
	// so the declared axis is used, and without one the box centres.
	doc := decad.New()
	plate := facetedCopy(t, doc, boredPlate(t, doc))
	pin := cylinder(t, doc, 10, 3, -2, 17)

	build := func(axis *parts.Axis) *parts.Exploded {
		s := parts.NewSet(doc)
		mustAdd(t, s, parts.Part{Name: namePlate, Kind: parts.Manufactured, Body: plate},
			parts.Part{Name: namePin, Kind: parts.Manufactured, Body: pin})
		mustJoin(t, s, parts.Joint{Kind: parts.PinInBore, A: namePin, B: namePlate, Axis: axis})
		ex, err := s.Explode(t.Context(), parts.WithExplodeGround(namePlate))
		require.NoError(t, err)
		return ex
	}
	declared := build(&parts.Axis{Point: r3.Vec{X: 10, Y: 10}, Dir: r3.Vec{Z: 1}})
	p := partByName(t, declared, namePin)
	require.Equal(t, parts.AxisDeclared, p.Pull)
	requireVec(t, r3.Vec{Z: 22}, p.Offset, "declared axis")

	guessed := build(nil)
	require.Equal(t, parts.BoxCentres, partByName(t, guessed, namePin).Pull)
}

func TestExplodePinInSlot(t *testing.T) {
	// A Ø4 pin stands in a 4.4 mm wide slot cut through a 30 x 20 x 10 block.
	doc := decad.New()
	blk, err := decad.Cut(t.Context(), block(t, doc, 0, 0, 30, 20, 0, 10), block(t, doc, 12.8, 5, 17.2, 15, -1, 12))
	require.NoError(t, err)
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameBlock, Kind: parts.Manufactured, Body: blk},
		parts.Part{Name: namePin, Kind: parts.Manufactured, Body: cylinder(t, doc, 15, 2, -2, 15)})
	mustJoin(t, s, parts.Joint{Kind: parts.PinInSlot, A: namePin, B: nameBlock, Motion: parts.Slides})
	ex, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBlock))
	require.NoError(t, err)
	pin := partByName(t, ex, namePin)
	require.Equal(t, parts.AxisFromFaces, pin.Pull)
	requireVec(t, r3.Vec{Z: 22}, pin.Offset, namePin)
	requireVec(t, r3.Vec{X: 15, Y: 10, Z: 5 + 22}, ex.Leaders[0].From, "leader")

	// Walls too far apart for MaxFit are not a match: the box centres are used.
	loose := parts.NewSet(doc)
	mustAdd(t, loose,
		parts.Part{Name: nameBlock, Kind: parts.Manufactured, Body: blk},
		parts.Part{Name: namePin, Kind: parts.Manufactured, Body: cylinder(t, doc, 15, 2, -2, 15)})
	mustJoin(t, loose, parts.Joint{Kind: parts.PinInSlot, A: namePin, B: nameBlock})
	ex, err = loose.Explode(t.Context(), parts.WithExplodeGround(nameBlock), parts.WithMaxFit(units.Millimeters(0.3)))
	require.NoError(t, err)
	require.Equal(t, parts.BoxCentres, partByName(t, ex, namePin).Pull)
}

func TestExplodeOptionsAndContext(t *testing.T) {
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s, parts.Part{Name: "a", Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 5, 5, 0, 5)})

	_, err := s.Explode(t.Context(), nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	for name, tc := range map[string]struct {
		opt  parts.CheckOption
		want error
	}{
		"fit kind":      {parts.WithFitTolerance(units.Degrees(1)), decad.ErrUnitKind},
		"fit negative":  {parts.WithFitTolerance(units.Millimeters(-1)), decad.ErrNegativeMagnitude},
		"fit zero":      {parts.WithFitTolerance(units.Millimeters(0)), decad.ErrDegenerate},
		"max fit nan":   {parts.WithMaxFit(units.Millimeters(math.NaN())), decad.ErrNotFinite},
		"axis kind":     {parts.WithAxisTolerance(units.Millimeters(1)), decad.ErrUnitKind},
		"axis too wide": {parts.WithAxisTolerance(units.Degrees(90)), decad.ErrDegenerate},
		"contact kind":  {parts.WithContactTolerance(units.Degrees(1)), decad.ErrUnitKind},
		"nil verify":    {parts.WithVerifyReport(nil), decad.ErrDegenerate},
	} {
		_, err := s.Explode(t.Context(), tc.opt)
		require.ErrorIs(t, err, tc.want, name)
	}
	_, err = s.Explode(t.Context(), parts.WithFitTolerance(units.Millimeters(0.1)),
		parts.WithAxisTolerance(units.Degrees(1)), parts.WithMaxFit(units.Millimeters(2)))
	require.NoError(t, err)

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = s.Explode(ctx)
	require.ErrorIs(t, err, context.Canceled)
	// Options are validated before the context is read.
	_, err = s.Explode(ctx, parts.WithFitTolerance(units.Degrees(1)))
	require.ErrorIs(t, err, decad.ErrUnitKind)
}
