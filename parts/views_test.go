package parts_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func viewOf(t *testing.T, v *parts.Views, name string) parts.PartView {
	t.Helper()
	for _, p := range v.Parts {
		if p.Part == name {
			return p
		}
	}
	require.Failf(t, "missing view", "no view of %q", name)
	return parts.PartView{}
}

func TestViewsCentringAndPorts(t *testing.T) {
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: namePlate, Kind: parts.Manufactured, Body: boredPlate(t, doc)},
		parts.Part{Name: namePin, Kind: parts.Manufactured, Body: cylinder(t, doc, 10, 3, -2, 17)},
		parts.Part{Name: nameStrap, Kind: parts.Purchased, Quantity: 1})
	mustJoin(t, s,
		parts.Joint{Name: "pivot", Kind: parts.PinInBore, A: namePin, B: namePlate, Motion: parts.Turns},
		parts.Joint{Name: "band", Kind: parts.Tie, A: nameStrap, B: namePlate})

	v, err := s.Views(t.Context())
	require.NoError(t, err)
	require.Len(t, v.Parts, 2, "a body-less part has no view")

	// Each part is centred at the origin, keeping its orientation.
	plate := viewOf(t, v, namePlate)
	requireVec(t, r3.Vec{X: -10, Y: -10, Z: -5}, plate.Bounds.Min, "plate min")
	requireVec(t, r3.Vec{X: 10, Y: 10, Z: 5}, plate.Bounds.Max, "plate max")
	requireVec(t, r3.Vec{Z: 5}, plate.Anchor, "plate anchor")
	pin := viewOf(t, v, namePin)
	requireVec(t, r3.Vec{X: -3, Y: -3, Z: -8.5}, pin.Bounds.Min, "pin min")
	requireVec(t, r3.Vec{X: 3, Y: 3, Z: 8.5}, pin.Bounds.Max, "pin max")
	requireVec(t, r3.Vec{X: -10, Y: -10, Z: -5}, plate.Placement.Apply(r3.Vec{}), "plate placement")

	// The pin's port is the middle of its engagement (z 0..10), on the axis,
	// in each part's own view coordinates.
	require.Len(t, pin.Ports, 1)
	require.Equal(t, "pivot", pin.Ports[0].Joint)
	require.Equal(t, namePlate, pin.Ports[0].Other)
	requireVec(t, r3.Vec{Z: -1.5}, pin.Ports[0].Point, "pin port")
	requireVec(t, r3.Vec{Z: 1}, pin.Ports[0].Dir, "pin port dir")
	// The plate has the pivot port and the tie, which falls back to its own box centre.
	require.Len(t, plate.Ports, 2)
	requireVec(t, r3.Vec{}, plate.Ports[0].Point, "plate port")
	require.Equal(t, "band", plate.Ports[1].Joint)
	require.Equal(t, nameStrap, plate.Ports[1].Other)
	requireVec(t, r3.Vec{}, plate.Ports[1].Point, "tie port")
	requireVec(t, r3.Vec{}, plate.Ports[1].Dir, "tie dir")
}

func TestViewsPosesAndFallbackPorts(t *testing.T) {
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: "a", Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 10, 10, 5, 5)},
		parts.Part{Name: nameBase, Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 10, 10, 0, 5)},
		parts.Part{Name: "side", Kind: parts.Manufactured, Body: block(t, doc, 10, 0, 20, 10, 0, 5)})
	mustJoin(t, s,
		parts.Joint{Name: nameLap, Kind: parts.FaceContact, A: "a", B: nameBase},
		parts.Joint{Name: "snap", Kind: parts.Contact, A: "side", B: nameBase})

	// A turn of 90 degrees about Z, then 10 mm along X, poses part a.
	turn, err := r3.Rotation(r3.Vec{Z: 1}, unitsDegrees(90))
	require.NoError(t, err)
	pose, err := turn.Then(translationOf(t, r3.Vec{X: 10}))
	require.NoError(t, err)
	v, err := s.Views(t.Context(), parts.WithPartPose("a", pose))
	require.NoError(t, err)

	// (x, y) -> (-y, x) + (10, 0): a's box 0..10 x 0..10 x 5..10 becomes x 0..10, y 0..10.
	a := viewOf(t, v, "a")
	require.InDelta(t, 0, a.Bounds.Min.X, 1e-6)
	require.InDelta(t, 10, a.Bounds.Max.X, 1e-6)
	require.InDelta(t, 0, a.Bounds.Min.Y, 1e-6)
	require.InDelta(t, 10, a.Bounds.Max.Y, 1e-6)
	require.InDelta(t, 5, a.Bounds.Min.Z, 1e-6)
	require.InDelta(t, 10, a.Bounds.Max.Z, 1e-6)
	require.Equal(t, decad.Approximate, a.Bounds.Exactness, "a turned box reads from its mesh")
	// The lap port is the centre of the shared face (5, 5, 5), turned with the part; its direction is a's
	// own face normal, -Z.
	require.Len(t, a.Ports, 1)
	requireVec(t, pose.Apply(r3.Vec{X: 5, Y: 5, Z: 5}), a.Ports[0].Point, "lap port on a")
	requireVec(t, r3.Vec{Z: -1}, a.Ports[0].Dir, "lap dir on a")

	// The base keeps its own pose: its lap port faces +Z, and the contact with side falls back to the
	// centre of the boxes' shared plane x = 10.
	base := viewOf(t, v, nameBase)
	require.Len(t, base.Ports, 2)
	requireVec(t, r3.Vec{Z: 1}, base.Ports[0].Dir, "lap dir on base")
	requireVec(t, r3.Vec{X: 5, Y: 0, Z: 0}, base.Ports[1].Point, "snap port on base")
	requireVec(t, r3.Vec{}, base.Ports[1].Dir, "snap dir")

	mirror, err := r3.Reflection(frameXY(t))
	require.NoError(t, err)
	_, err = s.Views(t.Context(), parts.WithPartPose("a", mirror))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = s.Views(t.Context(), parts.WithPartPose("zz", r3.Identity()))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = s.Views(t.Context(), nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
