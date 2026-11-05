package parts_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/linkage"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// hingeAndSlider is a 40 x 20 x 10 base with a Ø6.4 bore at (10, 10) and a 4.4 mm slot
// running along Y at x = 30, a pin turning in the bore and a pin sliding in the slot.
func hingeAndSlider(t *testing.T) (*parts.Set, *decad.Document) {
	t.Helper()
	doc := decad.New()
	base := block(t, doc, 0, 0, 40, 20, 0, 10)
	base, err := decad.Cut(t.Context(), base, cylinder(t, doc, 10, 3.2, -1, 12))
	require.NoError(t, err)
	base, err = decad.Cut(t.Context(), base, block(t, doc, 27.8, 4, 32.2, 16, -1, 12))
	require.NoError(t, err)
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameBase, Kind: parts.Manufactured, Body: base},
		parts.Part{Name: nameHinge, Kind: parts.Manufactured, Body: cylinder(t, doc, 10, 3, -2, 17)},
		parts.Part{Name: "slider", Kind: parts.Manufactured, Body: cylinder(t, doc, 30, 2, -2, 17)},
		parts.Part{Name: nameStrap, Kind: parts.Purchased, Quantity: 1})
	mustJoin(t, s,
		parts.Joint{Name: "turn", Kind: parts.PinInBore, A: nameHinge, B: nameBase, Motion: parts.Turns},
		parts.Joint{Name: "slide", Kind: parts.PinInSlot, A: "slider", B: nameBase, Motion: parts.Slides})
	return s, doc
}

func TestLinkage(t *testing.T) {
	s, doc := hingeAndSlider(t)
	rep, err := s.Check(t.Context())
	require.NoError(t, err)
	l, unplaced, err := s.Linkage(nameBase, rep)
	require.NoError(t, err)
	require.Empty(t, unplaced)

	// Two links hang off the ground: a revolute link about the bore's axis and a prismatic link along the slot.
	links := l.Links()
	require.Len(t, links, 2)
	rev, ok := links[0].Joint().(linkage.RevoluteJoint)
	require.True(t, ok, "the hinge is a revolute")
	requireVec(t, r3.Vec{X: 10, Y: 10, Z: 5}, rev.Center, "revolute centre")
	require.InDelta(t, 1.0, rev.Axis.Z*rev.Axis.Z, 1e-9)
	pri, ok := links[1].Joint().(linkage.PrismaticJoint)
	require.True(t, ok, "the slider is a prismatic")
	require.InDelta(t, 1.0, pri.Dir.Y*pri.Dir.Y, 1e-9, "it slides along the slot (Y), not across it")
	require.Equal(t, []*decad.Body{s.Parts()[1].Body}, links[0].Bodies())
	require.Equal(t, []*decad.Body{s.Parts()[2].Body}, links[1].Bodies())

	// Each joint's pair is declared as a contact, and VerifyLinkage runs on the result.
	require.Len(t, l.JointContacts(), 2)
	drive := decad.Drive{
		{Link: links[0], From: units.Degrees(0), To: units.Degrees(90)},
		{Link: links[1], From: units.Millimeters(0), To: units.Millimeters(4)},
	}
	report, err := doc.VerifyLinkage(t.Context(), l, drive, decad.WithResolution(units.Scalar(1.0/16)))
	require.NoError(t, err)
	require.NotNil(t, report)
}

func TestLinkageUnplaced(t *testing.T) {
	s, _ := hingeAndSlider(t)
	// A joint that joins the two moving links closes a loop; a Turns joint with no axis anywhere,
	// and a TurnsAndSlides joint, cannot be placed either.
	doc := s.Document()
	mustAdd(t, s,
		parts.Part{Name: nameArm, Kind: parts.Manufactured, Body: block(t, doc, 50, 0, 60, 10, 0, 5)},
		parts.Part{Name: "sleeve", Kind: parts.Manufactured, Body: block(t, doc, 70, 0, 80, 10, 0, 5)})
	mustJoin(t, s,
		parts.Joint{Name: "loop", Kind: parts.FaceContact, A: nameHinge, B: "slider", Motion: parts.Slides,
			Direction: r3.Vec{X: 1}},
		parts.Joint{Name: "noaxis", Kind: parts.PinInBore, A: nameArm, B: nameBase, Motion: parts.Turns},
		parts.Joint{Name: "cyl", Kind: parts.PinInBore, A: "sleeve", B: nameBase, Motion: parts.TurnsAndSlides,
			Axis: &parts.Axis{Point: r3.Vec{X: 75, Y: 5}, Dir: r3.Vec{Z: 1}}},
		parts.Joint{Name: "keep", Kind: parts.Keepout, A: nameArm, B: "sleeve", MinGap: units.Millimeters(1)})
	rep, err := s.Check(t.Context())
	require.NoError(t, err)
	l, unplaced, err := s.Linkage(nameBase, rep)
	require.NoError(t, err)
	var names []string
	for _, j := range unplaced {
		names = append(names, j.Name)
	}
	require.ElementsMatch(t, []string{"loop", "noaxis", "cyl"}, names)
	require.Len(t, l.Links(), 2, "only the hinge and the slider are placed")

	// Without a report the joint's declared axis is all there is: the hinge has none, so it is
	// unplaced, while a declared axis places it.
	s2, _ := hingeAndSlider(t)
	_, unplaced, err = s2.Linkage(nameBase, nil)
	require.NoError(t, err)
	require.Len(t, unplaced, 2)
	doc3 := decad.New()
	s3 := parts.NewSet(doc3)
	mustAdd(t, s3,
		parts.Part{Name: nameBase, Kind: parts.Manufactured, Body: block(t, doc3, 0, 0, 20, 20, 0, 10)},
		parts.Part{Name: nameArm, Kind: parts.Manufactured, Body: block(t, doc3, 20, 0, 40, 20, 0, 10)})
	mustJoin(t, s3, parts.Joint{Kind: parts.PinInBore, A: nameArm, B: nameBase, Motion: parts.Turns,
		Axis: &parts.Axis{Point: r3.Vec{X: 20, Y: 10}, Dir: r3.Vec{Z: 2}}})
	l3, unplaced, err := s3.Linkage(nameBase, nil)
	require.NoError(t, err)
	require.Empty(t, unplaced)
	rev, ok := l3.Links()[0].Joint().(linkage.RevoluteJoint)
	require.True(t, ok)
	requireVec(t, r3.Vec{X: 20, Y: 10}, rev.Center, "declared centre")

	_, _, err = s.Linkage("nope", rep)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = s.Linkage(nameStrap, rep)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestLinkageFixedGroups(t *testing.T) {
	// A part held by a Fixed joint joins its neighbour's link; the ground's group is static, and
	// a Contact or Tie between links is declared, a Keepout is not.
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameBase, Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 20, 20, 0, 10)},
		parts.Part{Name: "plate", Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 20, 20, 10, 5)},
		parts.Part{Name: nameArm, Kind: parts.Manufactured, Body: block(t, doc, 30, 0, 50, 20, 0, 10)},
		parts.Part{Name: "rider", Kind: parts.Manufactured, Body: block(t, doc, 30, 0, 50, 20, 10, 5)})
	mustJoin(t, s,
		parts.Joint{Name: nameLap, Kind: parts.FaceContact, A: "plate", B: nameBase},
		parts.Joint{Name: "slide", Kind: parts.FaceContact, A: nameArm, B: nameBase, Motion: parts.Slides, Direction: r3.Vec{X: 1}},
		parts.Joint{Name: "lap2", Kind: parts.FaceContact, A: "rider", B: nameArm},
		parts.Joint{Name: "touch", Kind: parts.Contact, A: "rider", B: "plate"},
		parts.Joint{Name: "keep", Kind: parts.Keepout, A: "rider", B: nameBase, MinGap: units.Millimeters(1)})
	l, unplaced, err := s.Linkage(nameBase, nil)
	require.NoError(t, err)
	require.Empty(t, unplaced)
	links := l.Links()
	require.Len(t, links, 1)
	require.Len(t, links[0].Bodies(), 2, "the arm and the rider are one link")
	pri, ok := links[0].Joint().(linkage.PrismaticJoint)
	require.True(t, ok)
	requireVec(t, r3.Vec{X: 1}, pri.Dir, "declared direction")
	// The arm and the base share a joint, and so do the rider and the plate; the keepout is not declared.
	require.Len(t, l.JointContacts(), 2)
}
