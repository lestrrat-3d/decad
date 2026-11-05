package parts_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// pinSet joins a Ø(2r) pin standing at x = cx to a plate bored to boreR.
func pinSet(t *testing.T, doc *decad.Document, boreR, pinR, cx float64, j parts.Joint) *parts.Set {
	t.Helper()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: namePlate, Kind: parts.Manufactured, Body: boredPlateR(t, doc, boreR)},
		parts.Part{Name: namePin, Kind: parts.Manufactured, Body: cylinder(t, doc, cx, pinR, -2, 17)})
	j.Kind, j.A, j.B = parts.PinInBore, namePin, namePlate
	mustJoin(t, s, j)
	return s
}

func checkOne(t *testing.T, s *parts.Set) parts.JointResult {
	t.Helper()
	rep, err := s.Check(t.Context())
	require.NoError(t, err)
	require.Len(t, rep.Joints, len(s.Joints()))
	return rep.Joints[0]
}

func TestCheckPins(t *testing.T) {
	// A Ø6 pin in a Ø6.4 bore: confirmed, measured 0.2, engaged over the plate's 10 mm.
	doc := decad.New()
	s := pinSet(t, doc, 3.2, 3, 10, parts.Joint{})
	r := checkOne(t, s)
	require.Equal(t, parts.Confirmed, r.Status, r.Reason)
	require.Equal(t, parts.CauseNone, r.Cause)
	require.InDelta(t, 0.2, r.Measured.Value.Base(), 1e-9)
	require.InDelta(t, 10.0, r.Engaged.Value.Base(), 1e-9)
	require.Equal(t, parts.AxisSourceFaces, r.AxisFrom)
	requireVec(t, r3.Vec{X: 10, Y: 10, Z: 5}, r.Axis.Point, "axis point")
	require.InDelta(t, 1.0, r.Axis.Dir.Z*r.Axis.Dir.Z, 1e-9)
	require.NotEmpty(t, r.FacesA)
	require.NotEmpty(t, r.FacesB)

	rep, err := s.Check(t.Context())
	require.NoError(t, err)
	require.Len(t, rep.Request.Meshes, 2)
	require.True(t, rep.Request.Meshes[0].Verified)
	require.InDelta(t, 0.25, rep.Request.ContactTolerance.Base(), 1e-12)
	require.Empty(t, rep.Unexplained)

	for name, tc := range map[string]struct {
		joint  parts.Joint
		boreR  float64
		pinR   float64
		cx     float64
		status parts.JointStatus
		cause  parts.Cause
	}{
		"declared clearance agrees":    {parts.Joint{Clearance: units.Millimeters(0.2)}, 3.2, 3, 10, parts.Confirmed, parts.CauseNone},
		"declared clearance disagrees": {parts.Joint{Clearance: units.Millimeters(0.3)}, 3.2, 3, 10, parts.Contradicted, parts.CauseClearance},
		"declared axis agrees": {parts.Joint{Axis: &parts.Axis{Point: r3.Vec{X: 10, Y: 10}, Dir: r3.Vec{Z: 1}}}, 3.2, 3, 10,
			parts.Confirmed, parts.CauseNone},
		"declared axis 1 mm off": {parts.Joint{Axis: &parts.Axis{Point: r3.Vec{X: 11, Y: 10}, Dir: r3.Vec{Z: 1}}}, 3.2, 3, 10,
			parts.Contradicted, parts.CauseAxis},
		"declared axis tilted": {parts.Joint{Axis: &parts.Axis{Point: r3.Vec{X: 10, Y: 10}, Dir: r3.Vec{X: 0.1, Z: 1}}}, 3.2, 3, 10,
			parts.Contradicted, parts.CauseAxis},
		"press fit declared":       {parts.Joint{Clearance: units.Millimeters(-0.05)}, 3, 3.05, 10, parts.Confirmed, parts.CauseNone},
		"press fit undeclared":     {parts.Joint{}, 3, 3.1, 10, parts.Contradicted, parts.CauseInterference},
		"press fit wrong declared": {parts.Joint{Clearance: units.Millimeters(0.1)}, 3, 3.1, 10, parts.Contradicted, parts.CauseClearance},
		"pin 1 mm off the axis":    {parts.Joint{}, 3.2, 3, 11, parts.Contradicted, parts.CauseNoMatch},
		"pin 5 mm clear":           {parts.Joint{}, 3.2, 3, 28, parts.Contradicted, parts.CauseApart},
	} {
		got := checkOne(t, pinSet(t, decad.New(), tc.boreR, tc.pinR, tc.cx, tc.joint))
		require.Equal(t, tc.status, got.Status, name+": "+got.Reason)
		require.Equal(t, tc.cause, got.Cause, name+": "+got.Reason)
		if tc.cause != parts.CauseNone {
			require.NotEmpty(t, got.Reason, name)
		}
	}

	// The reason of a missing match names both radii and the distance between the axes.
	off := checkOne(t, pinSet(t, decad.New(), 3.2, 3, 11, parts.Joint{}))
	require.Contains(t, off.Reason, "pin radius 3 mm")
	require.Contains(t, off.Reason, "bore radius 3.2 mm")
	require.Contains(t, off.Reason, "1 mm off axis")
	// A pin 5 mm clear of the plate is apart: the gap is the boxes' (28 - 3 - 20).
	apart := checkOne(t, pinSet(t, decad.New(), 3.2, 3, 28, parts.Joint{}))
	require.Contains(t, apart.Reason, "at least 5 mm apart")

	// A declared axis is reported even when the faces cannot confirm.
	far := checkOne(t, pinSet(t, decad.New(), 3.2, 3, 28, parts.Joint{Axis: &parts.Axis{Point: r3.Vec{X: 28, Y: 10}, Dir: r3.Vec{Z: 2}}}))
	require.Equal(t, parts.AxisSourceDeclared, far.AxisFrom)
	require.InDelta(t, 1.0, far.Axis.Dir.Z, 1e-12)
}

func TestCheckPinInSlot(t *testing.T) {
	build := func(pinR, cx float64) parts.JointResult {
		doc := decad.New()
		blk, err := decad.Cut(t.Context(), block(t, doc, 0, 0, 30, 20, 0, 10), block(t, doc, 12.8, 5, 17.2, 15, -1, 12))
		require.NoError(t, err)
		s := parts.NewSet(doc)
		mustAdd(t, s, parts.Part{Name: nameBlock, Kind: parts.Manufactured, Body: blk},
			parts.Part{Name: namePin, Kind: parts.Manufactured, Body: cylinder(t, doc, cx, pinR, -2, 15)})
		mustJoin(t, s, parts.Joint{Kind: parts.PinInSlot, A: namePin, B: nameBlock, Motion: parts.Slides})
		return checkOne(t, s)
	}
	ok := build(2, 15)
	require.Equal(t, parts.Confirmed, ok.Status, ok.Reason)
	require.InDelta(t, 0.4, ok.Measured.Value.Base(), 1e-9)
	require.InDelta(t, 10.0, ok.Engaged.Value.Base(), 1e-9)
	require.Len(t, ok.FacesB, 2)
	// A Ø5 pin does not fit the 4.4 mm slot.
	big := build(2.5, 15)
	require.Equal(t, parts.Contradicted, big.Status)
	require.Equal(t, parts.CauseInterference, big.Cause)
	// A pin off to the side stands between no pair of walls.
	side := build(2, 25)
	require.Equal(t, parts.Contradicted, side.Status)
	require.Equal(t, parts.CauseNoMatch, side.Cause)
	require.Contains(t, side.Reason, "no pin between parallel walls")
}

func TestCheckUnread(t *testing.T) {
	// A bore whose faces read Faceted gives no radius to compare: the joint is undecided,
	// never contradicted, and the declared axis is reported.
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: namePlate, Kind: parts.Manufactured, Body: facetedCopy(t, doc, boredPlate(t, doc))},
		parts.Part{Name: namePin, Kind: parts.Manufactured, Body: cylinder(t, doc, 10, 3, -2, 17)})
	mustJoin(t, s, parts.Joint{Kind: parts.PinInBore, A: namePin, B: namePlate,
		Axis: &parts.Axis{Point: r3.Vec{X: 10, Y: 10}, Dir: r3.Vec{Z: 1}}})
	r := checkOne(t, s)
	require.Equal(t, parts.Undecided, r.Status, r.Reason)
	require.Equal(t, parts.CauseUnreadFace, r.Cause)
	require.Equal(t, parts.AxisSourceDeclared, r.AxisFrom)
	require.Nil(t, r.Measured)
}

func TestCheckFaceContact(t *testing.T) {
	build := func(z float64) parts.JointResult {
		doc := decad.New()
		s := parts.NewSet(doc)
		mustAdd(t, s, parts.Part{Name: nameBase, Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 10, 10, 0, 5)},
			parts.Part{Name: nameTop, Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 10, 10, z, 5)})
		mustJoin(t, s, parts.Joint{Kind: parts.FaceContact, A: nameTop, B: nameBase})
		return checkOne(t, s)
	}
	ok := build(5)
	require.Equal(t, parts.Confirmed, ok.Status, ok.Reason)
	require.InDelta(t, 0.0, ok.Measured.Value.Base(), 1e-9)
	require.NotNil(t, ok.Witness)
	require.InDelta(t, 5.0, ok.Witness.Z, 1e-9, "the witness lies on the shared plane")
	require.Len(t, ok.FacesA, 1)
	require.Len(t, ok.FacesB, 1)

	// 0.5 mm apart is within the 1 mm the faces are searched over but not the 0.05 mm fit.
	apart := build(5.5)
	require.Equal(t, parts.Contradicted, apart.Status)
	require.Equal(t, parts.CauseApart, apart.Cause)
	// 3 mm apart is past MaxFit: the boxes alone prove it.
	require.Equal(t, parts.CauseApart, build(8).Cause)

	// An L-shaped plate and a block over its notch share a plane and their vertex boxes overlap,
	// but their faces do not: no witness, so the joint is undecided and never confirmed.
	doc := decad.New()
	ell, err := decad.Cut(t.Context(), block(t, doc, 0, 0, 10, 10, 0, 5), block(t, doc, 5, 5, 11, 11, -1, 7))
	require.NoError(t, err)
	s := parts.NewSet(doc)
	mustAdd(t, s, parts.Part{Name: "ell", Kind: parts.Manufactured, Body: ell},
		parts.Part{Name: nameTop, Kind: parts.Manufactured, Body: block(t, doc, 6, 6, 10, 10, 5, 5)})
	mustJoin(t, s, parts.Joint{Kind: parts.FaceContact, A: nameTop, B: "ell"})
	r := checkOne(t, s)
	require.Equal(t, parts.Undecided, r.Status, r.Reason)
	require.Equal(t, parts.CauseNoWitness, r.Cause)
	require.Nil(t, r.Witness)

	// Parts side by side share no opposed planes.
	doc = decad.New()
	side := parts.NewSet(doc)
	mustAdd(t, side, parts.Part{Name: "a", Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 10, 10, 0, 5)},
		parts.Part{Name: "b", Kind: parts.Manufactured, Body: block(t, doc, 10, 0, 20, 10, 0, 5)})
	mustJoin(t, side, parts.Joint{Kind: parts.FaceContact, A: "a", B: "b"})
	// They do share the plane x = 10 face to face.
	require.Equal(t, parts.Confirmed, checkOne(t, side).Status)
}

func TestCheckUnexplained(t *testing.T) {
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: "a", Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 10, 10, 0, 5)},
		parts.Part{Name: "touching", Kind: parts.Manufactured, Body: block(t, doc, 10, 0, 20, 10, 0, 5)},
		parts.Part{Name: "near", Kind: parts.Manufactured, Body: block(t, doc, 0, 11, 10, 20, 0, 5)},
		parts.Part{Name: nameStrap, Kind: parts.Purchased, Quantity: 1})
	rep, err := s.Check(t.Context())
	require.NoError(t, err)
	// The touching pair is listed with where the grown boxes meet (0.25 mm each way); the pair
	// 1 mm apart is not, and a body-less part is never listed.
	require.Len(t, rep.Unexplained, 1)
	f := rep.Unexplained[0]
	require.Equal(t, "a", f.A)
	require.Equal(t, "touching", f.B)
	requireVec(t, r3.Vec{X: 9.75, Y: -0.25, Z: -0.25}, f.Overlap.Min, "overlap min")
	requireVec(t, r3.Vec{X: 10.25, Y: 10.25, Z: 5.25}, f.Overlap.Max, "overlap max")

	// A declared joint explains the pair, whichever way round it is stated.
	mustJoin(t, s, parts.Joint{Kind: parts.FaceContact, A: "touching", B: "a"})
	rep, err = s.Check(t.Context())
	require.NoError(t, err)
	require.Empty(t, rep.Unexplained)
	// A wider contact tolerance lists the nearby pair too.
	rep, err = s.Check(t.Context(), parts.WithContactTolerance(units.Millimeters(0.6)))
	require.NoError(t, err)
	require.Len(t, rep.Unexplained, 2)
}

func TestCheckLeavesTheDocumentAlone(t *testing.T) {
	doc := decad.New()
	s := pinSet(t, doc, 3.2, 3, 10, parts.Joint{})
	bodies, ops := doc.Bodies(), len(doc.OperationHistory().Operations())
	_, err := s.Check(t.Context())
	require.NoError(t, err)
	require.Equal(t, bodies, doc.Bodies())
	require.Len(t, doc.OperationHistory().Operations(), ops)

	// A tie is never checked. The pin is 0.2 mm inside its bore, so no sampled point proves a
	// contact within 0.25 mm (the witness sums read 0.29 mm) and a 1 mm keepout is violated.
	mustAdd(t, s, parts.Part{Name: nameStrap, Kind: parts.Purchased, Quantity: 1})
	mustJoin(t, s, parts.Joint{Name: "tie", Kind: parts.Tie, A: nameStrap, B: namePlate},
		parts.Joint{Name: "snap", Kind: parts.Contact, A: namePin, B: namePlate},
		parts.Joint{Name: "gap", Kind: parts.Keepout, A: namePin, B: namePlate, MinGap: units.Millimeters(1)})
	rep, err := s.Check(t.Context())
	require.NoError(t, err)
	require.Equal(t, parts.Confirmed, rep.Joints[0].Status)
	require.Equal(t, parts.NotChecked, rep.Joints[1].Status)
	require.Equal(t, parts.Undecided, rep.Joints[2].Status)
	require.Equal(t, parts.CauseNoWitness, rep.Joints[2].Cause)
	require.Equal(t, parts.Contradicted, rep.Joints[3].Status)
	require.Equal(t, parts.CauseTooClose, rep.Joints[3].Cause)

	retired := decad.New()
	rs := pinSet(t, retired, 3.2, 3, 10, parts.Joint{})
	_, err = decad.Union(t.Context(), rs.Parts()[1].Body, block(t, retired, 9, 9, 12, 12, 0, 3))
	require.NoError(t, err)
	_, err = rs.Check(t.Context())
	require.ErrorIs(t, err, decad.ErrRetiredBody)
	_, err = rs.Check(t.Context(), nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
