package parts_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// twoBlocks is a 10 x 10 x 5 base and a same-size block whose near face is gap mm
// away along X.
func twoBlocks(t *testing.T, gap float64, j parts.Joint) *parts.Set {
	t.Helper()
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameBase, Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 10, 10, 0, 5)},
		parts.Part{Name: nameTop, Kind: parts.Manufactured, Body: block(t, doc, 10+gap, 0, 20+gap, 10, 0, 5)})
	j.A, j.B = nameTop, nameBase
	mustJoin(t, s, j)
	return s
}

// notched is an L-shaped plate (a 10 x 10 square less its far corner) and a block
// standing in the notch gap mm from both inner walls. Their boxes overlap whatever
// gap is.
func notched(t *testing.T, gap float64, j parts.Joint) *parts.Set {
	t.Helper()
	doc := decad.New()
	ell, err := decad.Cut(t.Context(), block(t, doc, 0, 0, 10, 10, 0, 5), block(t, doc, 5, 5, 11, 11, -1, 7))
	require.NoError(t, err)
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: "ell", Kind: parts.Manufactured, Body: ell},
		parts.Part{Name: nameBlock, Kind: parts.Manufactured, Body: block(t, doc, 5+gap, 5+gap, 10, 10, 0, 5)})
	if j.Kind != parts.JointUnset {
		j.A, j.B = nameBlock, "ell"
		mustJoin(t, s, j)
	}
	return s
}

// diamondSet is a 10 x 10 block and a block turned 45 degrees about its centre
// (16, 16): their boxes overlap but their nearest points are 3.49 mm apart.
func diamondSet(t *testing.T, j parts.Joint) (*parts.Set, *decad.Document) {
	t.Helper()
	doc := decad.New()
	set := parts.NewSet(doc)
	diamond := block(t, doc, 11, 11, 21, 21, 0, 5)
	turn, err := r3.RotationAround(r3.Vec{X: 16, Y: 16}, r3.Vec{Z: 1}, units.Degrees(45))
	require.NoError(t, err)
	diamond, err = decad.Transform(t.Context(), diamond, turn)
	require.NoError(t, err)
	mustAdd(t, set,
		parts.Part{Name: nameBase, Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 10, 10, 0, 5)},
		parts.Part{Name: "diamond", Kind: parts.Manufactured, Body: diamond})
	if j.Kind != parts.JointUnset {
		j.A, j.B = "diamond", nameBase
		mustJoin(t, set, j)
	}
	return set, doc
}

func TestCheckContact(t *testing.T) {
	// Blocks resting face to face: a witness on the shared face proves the gap within 0.25 mm.
	r := checkOne(t, twoBlocks(t, 0, parts.Joint{Kind: parts.Contact}))
	require.Equal(t, parts.Confirmed, r.Status, r.Reason)
	require.NotNil(t, r.Witness)
	require.InDelta(t, 10.0, r.Witness.X, 1e-6, "the witness lies on the shared face x = 10")
	require.LessOrEqual(t, r.Gap.Value.Base()+r.Gap.Bound.Base(), 0.25, "the gap's upper end")
	require.InDelta(t, 0.0, r.Gap.Value.Base()-r.Gap.Bound.Base(), 1e-9, "the gap's lower end is the boxes'")

	// Blocks 2 mm apart: the boxes alone prove it, with no search.
	r = checkOne(t, twoBlocks(t, 2, parts.Joint{Kind: parts.Contact}))
	require.Equal(t, parts.Contradicted, r.Status)
	require.Equal(t, parts.CauseApart, r.Cause)
	require.Nil(t, r.Witness)
	require.InDelta(t, 2.0, r.Gap.Value.Base()-r.Gap.Bound.Base(), 1e-9)

	// Two cylinders touching along a line, meshed at 0.05 mm, have mesh bounds that
	// together exceed a 0.05 mm contact tolerance: below resolution, so undecided.
	doc := decad.New()
	cyl := parts.NewSet(doc)
	mustAdd(t, cyl,
		parts.Part{Name: "a", Kind: parts.Manufactured, Body: cylinder(t, doc, 5, 5, 0, 10)},
		parts.Part{Name: "b", Kind: parts.Manufactured, Body: cylinder(t, doc, 15, 5, 0, 10)})
	mustJoin(t, cyl, parts.Joint{Kind: parts.Contact, A: "a", B: "b"})
	rep, err := cyl.Check(t.Context(), parts.WithContactTolerance(units.Millimeters(0.05)))
	require.NoError(t, err)
	require.Equal(t, parts.Undecided, rep.Joints[0].Status, rep.Joints[0].Reason)
	require.Equal(t, parts.CauseNoWitness, rep.Joints[0].Cause)
	require.Contains(t, rep.Joints[0].Reason, "below resolution")
	require.True(t, rep.Request.Meshes[0].Verified)
}

func TestCheckKeepout(t *testing.T) {
	two := units.Millimeters(2)
	// 5 mm apart with a 2 mm minimum: the boxes alone confirm it.
	r := checkOne(t, twoBlocks(t, 5, parts.Joint{Kind: parts.Keepout, MinGap: two}))
	require.Equal(t, parts.Confirmed, r.Status, r.Reason)
	require.Nil(t, r.Witness, "no search is needed")
	require.InDelta(t, 5.0, r.Gap.Value.Base()-r.Gap.Bound.Base(), 1e-9)

	// L-shaped parts whose boxes meet but whose nearest points are 1 mm apart: a witness proves
	// them too close.
	r = checkOne(t, notched(t, 1, parts.Joint{Kind: parts.Keepout, MinGap: two}))
	require.Equal(t, parts.Contradicted, r.Status, r.Reason)
	require.Equal(t, parts.CauseTooClose, r.Cause)
	require.NotNil(t, r.Witness)
	require.Less(t, r.Gap.Value.Base()+r.Gap.Bound.Base(), 2.0, "the witness proves the gap below the minimum")
	require.GreaterOrEqual(t, r.Gap.Value.Base()+r.Gap.Bound.Base(), 1.0, "and cannot prove it below the true 1 mm")

	// A block and a diamond whose boxes overlap but whose nearest points are 3.49 mm apart: no
	// sampled point shows them closer than 2 mm, and the boxes cannot prove them farther, so only
	// a pair proof decides it. A Verify report from the caller does.
	set, doc := diamondSet(t, parts.Joint{Kind: parts.Keepout, MinGap: two})
	r = checkOne(t, set)
	require.Equal(t, parts.Undecided, r.Status, r.Reason)
	require.Equal(t, parts.CauseNeedsPairProof, r.Cause)
	require.Nil(t, r.Verified)
	verify, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	rep, err := set.Check(t.Context(), parts.WithVerifyReport(verify))
	require.NoError(t, err)
	require.True(t, rep.Request.VerifyReport)
	require.Equal(t, parts.Confirmed, rep.Joints[0].Status, rep.Joints[0].Reason)
	require.NotNil(t, rep.Joints[0].Verified)
	require.Equal(t, decad.ContactSeparated, *rep.Joints[0].Verified)
}

func TestCheckWitnessSearch(t *testing.T) {
	// Touching blocks with no joint: the search proves the pair within the contact tolerance.
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: "a", Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 10, 10, 0, 5)},
		parts.Part{Name: "touching", Kind: parts.Manufactured, Body: block(t, doc, 10, 0, 20, 10, 0, 5)})
	rep, err := s.Check(t.Context())
	require.NoError(t, err)
	require.Len(t, rep.Unexplained, 1)
	require.Nil(t, rep.Unexplained[0].Near, "without the search a pair carries no Near")
	rep, err = s.Check(t.Context(), parts.WithWitnessSearch())
	require.NoError(t, err)
	require.True(t, rep.Request.WitnessSearch)
	require.Len(t, rep.Unexplained, 1)
	require.NotNil(t, rep.Unexplained[0].Near)
	require.LessOrEqual(t, rep.Unexplained[0].Near.Value.Base()+rep.Unexplained[0].Near.Bound.Base(), 0.25)

	// An L-shaped part and a block 1 mm from its notch walls: their boxes meet, so the pair is
	// listed, but no witness proves them within 0.25 mm.
	rep, err = notched(t, 1, parts.Joint{}).Check(t.Context(), parts.WithWitnessSearch())
	require.NoError(t, err)
	require.Len(t, rep.Unexplained, 1)
	require.Nil(t, rep.Unexplained[0].Near)
}

func TestCheckVerifyReport(t *testing.T) {
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameBase, Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 10, 10, 0, 5)},
		parts.Part{Name: nameTop, Kind: parts.Manufactured, Body: block(t, doc, 10, 0, 20, 10, 0, 5)},
		parts.Part{Name: "inside", Kind: parts.Manufactured, Body: block(t, doc, 2, 2, 8, 8, 1, 3)})
	mustJoin(t, s, parts.Joint{Name: "touch", Kind: parts.Contact, A: nameTop, B: nameBase},
		parts.Joint{Name: "nested", Kind: parts.Contact, A: "inside", B: nameBase})
	verify, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)

	plain, err := s.Check(t.Context())
	require.NoError(t, err)
	withReport, err := s.Check(t.Context(), parts.WithVerifyReport(verify))
	require.NoError(t, err)
	// The report adds each pair's relation and leaves a structural answer it does not contradict.
	require.Equal(t, plain.Joints[0].Status, withReport.Joints[0].Status)
	require.Nil(t, plain.Joints[0].Verified)
	require.Equal(t, decad.ContactTouching, *withReport.Joints[0].Verified)
	// A part sunk into another is a witness-confirmed "contact" to the structural rule; the
	// report proves the overlap and contradicts it.
	require.Equal(t, parts.Confirmed, plain.Joints[1].Status, plain.Joints[1].Reason)
	require.Equal(t, parts.Contradicted, withReport.Joints[1].Status)
	require.Equal(t, parts.CauseInterference, withReport.Joints[1].Cause)
	require.Equal(t, decad.ContactOverlapping, *withReport.Joints[1].Verified)
	for _, p := range withReport.Unexplained {
		require.NotNil(t, p.Verified)
	}

	// The diamond's box overlaps the block's, so the pair is listed; a report that proves them
	// 3.49 mm apart, beyond the contact tolerance, drops it.
	dset, ddoc := diamondSet(t, parts.Joint{})
	dplain, err := dset.Check(t.Context())
	require.NoError(t, err)
	require.Len(t, dplain.Unexplained, 1)
	dverify, err := ddoc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	dreport, err := dset.Check(t.Context(), parts.WithVerifyReport(dverify))
	require.NoError(t, err)
	require.Empty(t, dreport.Unexplained)

	// A report of another document, or one that does not name a body of the Set, is refused.
	other := decad.New()
	block(t, other, 0, 0, 5, 5, 0, 5)
	foreign, err := other.Verify(t.Context())
	require.NoError(t, err)
	_, err = s.Check(t.Context(), parts.WithVerifyReport(foreign))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	older, err := doc.Verify(t.Context())
	require.NoError(t, err)
	later := block(t, doc, 30, 30, 35, 35, 0, 5)
	mustAdd(t, s, parts.Part{Name: "later", Kind: parts.Manufactured, Body: later})
	_, err = s.Check(t.Context(), parts.WithVerifyReport(older))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = s.Check(t.Context(), parts.WithVerifyReport(nil))
	require.ErrorIs(t, err, decad.ErrDegenerate)

	// A declared press fit stays confirmed when the report shows the overlap it expects.
	pinDoc := decad.New()
	pins := pinSet(t, pinDoc, 3, 3.1, 10, parts.Joint{Clearance: units.Millimeters(-0.1)})
	pv, err := pinDoc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	pr, err := pins.Check(t.Context(), parts.WithVerifyReport(pv))
	require.NoError(t, err)
	require.Equal(t, parts.Confirmed, pr.Joints[0].Status, pr.Joints[0].Reason)
}
