package parts_test

import (
	"bytes"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

const (
	nameBottom  = "housing-bottom"
	nameTopHalf = "housing-top"
	nameLever   = "lever"
)

// housing is two 40 x 20 x 10 halves split at z = 0 holding a 12 x 6 x 8 lever in a pocket,
// with one Ø4 pin along Z through both halves and the lever.
func housing(t *testing.T) *parts.Set {
	t.Helper()
	doc := decad.New()
	cut := func(b *decad.Body, tool *decad.Body) *decad.Body {
		out, err := decad.Cut(t.Context(), b, tool)
		require.NoError(t, err)
		return out
	}
	bore := func(b *decad.Body, z0, h float64) *decad.Body { return cut(b, cylinder(t, doc, 20, 2.1, z0, h)) }
	bottom := bore(cut(block(t, doc, 0, 0, 40, 20, -10, 10), block(t, doc, 10, 4, 30, 16, -6, 7)), -11, 12)
	top := bore(cut(block(t, doc, 0, 0, 40, 20, 0, 10), block(t, doc, 10, 4, 30, 16, -1, 7)), -1, 12)
	lever := bore(block(t, doc, 14, 7, 26, 13, -4, 8), -5, 10)
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameBottom, Kind: parts.Manufactured, Body: bottom},
		parts.Part{Name: nameTopHalf, Kind: parts.Manufactured, Body: top},
		parts.Part{Name: nameLever, Kind: parts.Manufactured, Body: lever},
		parts.Part{Name: namePin, Kind: parts.Manufactured, Body: cylinder(t, doc, 20, 2, -12, 24)})
	mustJoin(t, s,
		parts.Joint{Kind: parts.FaceContact, A: nameTopHalf, B: nameBottom},
		parts.Joint{Kind: parts.PinInBore, A: namePin, B: nameTopHalf},
		parts.Joint{Kind: parts.PinInBore, A: namePin, B: nameBottom},
		parts.Joint{Kind: parts.PinInBore, A: namePin, B: nameLever, Motion: parts.Turns})
	return s
}

func threeStages() parts.ExplodeOption {
	return parts.WithExplodeStages(
		parts.Stage{Name: "shell", Parts: []string{nameTopHalf}},
		parts.Stage{Name: "mechanism", Parts: []string{nameLever}, Clear: parts.ClearAll, Direction: r3.Vec{Z: 1}},
		parts.Stage{Name: "pins", Parts: []string{namePin}, Clear: parts.ClearAll, Gap: units.Millimeters(10)})
}

func TestExplodeStagesDefault(t *testing.T) {
	// No stages: P1's offsets, with every new field filled in.
	s := lapSet(t)
	ex, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBase))
	require.NoError(t, err)
	require.Equal(t, []string{""}, ex.Stages)
	require.InDelta(t, 1.0, ex.Scale.Base(), 0)
	// The pins stand 4 mm apart, closer than their 10 mm gap, so they are short; nothing else is.
	require.Equal(t, []string{namePin1, namePin2}, ex.Short)
	top := partByName(t, ex, nameTop)
	requireVec(t, r3.Vec{Z: 10}, top.Offset, "top")
	require.Equal(t, "", top.Stage)
	require.InDelta(t, 10.0, top.Gap.Base(), 1e-12)
	require.InDelta(t, 10.0, top.Moved.Base(), 1e-12)
	require.InDelta(t, 0.0, top.Travel.Base(), 1e-12)
	requireVec(t, r3.Vec{Z: 1}, top.Dir, "top direction")
	require.Equal(t, parts.PathClear, top.Path)
	pin := partByName(t, ex, namePin1)
	requireVec(t, r3.Vec{Z: -22}, pin.Offset, "pin1")
	require.InDelta(t, 22.0, pin.Moved.Base(), 1e-12)
	require.InDelta(t, 12.0, pin.Travel.Base(), 1e-12)
	// The ground did not move: no gap, no direction, no path.
	base := partByName(t, ex, nameBase)
	require.Zero(t, base.Gap.Base())
	require.Equal(t, parts.PathNone, base.Path)
	require.Equal(t, r3.Vec{}, base.Dir)
	for _, l := range ex.Leaders {
		require.GreaterOrEqual(t, l.Separation.Base(), 0.0)
	}
}

func TestExplodeStagesHousing(t *testing.T) {
	s := housing(t)
	flat, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBottom))
	require.NoError(t, err)
	require.Equal(t, [][2]string{{nameBottom, nameLever}}, flat.Overlaps, "one stage leaves the lever in the bottom half's box")
	requireVec(t, r3.Vec{Z: -6}, partByName(t, flat, nameLever).Offset, "one-stage lever")

	ex, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBottom), threeStages())
	require.NoError(t, err)
	require.Equal(t, []string{"shell", "mechanism", "pins"}, ex.Stages)
	top, lever, pin := partByName(t, ex, nameTopHalf), partByName(t, ex, nameLever), partByName(t, ex, namePin)
	requireVec(t, r3.Vec{Z: 10}, top.Offset, "top")
	requireVec(t, r3.Vec{Z: 34}, lever.Offset, "lever")
	requireVec(t, r3.Vec{Z: -32}, pin.Offset, "pin")
	// The lever rose until its box was 10 mm above the lifted top half: 24 mm to touch, 34 with the gap.
	require.InDelta(t, 24.0, lever.Travel.Base(), 1e-9)
	require.InDelta(t, 34.0, lever.Moved.Base(), 1e-9)
	require.Equal(t, "mechanism", lever.Stage)
	require.Equal(t, parts.StageDirection, lever.Pull)
	require.Equal(t, parts.PathEnclosed, lever.Path)
	require.Equal(t, nameBottom, lever.Blocker)
	require.Equal(t, parts.PathEnclosed, top.Path)
	require.Equal(t, nameLever, top.Blocker)
	require.Equal(t, parts.PathClear, pin.Path)
	require.Equal(t, parts.AxisFromFaces, pin.Pull)
	require.Empty(t, ex.Overlaps)
	require.Empty(t, ex.Short)
	// The lever ends 10 mm from the top half it is not joined to.
	require.InDelta(t, 10.0, lever.Clearance.Base(), 1e-9)
}

func TestExplodeClearAllIntervals(t *testing.T) {
	// A 4 mm block asked for a 5 mm gap, sent along +X past a row of three standing blocks with
	// 3, 8 and 30 mm between them: it can only stop in the 30 mm space, 5 mm clear of the third.
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameBase, Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 10, 10, 0, 5)},
		parts.Part{Name: "row-1", Kind: parts.Manufactured, Body: block(t, doc, 13, 0, 23, 10, 0, 5)},
		parts.Part{Name: "row-2", Kind: parts.Manufactured, Body: block(t, doc, 31, 0, 41, 10, 0, 5)},
		parts.Part{Name: "row-3", Kind: parts.Manufactured, Body: block(t, doc, 71, 0, 81, 10, 0, 5)},
		parts.Part{Name: nameBlock, Kind: parts.Manufactured, Body: block(t, doc, 3, 3, 7, 7, 0, 5)})
	mustJoin(t, s, parts.Joint{Kind: parts.Contact, A: nameBlock, B: nameBase})
	ex, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBase), parts.WithExplodeStages(
		parts.Stage{Name: "out", Parts: []string{nameBlock}, Clear: parts.ClearAll, Direction: r3.Vec{X: 1}, Gap: units.Millimeters(5)}))
	require.NoError(t, err)
	// Its box starts at x 3 to 7. With the standing blocks grown by 5 mm the intervals of its travel are
	// (-12, 12), (1, 25), (19, 43) and (59, 83): the first three merge, so it stops at 43, its box at
	// x 46 to 50, 5 mm past the second row block and 21 mm short of the third.
	b := partByName(t, ex, nameBlock)
	requireVec(t, r3.Vec{X: 43}, b.Offset, "block offset")
	require.InDelta(t, 38.0, b.Travel.Base(), 1e-9, "the move less the gap")
	require.InDelta(t, 5.0, b.Gap.Base(), 1e-12)
	require.Equal(t, parts.StageDirection, b.Pull)
}

func TestExplodeTogether(t *testing.T) {
	// Two parts side by side on a base, lifted as one clear of a cover overhanging both; a later stage
	// takes them apart from where the group left them.
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameBase, Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 40, 10, 0, 5)},
		parts.Part{Name: nameLeft, Kind: parts.Manufactured, Body: block(t, doc, 2, 2, 8, 8, 5, 4)},
		parts.Part{Name: nameRight, Kind: parts.Manufactured, Body: block(t, doc, 12, 2, 18, 8, 5, 4)},
		parts.Part{Name: "cover", Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 40, 10, 9, 3)})
	mustJoin(t, s,
		parts.Joint{Kind: parts.Contact, A: nameLeft, B: nameBase, Pull: r3.Vec{Z: 1}},
		parts.Joint{Kind: parts.Contact, A: nameRight, B: nameBase, Pull: r3.Vec{Z: 1}},
		parts.Joint{Kind: parts.Contact, A: "cover", B: nameBase, Pull: r3.Vec{Z: 1}})
	run := func(stages ...parts.Stage) *parts.Exploded {
		ex, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBase), parts.WithExplodeStages(stages...))
		require.NoError(t, err)
		return ex
	}
	ex := run(parts.Stage{Name: "cover", Parts: []string{"cover"}, Gap: units.Millimeters(20)},
		parts.Stage{Name: "group", Parts: []string{nameLeft, nameRight}, Together: true, Clear: parts.ClearAll,
			Direction: r3.Vec{Z: 1}, Gap: units.Millimeters(3)})
	left, right := partByName(t, ex, nameLeft), partByName(t, ex, nameRight)
	require.Equal(t, left.Offset, right.Offset, "members move by one offset")
	// The cover rose 20 and now spans z 29 to 32, far above the group (z 5 to 9). The base's box (z 0
	// to 5) grown by 3 reaches z 8, so the group rises 3 to keep that gap: one offset for both.
	require.InDelta(t, 0.0, left.Offset.X, 0)
	require.InDelta(t, 3.0, left.Offset.Z, 1e-9)

	// A later stage takes them apart from there.
	ex = run(parts.Stage{Name: "group", Parts: []string{nameLeft, nameRight}, Together: true, Direction: r3.Vec{Z: 1}, Gap: units.Millimeters(7)},
		parts.Stage{Name: "split", Parts: []string{nameRight}, Direction: r3.Vec{X: 1}, Gap: units.Millimeters(5)})
	left, right = partByName(t, ex, nameLeft), partByName(t, ex, nameRight)
	requireVec(t, r3.Vec{Z: 7}, left.Offset, "left stays where the group left it")
	// right clears the base's box (x to 40) along +X from where the group left it, 28 mm, plus 5.
	requireVec(t, r3.Vec{X: 33, Z: 7}, right.Offset, "right continues from there")
	require.Equal(t, "split", right.Stage)
	require.Equal(t, "group", left.Stage)
	// The leaders follow the final offsets.
	for _, l := range ex.Leaders {
		if l.Joint == "right~base" {
			require.InDelta(t, 33.0, l.From.X-l.To.X, 1e-9)
		}
	}
}

func TestExplodeStuck(t *testing.T) {
	// A standing slab 3,000,000 mm long that a block sent along it can never leave.
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameBase, Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 10, 10, 0, 5)},
		parts.Part{Name: "slab", Kind: parts.Manufactured, Body: block(t, doc, -1e6, 1, 2e6, 9, 6, 5)},
		parts.Part{Name: nameBlock, Kind: parts.Manufactured, Body: block(t, doc, 2, 2, 8, 8, 5, 1)})
	mustJoin(t, s, parts.Joint{Kind: parts.Contact, A: nameBlock, B: nameBase, Pull: r3.Vec{Z: 1}})
	ex, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBase), parts.WithExplodeStages(
		parts.Stage{Name: "out", Parts: []string{nameBlock}, Clear: parts.ClearAll, Direction: r3.Vec{X: 1}}))
	require.NoError(t, err)
	b := partByName(t, ex, nameBlock)
	require.Equal(t, parts.PathStuck, b.Path)
	requireVec(t, r3.Vec{}, b.Offset, "a stuck part stays at its start")
}

func TestExplodePartGapAndScale(t *testing.T) {
	s := housing(t)
	base, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBottom), threeStages())
	require.NoError(t, err)
	// A part gap changes one part's offset only: the top half rises 25 instead of 10, so the
	// lever, which clears the lifted top half, rises 15 more too; the pin is unmoved.
	gapped, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBottom), threeStages(),
		parts.WithPartGap(nameTopHalf, units.Millimeters(25)))
	require.NoError(t, err)
	requireVec(t, r3.Vec{Z: 25}, partByName(t, gapped, nameTopHalf).Offset, "top")
	require.InDelta(t, 25.0, partByName(t, gapped, nameTopHalf).Gap.Base(), 1e-12)
	require.InDelta(t, partByName(t, base, namePin).Offset.Z-0, partByName(t, gapped, namePin).Offset.Z+0, 40,
		"the pin's offset is not asked about")

	// Scale 2 doubles every offset, and clearance and the short list are read after it.
	scaled, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBottom), threeStages(), parts.WithExplodeScale(units.Scalar(2)))
	require.NoError(t, err)
	require.InDelta(t, 2.0, scaled.Scale.Base(), 0)
	for _, name := range []string{nameTopHalf, nameLever, namePin} {
		b, sc := partByName(t, base, name), partByName(t, scaled, name)
		requireVec(t, b.Offset.Scale(2), sc.Offset, name)
		require.InDelta(t, 2*b.Moved.Base(), sc.Moved.Base(), 1e-9, name)
		require.InDelta(t, b.Travel.Base(), sc.Travel.Base(), 1e-9, name+" travel is not scaled")
	}
	require.InDelta(t, 34.0, partByName(t, scaled, nameLever).Clearance.Base(), 1e-9, "the top half at z 20 to 30, the lever at 64 to 72")

	// A scale below 1 closes the gaps the stages left, and the parts that fall short are listed.
	tight, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBottom), threeStages(), parts.WithExplodeScale(units.Scalar(0.25)))
	require.NoError(t, err)
	require.Contains(t, tight.Short, nameTopHalf)
	require.Contains(t, tight.Short, nameLever)
}

func TestExplodeStageRefusals(t *testing.T) {
	s := housing(t)
	explode := func(opts ...parts.ExplodeOption) error {
		_, err := s.Explode(t.Context(), append([]parts.ExplodeOption{parts.WithExplodeGround(nameBottom)}, opts...)...)
		return err
	}
	st := func(name string, members ...string) parts.Stage { return parts.Stage{Name: name, Parts: members} }
	for name, tc := range map[string]struct {
		opt  parts.ExplodeOption
		want error
	}{
		"empty name":       {parts.WithExplodeStages(st("", nameLever)), decad.ErrDegenerate},
		"repeated name":    {parts.WithExplodeStages(st("a", nameLever), st("a", namePin)), decad.ErrDegenerate},
		"no members":       {parts.WithExplodeStages(st("a")), decad.ErrDegenerate},
		"unknown part":     {parts.WithExplodeStages(st("a", "nope")), decad.ErrDegenerate},
		"unknown group":    {parts.WithExplodeStages(parts.Stage{Name: "a", Groups: []string{"nope"}}), decad.ErrDegenerate},
		"part twice":       {parts.WithExplodeStages(st("a", nameLever, nameLever)), decad.ErrDegenerate},
		"ground, no way":   {parts.WithExplodeStages(st("a", nameBottom)), decad.ErrDegenerate},
		"bad direction":    {parts.WithExplodeStages(parts.Stage{Name: "a", Parts: []string{nameLever}, Direction: r3.Vec{X: math.NaN()}}), decad.ErrNotFinite},
		"gap kind":         {parts.WithExplodeStages(parts.Stage{Name: "a", Parts: []string{nameLever}, Gap: units.Degrees(1)}), decad.ErrUnitKind},
		"gap negative":     {parts.WithExplodeStages(parts.Stage{Name: "a", Parts: []string{nameLever}, Gap: units.Millimeters(-1)}), decad.ErrNegativeMagnitude},
		"part gap kind":    {parts.WithPartGap(nameLever, units.Degrees(1)), decad.ErrUnitKind},
		"part gap unknown": {parts.WithPartGap("nope", units.Millimeters(1)), decad.ErrDegenerate},
		"scale kind":       {parts.WithExplodeScale(units.Millimeters(2)), decad.ErrUnitKind},
		"scale zero":       {parts.WithExplodeScale(units.Scalar(0)), decad.ErrDegenerate},
		"scale negative":   {parts.WithExplodeScale(units.Scalar(-1)), decad.ErrNegativeMagnitude},
		"scale nan":        {parts.WithExplodeScale(units.Scalar(math.NaN())), decad.ErrNotFinite},
	} {
		require.ErrorIs(t, explode(tc.opt), tc.want, name)
	}
	// Two stages naming one part through a part and a group.
	doc := decad.New()
	rig, err := doc.Root().NewChild("rig")
	require.NoError(t, err)
	g := parts.NewSet(doc)
	a := block(t, doc, 0, 0, 10, 10, 0, 5)
	b := block(t, doc, 20, 0, 30, 10, 0, 5)
	require.NoError(t, doc.MoveBody(b, rig))
	mustAdd(t, g, parts.Part{Name: "a", Kind: parts.Manufactured, Body: a}, parts.Part{Name: "b", Kind: parts.Manufactured, Body: b})
	mustJoin(t, g, parts.Joint{Kind: parts.Contact, A: "b", B: "a", Pull: r3.Vec{X: 1}})
	_, err = g.Explode(t.Context(), parts.WithExplodeGround("a"), parts.WithExplodeStages(
		parts.Stage{Name: "s", Parts: []string{"b"}, Groups: []string{"rig"}}))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	ex, err := g.Explode(t.Context(), parts.WithExplodeGround("a"), parts.WithExplodeStages(
		parts.Stage{Name: "s", Groups: []string{"rig"}, Gap: units.Millimeters(3)}))
	require.NoError(t, err)
	require.Equal(t, "s", partByName(t, ex, "b").Stage)
	requireVec(t, r3.Vec{X: 3}, partByName(t, ex, "b").Offset, "b starts clear of a: only the gap")
}

func TestExplodeHidden(t *testing.T) {
	s := housing(t)
	flat, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBottom))
	require.NoError(t, err)
	// Looking down -Z, the top half sits over the lever and covers all of its projected box.
	hidden, err := flat.Hidden(r3.Vec{Z: -1})
	require.NoError(t, err)
	var covers float64
	for _, h := range hidden {
		if h.Back == nameLever && h.Front == nameTopHalf {
			covers = h.Covered.Base()
			require.Equal(t, units.Dimensionless, h.Covered.Kind())
		}
	}
	require.InDelta(t, 1.0, covers, 1e-12)

	ex, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBottom), threeStages())
	require.NoError(t, err)
	hidden, err = ex.Hidden(r3.Vec{Z: -1})
	require.NoError(t, err)
	for _, h := range hidden {
		if h.Back == nameLever {
			require.Less(t, h.Covered.Base(), 1.0, "after staging nothing covers the lever whole")
			require.NotEqual(t, nameTopHalf, h.Front)
		}
	}
	// Looking along Y, the stack is side by side: the lever is not hidden by the top half.
	_, err = ex.Hidden(r3.Vec{})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = ex.Hidden(r3.Vec{X: math.Inf(1)})
	require.ErrorIs(t, err, decad.ErrNotFinite)
}

func TestExplodeDeterminism(t *testing.T) {
	s := housing(t)
	threeMF := func() []byte {
		ex, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBottom), threeStages())
		require.NoError(t, err)
		var items []export.ThreeMFItem
		for _, p := range ex.Parts {
			part, err := s.Part(p.Name)
			require.NoError(t, err)
			items = append(items, export.ThreeMFItem{Name: p.Name, Body: part.Body, Placement: p.Placement})
		}
		var out bytes.Buffer
		require.NoError(t, export.ThreeMFItems(t.Context(), &out, items, units.Millimeters(0.1)))
		return out.Bytes()
	}
	first := threeMF()
	require.Equal(t, first, threeMF())
	a, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBottom), threeStages())
	require.NoError(t, err)
	b, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBottom), threeStages())
	require.NoError(t, err)
	require.Equal(t, a.Parts, b.Parts)
	require.Equal(t, a.Leaders, b.Leaders)
}

func TestExplodeJointPull(t *testing.T) {
	// A contact has no direction of its own: its Pull replaces the line between box centres.
	doc := decad.New()
	build := func(pull r3.Vec) *parts.Exploded {
		s := parts.NewSet(doc)
		mustAdd(t, s,
			parts.Part{Name: "a", Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 10, 10, 0, 5)},
			parts.Part{Name: "b", Kind: parts.Manufactured, Body: block(t, doc, 12, 0, 22, 10, 0, 5)})
		mustJoin(t, s, parts.Joint{Kind: parts.Contact, A: "b", B: "a", Pull: pull})
		ex, err := s.Explode(t.Context(), parts.WithExplodeGround("a"))
		require.NoError(t, err)
		return ex
	}
	plain := build(r3.Vec{})
	require.Equal(t, parts.BoxCentres, partByName(t, plain, "b").Pull)
	pulled := build(r3.Vec{Y: 5})
	b := partByName(t, pulled, "b")
	require.Equal(t, parts.PullDeclared, b.Pull)
	requireVec(t, r3.Vec{Y: 1}, b.Dir, "direction (its length does not matter)")
	require.InDelta(t, 20.0, b.Offset.Y, 1e-9, "10 mm to clear a in Y, plus the 10 mm gap")

	// On a pin, Pull picks the sense E3 would not: the pin goes down, though up is the shorter way.
	s := pinSet(t, decad.New(), 3.2, 3, 10, parts.Joint{})
	ex, err := s.Explode(t.Context(), parts.WithExplodeGround(namePlate))
	require.NoError(t, err)
	require.Greater(t, partByName(t, ex, namePin).Offset.Z, 0.0)
	down := pinSet(t, decad.New(), 3.2, 3, 10, parts.Joint{Pull: r3.Vec{Z: -2}})
	ex, err = down.Explode(t.Context(), parts.WithExplodeGround(namePlate))
	require.NoError(t, err)
	p := partByName(t, ex, namePin)
	require.Less(t, p.Offset.Z, 0.0)
	require.Equal(t, parts.PullDeclared, p.Pull)
	// A pull at right angles to the pin's axis names no sense; a non-finite one is refused at Join.
	across := pinSet(t, decad.New(), 3.2, 3, 10, parts.Joint{Pull: r3.Vec{X: 1}})
	_, err = across.Explode(t.Context(), parts.WithExplodeGround(namePlate))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "pin~plate")
	doc2 := decad.New()
	bad := parts.NewSet(doc2)
	mustAdd(t, bad, parts.Part{Name: "a", Kind: parts.Manufactured, Body: block(t, doc2, 0, 0, 5, 5, 0, 5)},
		parts.Part{Name: "b", Kind: parts.Manufactured, Body: block(t, doc2, 6, 0, 11, 5, 0, 5)})
	require.ErrorIs(t, bad.Join(parts.Joint{Kind: parts.Contact, A: "b", B: "a", Pull: r3.Vec{X: math.NaN()}}), decad.ErrNotFinite)
}

func TestExplodeGroundStage(t *testing.T) {
	// The ground in a stage with a direction moves by its gap, so two halves open symmetrically.
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: nameLeft, Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 10, 10, 0, 5)},
		parts.Part{Name: nameRight, Kind: parts.Manufactured, Body: block(t, doc, 10, 0, 20, 10, 0, 5)})
	mustJoin(t, s, parts.Joint{Kind: parts.Contact, A: nameRight, B: nameLeft, Pull: r3.Vec{X: 1}})
	ex, err := s.Explode(t.Context(), parts.WithExplodeGround(nameLeft), parts.WithExplodeStages(
		parts.Stage{Name: "open", Parts: []string{nameLeft}, Direction: r3.Vec{X: -1}, Gap: units.Millimeters(30)},
		parts.Stage{Name: "other", Parts: []string{nameRight}, Gap: units.Millimeters(30)}))
	require.NoError(t, err)
	left, right := partByName(t, ex, nameLeft), partByName(t, ex, nameRight)
	requireVec(t, r3.Vec{X: -30}, left.Offset, "the ground moves by its gap")
	require.Equal(t, parts.StageDirection, left.Pull)
	requireVec(t, r3.Vec{X: 30}, right.Offset, "the other half opens the other way")
}

func TestExplodeLeaderSeparation(t *testing.T) {
	s := housing(t)
	ex, err := s.Explode(t.Context(), parts.WithExplodeGround(nameBottom), threeStages())
	require.NoError(t, err)
	// The pin (z -44 to -20 after it dropped 32) is 10 mm below the bottom half (z -10 to 0), and the
	// lifted top half (z 10 to 20) is 10 mm above it: a leader reads its two parts' box distance.
	sep := map[string]float64{}
	for _, l := range ex.Leaders {
		sep[l.Joint] = l.Separation.Base()
	}
	require.InDelta(t, 10.0, sep["pin~housing-bottom"], 1e-9)
	require.InDelta(t, 10.0, sep["housing-top~housing-bottom"], 1e-9)
	// A pin that has barely left its bore reads zero: a scale of 0.01 keeps its box inside the base's.
	flat, err := lapSet(t).Explode(t.Context(), parts.WithExplodeGround(nameBase), parts.WithExplodeScale(units.Scalar(0.01)))
	require.NoError(t, err)
	var zero bool
	for _, l := range flat.Leaders {
		if l.Joint == jointPin1Base {
			zero = l.Separation.Base() == 0
		}
	}
	require.True(t, zero, "the pin's box still meets the base's")
}
