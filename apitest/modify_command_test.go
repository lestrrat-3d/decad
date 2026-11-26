package apitest_test

import (
	"context"
	"math"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// modifyCommand is one modify command built over a fresh fixture whose result
// is known. check asserts the computed geometry of the result.
type modifyCommand struct {
	name  string
	kind  decad.OperationKind
	build func(t *testing.T) (do func(context.Context) (*decad.Body, error), doc *decad.Document)
	check func(t *testing.T, got *decad.Body)
}

func requireModifyVolume(t *testing.T, b *decad.Body, want float64) {
	t.Helper()
	v, err := b.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+1e-6)
}

// patchBody is a 10 x 10 sheet patch in the XY plane.
func patchBody(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	patch, err := decad.PatchProfile(doc, s, s.Profiles()[0]).Do(t.Context())
	require.NoError(t, err)
	return doc, patch
}

func modifyCommands() []modifyCommand {
	const h = filletBoxHeight
	return []modifyCommand{
		{name: "Fillet", kind: decad.OperationFillet,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, box := filletBox(t)
				return decad.Fillet(box, verticalEdges(), units.Millimeters(5)).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireModifyVolume(t, got, (6000-25*(4-math.Pi))*h) }},
		{name: "Chamfer", kind: decad.OperationChamfer,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, box := filletBox(t)
				return decad.Chamfer(box, cornerEdge(), units.Millimeters(3)).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireModifyVolume(t, got, 6000*h-4.5*h) }},
		{name: "FilletChord", kind: decad.OperationFillet,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, box := filletBox(t)
				return decad.FilletChord(box, verticalEdges(), units.Millimeters(5*math.Sqrt2)).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireModifyVolume(t, got, (6000-25*(4-math.Pi))*h) }},
		{name: "ChamferAngle", kind: decad.OperationChamfer,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, box := filletBox(t)
				ref := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))
				return decad.ChamferAngle(box, cornerEdge(), units.Millimeters(3), ref, units.Degrees(45)).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireModifyVolume(t, got, 6000*h-4.5*h) }},
		{name: "Shell", kind: decad.OperationShell,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
				top := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1)))
				return decad.Shell(block, top, units.Millimeters(1)).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireModifyVolume(t, got, 480-240) }},
		{name: "ShellClosed", kind: decad.OperationShell,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, box := filletBox(t)
				return decad.ShellClosed(box, units.Millimeters(5)).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireModifyVolume(t, got, 6000*h-90*50*10) }},
		{name: "Draft", kind: decad.OperationDraft,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				doc := decad.New()
				box := plainBox(t, doc, decad.Along)
				return decad.Draft(box, decad.Faces(decad.Walls(box)), capNeutral(box, decad.CapEnd), units.Degrees(5)).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) {
				want, _ := boxVolume(bfMul(bf(boxHeight), taperTan(5))).Float64()
				requireModifyVolume(t, got, want)
			}},
		{name: "Offset", kind: decad.OperationOffset,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, patch := patchBody(t)
				return decad.Offset(patch, units.Millimeters(2)).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) {
				box, err := got.Bounds()
				require.NoError(t, err)
				require.InDelta(t, 2.0, math.Abs(box.Min.Z), 1e-9)
				require.InDelta(t, 2.0, math.Abs(box.Max.Z), 1e-9)
			}},
		{name: "Thicken", kind: decad.OperationThicken,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, patch := patchBody(t)
				return decad.Thicken(patch, units.Millimeters(2)).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireModifyVolume(t, got, 200) }},
		{name: "MoveFace", kind: decad.OperationMoveFace,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
				top := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1)))
				return decad.MoveFace(block, top, units.Millimeters(2)).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireModifyVolume(t, got, 10*8*8) }},
	}
}

func TestModifyCommandsRunOnceAndConsume(t *testing.T) {
	t.Parallel()
	for _, tc := range modifyCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			do, doc := tc.build(t)
			got, err := do(t.Context())
			require.NoError(t, err)
			tc.check(t, got)
			bodies := doc.Bodies()

			again, err := do(t.Context())
			require.ErrorIs(t, err, decad.ErrConsumed)
			require.Nil(t, again)
			diagnostics := requireOperationKind(t, err, tc.kind)
			require.Equal(t, decad.OpDiagInputRejected, diagnostics[0].Code)
			require.Equal(t, decad.OperationStageInput, diagnostics[0].Stage)
			require.Empty(t, diagnostics[0].Subject.Name)
			require.Equal(t, bodies, doc.Bodies(), "a consumed command changes nothing")
		})
	}
}

func TestModifyCommandsRefuseNilContext(t *testing.T) {
	t.Parallel()
	for _, tc := range modifyCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			do, doc := tc.build(t)
			before := doc.Bodies()
			var nilCtx context.Context
			got, err := do(nilCtx)
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.Nil(t, got)
			requireOperationKind(t, err, tc.kind)
			require.Equal(t, before, doc.Bodies())

			_, err = do(t.Context())
			require.ErrorIs(t, err, decad.ErrConsumed, "the refused Do consumed the command")
		})
	}
}

func TestModifyCommandsReturnCancellationByIdentity(t *testing.T) {
	t.Parallel()
	for _, tc := range modifyCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			do, doc := tc.build(t)
			before := doc.Bodies()
			ctx, cancel := context.WithCancel(t.Context())
			cancel()
			_, err := do(ctx)
			require.Equal(t, context.Canceled, err)
			require.Equal(t, before, doc.Bodies())
		})
	}
}

// A nil body is recorded by the constructor, named body, and a nil ctx is
// still refused ahead of it.
func TestModifyConstructorsRecordANilBody(t *testing.T) {
	t.Parallel()
	edges, faces := verticalEdges(), decad.Faces(decad.Planar())
	mm := units.Millimeters(1)
	for _, tc := range []struct {
		name string
		kind decad.OperationKind
		do   func(context.Context) (*decad.Body, error)
	}{
		{"Fillet", decad.OperationFillet, decad.Fillet(nil, edges, mm).Do},
		{"Chamfer", decad.OperationChamfer, decad.Chamfer(nil, edges, mm).Do},
		{"FilletChord", decad.OperationFillet, decad.FilletChord(nil, edges, mm).Do},
		{"ChamferAngle", decad.OperationChamfer, decad.ChamferAngle(nil, edges, mm, faces, units.Degrees(30)).Do},
		{"Shell", decad.OperationShell, decad.Shell(nil, faces, mm).Do},
		{"ShellClosed", decad.OperationShell, decad.ShellClosed(nil, mm).Do},
		{"Draft", decad.OperationDraft, decad.Draft(nil, faces, decad.NeutralFace{}, units.Degrees(5)).Do},
		{"MoveFace", decad.OperationMoveFace, decad.MoveFace(nil, faces, mm).Do},
		{"ConeFromFace", decad.OperationConeFromFace, decad.ConeFromFace(nil, faces, mm).Do},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			got, err := tc.do(t.Context())
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.Nil(t, got)
			diagnostics := requireOperationKind(t, err, tc.kind)
			require.Equal(t, "body", diagnostics[0].Subject.Name)
		})
	}
	t.Run("nil context first", func(t *testing.T) {
		t.Parallel()
		var nilCtx context.Context
		_, err := decad.Fillet(nil, edges, mm).Do(nilCtx)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		diagnostics := requireOperationKind(t, err, decad.OperationFillet)
		require.Empty(t, diagnostics[0].Subject.Name, "a nil context is no fault of the body")
	})
}

// Table S of agent-docs/modify-design.md keeps its order: a retired receiver
// is ErrRetiredBody ahead of a bad magnitude, and each magnitude gate keeps
// its sentinel.
func TestModifyCommandsKeepTheGateOrder(t *testing.T) {
	t.Parallel()
	doc, box := filletBox(t)
	_, err := decad.Fillet(box, verticalEdges(), units.Millimeters(5)).Do(t.Context())
	require.NoError(t, err)
	for _, tc := range []struct {
		name string
		do   func() error
	}{
		{"Fillet", func() error {
			_, err := decad.Fillet(box, verticalEdges(), units.Millimeters(-1)).Do(t.Context())
			return err
		}},
		{"Chamfer", func() error {
			_, err := decad.Chamfer(box, verticalEdges(), units.Millimeters(-1)).Do(t.Context())
			return err
		}},
		{"Shell", func() error {
			_, err := decad.Shell(box, topCap(box), units.Millimeters(-1)).Do(t.Context())
			return err
		}},
	} {
		require.ErrorIs(t, tc.do(), decad.ErrRetiredBody, tc.name)
	}

	_, live := filletBox(t)
	_, err = decad.Fillet(live, verticalEdges(), units.Millimeters(-1)).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	_, live = filletBox(t)
	_, err = decad.Fillet(live, verticalEdges(), units.Millimeters(0)).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Len(t, doc.Bodies(), 1)
}

// A command copies its selector when it is built, so a later change to the
// caller's query reaches nothing.
func TestModifyCommandsCopyTheirSelector(t *testing.T) {
	t.Parallel()
	_, box := filletBox(t)
	sel := verticalEdges().Exactly(4)
	cmd := decad.Fillet(box, sel, units.Millimeters(5))
	sel.Exactly(1) // a later change that would refuse the four edges.
	got, err := cmd.Do(t.Context())
	require.NoError(t, err)
	requireModifyVolume(t, got, (6000-25*(4-math.Pi))*filletBoxHeight)
}

func TestSideMethodsLastCallWins(t *testing.T) {
	t.Parallel()
	t.Run("Offset", func(t *testing.T) {
		t.Parallel()
		z := func(side decad.OffsetSide, extra bool) float64 {
			_, patch := patchBody(t)
			cmd := decad.Offset(patch, units.Millimeters(2))
			if extra {
				cmd.Side(decad.OffsetPositive)
			}
			got, err := cmd.Side(side).Do(t.Context())
			require.NoError(t, err)
			box, err := got.Bounds()
			require.NoError(t, err)
			return box.Max.Z
		}
		require.InDelta(t, 2.0, z(decad.OffsetPositive, false), 1e-9)
		require.InDelta(t, -2.0, z(decad.OffsetNegative, false), 1e-9)
		require.InDelta(t, -2.0, z(decad.OffsetNegative, true), 1e-9, "the second call replaces the first")

		_, patch := patchBody(t)
		got, err := decad.Offset(patch, units.Millimeters(2)).Side(decad.OffsetSide(7)).Side(decad.OffsetNegative).Do(t.Context())
		require.NoError(t, err, "a valid call replaces an invalid one")
		require.NotNil(t, got)

		doc, patch := patchBody(t)
		_, err = decad.Offset(patch, units.Millimeters(2)).Side(decad.OffsetNegative).Side(decad.OffsetSide(7)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		diagnostics := requireOperationKind(t, err, decad.OperationOffset)
		require.Equal(t, "Side", diagnostics[0].Subject.Name)
		require.Equal(t, []*decad.Body{patch}, doc.Bodies())
	})
	t.Run("Thicken", func(t *testing.T) {
		t.Parallel()
		zRange := func(side decad.ThickenSide) (lo, hi float64) {
			_, patch := patchBody(t)
			got, err := decad.Thicken(patch, units.Millimeters(2)).Side(decad.ThickenPositive).Side(side).Do(t.Context())
			require.NoError(t, err)
			box, err := got.Bounds()
			require.NoError(t, err)
			return box.Min.Z, box.Max.Z
		}
		lo, hi := zRange(decad.ThickenNegative)
		require.InDelta(t, -2.0, lo, 1e-9)
		require.InDelta(t, 0.0, hi, 1e-9)
		lo, hi = zRange(decad.ThickenCentered)
		require.InDelta(t, -1.0, lo, 1e-9)
		require.InDelta(t, 1.0, hi, 1e-9)

		doc, patch := patchBody(t)
		_, err := decad.Thicken(patch, units.Millimeters(2)).Side(decad.ThickenSide(7)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		diagnostics := requireOperationKind(t, err, decad.OperationThicken)
		require.Equal(t, "Side", diagnostics[0].Subject.Name)
		require.Equal(t, []*decad.Body{patch}, doc.Bodies())
	})
}

// ShellClosed keeps every face, and a Shell with a selector shares its Sense
// and FaceThickness methods: each command takes the last call of a method,
// refuses a bad one with the method as subject, and builds the same walls.
func TestShellClosedAndShellShareTheirMethods(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	t.Run("ShellClosed", func(t *testing.T) {
		t.Parallel()
		_, box := filletBox(t)
		closed, err := decad.ShellClosed(box, mm(5)).Sense(decad.Outward).Sense(decad.Inward).Do(t.Context())
		require.NoError(t, err)
		requireModifyVolume(t, closed, 6000*filletBoxHeight-90*50*10)

		doc, box := filletBox(t)
		_, err = decad.ShellClosed(box, mm(5)).Sense(decad.ShellSense(9)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "SX1")
		diagnostics := requireOperationKind(t, err, decad.OperationShell)
		require.Equal(t, "Sense", diagnostics[0].Subject.Name)
		require.Equal(t, []*decad.Body{box}, doc.Bodies())
	})
	t.Run("ShellOpen", func(t *testing.T) {
		t.Parallel()
		_, box := shellBox(t)
		open, err := decad.Shell(box, topCap(box), mm(5)).Sense(decad.Outward).Sense(decad.Inward).Do(t.Context())
		require.NoError(t, err)
		_, plain := shellBox(t)
		want, err := decad.Shell(plain, topCap(plain), mm(5)).Do(t.Context())
		require.NoError(t, err)
		wantVolume, err := want.Volume()
		require.NoError(t, err)
		requireModifyVolume(t, open, wantVolume.Value.Base())

		doc, box := shellBox(t)
		_, err = decad.Shell(box, topCap(box), mm(5)).Sense(decad.ShellSense(9)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "SX1")
		diagnostics := requireOperationKind(t, err, decad.OperationShell)
		require.Equal(t, "Sense", diagnostics[0].Subject.Name)
		require.Equal(t, []*decad.Body{box}, doc.Bodies())
	})
	t.Run("FaceThickness", func(t *testing.T) {
		t.Parallel()
		down := r3.NewVec(0, 0, -1)
		// The block is 480 mm³. Walls of 1 mm and a 3 mm floor leave a cavity
		// 8 x 6 x 2 inside it.
		_, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
		closed, err := decad.ShellClosed(block, mm(1)).FaceThickness(down, mm(2)).FaceThickness(down, mm(3)).Do(t.Context())
		require.NoError(t, err)
		requireModifyVolume(t, closed, 480-8*6*2)

		_, block = blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
		open, err := decad.Shell(block, decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), mm(1)).FaceThickness(down, mm(3)).Do(t.Context())
		require.NoError(t, err)
		requireModifyVolume(t, open, 480-8*6*3)
	})
}

// Each stored method fault is refused at its place in the gate order: after
// S17 and the sheet refusal of Table X, before S15 (agent-docs/modify-design.md
// Table S). The retired-body rows cover every member.
func TestModifyCommandsRefuseMethodFaultsAtTheirGate(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	ref := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))
	t.Run("retired body first", func(t *testing.T) {
		t.Parallel()
		// Retire a solid and a sheet, then call each member on them.
		_, box := filletBox(t)
		_, err := decad.Fillet(box, verticalEdges(), mm(5)).Do(t.Context())
		require.NoError(t, err)
		_, patch := patchBody(t)
		_, err = decad.Thicken(patch, mm(2)).Do(t.Context())
		require.NoError(t, err)
		top := topCap(box)
		for _, tc := range []struct {
			name string
			do   func() error
		}{
			{"Fillet", func() error { _, err := decad.Fillet(box, verticalEdges(), mm(-1)).Do(t.Context()); return err }},
			{"Chamfer", func() error {
				_, err := decad.Chamfer(box, cornerEdge(), mm(-1)).Asymmetric(ref, mm(0)).Do(t.Context())
				return err
			}},
			{"FilletChord", func() error { _, err := decad.FilletChord(box, verticalEdges(), mm(5)).Do(t.Context()); return err }},
			{"ChamferAngle", func() error {
				_, err := decad.ChamferAngle(box, cornerEdge(), mm(3), ref, units.Degrees(45)).Do(t.Context())
				return err
			}},
			{"ShellOpen", func() error {
				_, err := decad.Shell(box, top, mm(-1)).Sense(decad.ShellSense(9)).Do(t.Context())
				return err
			}},
			{"ShellClosed", func() error {
				_, err := decad.ShellClosed(box, mm(-1)).Sense(decad.ShellSense(9)).Do(t.Context())
				return err
			}},
			{"Draft", func() error {
				_, err := decad.Draft(box, decad.Faces(decad.Walls(box)), capNeutral(box, decad.CapEnd), units.Degrees(5)).Do(t.Context())
				return err
			}},
			{"MoveFace", func() error { _, err := decad.MoveFace(box, top, mm(2)).Do(t.Context()); return err }},
			{"Offset", func() error {
				_, err := decad.Offset(patch, mm(-1)).Side(decad.OffsetSide(7)).Do(t.Context())
				return err
			}},
			{"Thicken", func() error {
				_, err := decad.Thicken(patch, mm(-1)).Side(decad.ThickenSide(7)).Do(t.Context())
				return err
			}},
			{"ConeFromFace", func() error { _, err := decad.ConeFromFace(box, top, mm(-5)).Do(t.Context()); return err }},
		} {
			require.ErrorIs(t, tc.do(), decad.ErrRetiredBody, tc.name)
		}
	})
	t.Run("sheet refusal before the method fault", func(t *testing.T) {
		t.Parallel()
		_, patch := patchBody(t)
		_, err := decad.Chamfer(patch, cornerEdge(), mm(3)).Asymmetric(ref, mm(0)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		_, patch = patchBody(t)
		_, err = decad.Shell(patch, topCap(patch), mm(1)).Sense(decad.ShellSense(9)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
	t.Run("method fault before the magnitude gate", func(t *testing.T) {
		t.Parallel()
		check := func(t *testing.T, kind decad.OperationKind, subject string, err error) {
			t.Helper()
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.NotErrorIs(t, err, decad.ErrNegativeMagnitude)
			diagnostics := requireOperationKind(t, err, kind)
			require.Equal(t, subject, diagnostics[0].Subject.Name)
		}
		_, box := filletBox(t)
		_, err := decad.Chamfer(box, cornerEdge(), mm(-1)).Asymmetric(ref, mm(0)).Do(t.Context())
		check(t, decad.OperationChamfer, "Asymmetric", err)
		_, box = filletBox(t)
		_, err = decad.Shell(box, topCap(box), mm(-1)).Sense(decad.ShellSense(9)).Do(t.Context())
		check(t, decad.OperationShell, "Sense", err)
		_, box = filletBox(t)
		_, err = decad.ShellClosed(box, mm(-1)).Sense(decad.ShellSense(9)).Do(t.Context())
		check(t, decad.OperationShell, "Sense", err)
		_, patch := patchBody(t)
		_, err = decad.Offset(patch, mm(-1)).Side(decad.OffsetSide(7)).Do(t.Context())
		check(t, decad.OperationOffset, "Side", err)
		_, patch = patchBody(t)
		_, err = decad.Thicken(patch, mm(-1)).Side(decad.ThickenSide(7)).Do(t.Context())
		check(t, decad.OperationThicken, "Side", err)
	})
}

// A computed second setback that is refused names the constructor argument it
// came from, never the Asymmetric method ChamferAngleCommand does not have.
func TestChamferAngleNamesTheArgumentOfAComputedFault(t *testing.T) {
	t.Parallel()
	doc, box := filletBox(t)
	ref := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))
	_, err := decad.ChamferAngle(box, cornerEdge(), units.Millimeters(0), ref, units.Degrees(45)).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	diagnostics := requireOperationKind(t, err, decad.OperationChamfer)
	require.Equal(t, "d", diagnostics[0].Subject.Name)
	require.Equal(t, []*decad.Body{box}, doc.Bodies())
}

// cancelAfter is a context whose Err turns to context.Canceled after n checks,
// so a cancel can land in the middle of a build, past every early poll.
type cancelAfter struct {
	context.Context //nolint:containedctx // a test wrapper that counts Err calls.
	left            atomic.Int64
}

func newCancelAfter(parent context.Context, n int64) *cancelAfter {
	c := &cancelAfter{Context: parent}
	c.left.Store(n)
	return c
}

func (c *cancelAfter) Err() error {
	if c.left.Add(-1) < 0 {
		return context.Canceled
	}
	return c.Context.Err()
}

// A cancel that lands inside the build returns context.Canceled by identity,
// not a sentinel wrapping it (agent-docs/command-api-design.md section 3). The
// sweep moves the cancel one check later each run, or by the member's stride
// where its build polls so often that one check at a time would cost minutes
// (section 14), until the command finishes. Every canceled run leaves the
// document unchanged, so the fixture is built once and each run makes a fresh
// command over it. A strided sweep still runs the first few check points one
// at a time.
func TestModifyCommandsReturnMidBuildCancellationByIdentity(t *testing.T) {
	t.Parallel()
	const maxChecks = 4000
	const denseHead = 4
	type runner = func() func(context.Context) (*decad.Body, error)
	for _, tc := range []struct {
		name string
		// stride moves the cancel by this many checks after the first
		// denseHead runs; below two it moves one check.
		stride int64
		build  func(t *testing.T) (runner, *decad.Document)
	}{
		{name: "ShellOpen", build: func(t *testing.T) (runner, *decad.Document) {
			doc, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
			top := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1)))
			return func() func(context.Context) (*decad.Body, error) {
				return decad.Shell(block, top, units.Millimeters(1)).Do
			}, doc
		}},
		{name: "ShellClosed", build: func(t *testing.T) (runner, *decad.Document) {
			doc, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
			return func() func(context.Context) (*decad.Body, error) {
				return decad.ShellClosed(block, units.Millimeters(1)).Do
			}, doc
		}},
		// Offset polls 406 times, so it takes the stride of about N/10.
		{name: "Offset", stride: 40, build: func(t *testing.T) (runner, *decad.Document) {
			doc, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
			return func() func(context.Context) (*decad.Body, error) {
				return decad.Offset(block, units.Millimeters(1)).Do
			}, doc
		}},
		{name: "MoveFace", build: func(t *testing.T) (runner, *decad.Document) {
			doc, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
			top := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1)))
			return func() func(context.Context) (*decad.Body, error) {
				return decad.MoveFace(block, top, units.Millimeters(2)).Do
			}, doc
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			next, doc := tc.build(t)
			before := doc.Bodies()
			for n := int64(0); n < maxChecks; {
				got, err := next()(newCancelAfter(t.Context(), n))
				if err == nil {
					require.NotNil(t, got)
					require.Greater(t, n, int64(1), "the cancel never reached the build")
					return
				}
				require.Equal(t, context.Canceled, err, "cancel after %d checks", n)
				require.Nil(t, got)
				require.True(t, slicesEqualBodies(before, doc.Bodies()), "cancel after %d checks leaves the document unchanged", n)
				if n+1 < denseHead || tc.stride < 2 {
					n++
					continue
				}
				n += tc.stride
			}
			require.FailNow(t, "the command still polled the context", "after %d checks", maxChecks)
		})
	}
}

// A sealed argument that holds a selector is copied with that selector, so a
// later change to the caller's query reaches nothing (rule A3).
func TestDraftCopiesItsNeutralFaceSelector(t *testing.T) {
	t.Parallel()
	build := func(t *testing.T, neutral func(*decad.Body, *decad.FaceQuery) decad.NeutralPlane) {
		doc := decad.New()
		box := plainBox(t, doc, decad.Along)
		q := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(box)))
		cmd := decad.Draft(box, decad.Faces(decad.Walls(box)), neutral(box, q), units.Degrees(5))
		q.Exactly(3) // a later change that would refuse the one cap.
		got, err := cmd.Do(t.Context())
		require.NoError(t, err)
		want, _ := boxVolume(bfMul(bf(boxHeight), taperTan(5))).Float64()
		requireModifyVolume(t, got, want)
	}
	t.Run("value", func(t *testing.T) {
		t.Parallel()
		build(t, func(b *decad.Body, q *decad.FaceQuery) decad.NeutralPlane {
			return decad.NeutralFace{Body: b, Face: q}
		})
	})
	t.Run("pointer", func(t *testing.T) {
		t.Parallel()
		build(t, func(b *decad.Body, q *decad.FaceQuery) decad.NeutralPlane {
			return &decad.NeutralFace{Body: b, Face: q}
		})
	})
}
