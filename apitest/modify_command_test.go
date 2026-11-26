package apitest_test

import (
	"context"
	"math"
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
	patch, err := decad.PatchProfile(t.Context(), doc, s, s.Profiles()[0])
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

// ShellClosed keeps every face, so a Shell with a selector and the sense
// methods of both commands agree on the walls they build.
func TestShellClosedAndShellShareTheirMethods(t *testing.T) {
	t.Parallel()
	_, box := filletBox(t)
	closed, err := decad.ShellClosed(box, units.Millimeters(5)).Sense(decad.Inward).Sense(decad.Inward).Do(t.Context())
	require.NoError(t, err)
	requireModifyVolume(t, closed, 6000*filletBoxHeight-90*50*10)

	doc, box := filletBox(t)
	_, err = decad.ShellClosed(box, units.Millimeters(5)).Sense(decad.ShellSense(9)).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	diagnostics := requireOperationKind(t, err, decad.OperationShell)
	require.Equal(t, "Sense", diagnostics[0].Subject.Name)
	require.Equal(t, []*decad.Body{box}, doc.Bodies())
}
