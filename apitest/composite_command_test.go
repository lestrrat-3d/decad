package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// compositeCommand is one spec-composite command built over a fixture whose
// result is known. next makes a fresh command over the one fixture, so a
// canceled run, which leaves the document unchanged, costs one command and no
// fixture. check asserts the computed geometry of a successful result. noBody
// marks SheetMetal, which takes a document and no body.
//
// stride tunes the mid-build cancellation sweep only: it moves the cancel that
// many checks later per run, where one check at a time would cost minutes
// (agent-docs/command-api-design.md section 14).
type compositeCommand struct {
	name   string
	kind   decad.OperationKind
	noBody bool
	// recorded says the last history entry of a success is the command's own kind.
	recorded bool
	build    func(t *testing.T) (next func() func(context.Context) (*decad.Body, error), doc *decad.Document)
	check    func(t *testing.T, got *decad.Body, doc *decad.Document)
	// sweep builds the smallest fixture on the same build route for the mid-build
	// cancellation sweep, where build's fixture polls too often to sweep.
	sweep  func(t *testing.T) (next func() func(context.Context) (*decad.Body, error), doc *decad.Document)
	stride int64
}

// The names a composite fault carries.
const (
	subjectDocument = "d"
	methodDraft     = "Draft"
	methodHead      = "Head"
)

func requireCompositeBounds(t *testing.T, b *decad.Body, lo, hi r3.Vec) {
	t.Helper()
	box, err := b.Bounds()
	require.NoError(t, err)
	require.True(t, box.Min.Equal(lo, 1e-6), "min %v, want %v", box.Min, lo)
	require.True(t, box.Max.Equal(hi, 1e-6), "max %v, want %v", box.Max, hi)
}

func requireCompositeVolume(t *testing.T, b *decad.Body, want float64) {
	t.Helper()
	v, err := b.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(v.Value.Base()-want), v.Bound.Base()+1e-6, "volume %v ± %v, want %v",
		v.Value.Base(), v.Bound.Base(), want)
}

// sheetMetalFixture is the L bracket of the sheet metal tests: 30 mm and 20 mm
// legs joined by a quarter-turn bend.
func sheetMetalFixture() (legs, bends []units.Value) {
	return []units.Value{units.Millimeters(30), units.Millimeters(20)}, []units.Value{units.Degrees(90)}
}

func compositeCommands() []compositeCommand {
	mm := units.Millimeters
	return []compositeCommand{
		{name: "Hole", kind: decad.OperationHole, recorded: true,
			build: func(t *testing.T) (func() func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, plate := drillPlate(t)
				return func() func(context.Context) (*decad.Body, error) {
					return decad.Hole(plate, capFace(true, plate), r3.Vec{X: 10, Y: 10, Z: 10}, mm(6),
						decad.BlindHole{Depth: mm(6)}).Head(decad.Counterbore{Diameter: mm(10), Depth: mm(2)}).Do
				}, doc
			},
			check: func(t *testing.T, got *decad.Body, doc *decad.Document) {
				requireCompositeBounds(t, got, r3.Vec{}, r3.Vec{X: 20, Y: 20, Z: 10})
				requireCompositeVolume(t, got, drillPlateVolume-math.Pi*9*6-math.Pi*(25-9)*2)
				require.Equal(t, []*decad.Body{got}, doc.Bodies(), "the plate is retired")
			},
			sweep: func(t *testing.T) (func() func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, plate := drillPlate(t)
				return func() func(context.Context) (*decad.Body, error) {
					return decad.Hole(plate, capFace(true, plate), r3.Vec{X: 10, Y: 10, Z: 10}, mm(6),
						decad.BlindHole{Depth: mm(3)}).Do
				}, doc
			},
			stride: 2},
		{name: "HoleShaped", kind: decad.OperationHole, recorded: true,
			build: func(t *testing.T) (func() func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, plate := drillPlate(t)
				shape := decad.RectangularHole{Width: mm(4), Height: mm(6), Angle: units.Degrees(0)}
				return func() func(context.Context) (*decad.Body, error) {
					return decad.HoleShaped(plate, capFace(true, plate), r3.Vec{X: 10, Y: 10, Z: 10}, shape,
						decad.ThroughHole{}).Do
				}, doc
			},
			check: func(t *testing.T, got *decad.Body, doc *decad.Document) {
				requireCompositeBounds(t, got, r3.Vec{}, r3.Vec{X: 20, Y: 20, Z: 10})
				requireCompositeVolume(t, got, drillPlateVolume-4*6*10)
				require.Equal(t, []*decad.Body{got}, doc.Bodies(), "the plate is retired")
			},
			stride: 5},
		{name: "Boss", kind: decad.OperationBoss, recorded: true,
			build: func(t *testing.T) (func() func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, plate := bossPlate(t)
				return func() func(context.Context) (*decad.Body, error) {
					return decad.Boss(plate, capFace(true, plate), r3.Vec{Z: 5}, mm(8), mm(8)).
						Bore(decad.NewBossBore(mm(3), mm(4))).Do
				}, doc
			},
			check: func(t *testing.T, got *decad.Body, doc *decad.Document) {
				requireCompositeBounds(t, got, r3.Vec{X: -20, Y: -20}, r3.Vec{X: 20, Y: 20, Z: 13})
				requireCompositeVolume(t, got, 40*40*5+math.Pi*16*8-math.Pi*2.25*4)
				require.Equal(t, []*decad.Body{got}, doc.Bodies(), "the plate is retired")
			},
			sweep: func(t *testing.T) (func() func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, plate := bossPlate(t)
				return func() func(context.Context) (*decad.Body, error) {
					return decad.Boss(plate, capFace(true, plate), r3.Vec{Z: 5}, mm(8), mm(3)).Do
				}, doc
			},
			stride: 2},
		{name: "BossShaped", kind: decad.OperationBoss, recorded: true,
			build: func(t *testing.T) (func() func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, plate := bossPlate(t)
				shape := decad.RectangularHole{Width: mm(6), Height: mm(4), Angle: units.Degrees(0)}
				return func() func(context.Context) (*decad.Body, error) {
					return decad.BossShaped(plate, capFace(true, plate), r3.Vec{Z: 5}, shape, mm(8)).Do
				}, doc
			},
			check: func(t *testing.T, got *decad.Body, doc *decad.Document) {
				requireCompositeBounds(t, got, r3.Vec{X: -20, Y: -20}, r3.Vec{X: 20, Y: 20, Z: 13})
				requireCompositeVolume(t, got, 40*40*5+6*4*8)
				require.Equal(t, []*decad.Body{got}, doc.Bodies(), "the plate is retired")
			},
			stride: 4},
		{name: "Thread", kind: decad.OperationThread, recorded: true,
			// The smallest rod the Thread route accepts keeps the race cost down;
			// TestThreadExternalRod owns the full 20 mm rod.
			build: func(t *testing.T) (func() func(context.Context) (*decad.Body, error), *decad.Document) {
				doc := decad.New()
				rod := circleBodyAtZ(t, doc, 0, 2, 0, 1.5)
				return func() func(context.Context) (*decad.Body, error) {
					return decad.Thread(rod, decad.Faces(decad.Cylindrical()), mm(1.5)).Do
				}, doc
			},
			check: func(t *testing.T, got *decad.Body, doc *decad.Document) {
				v, err := got.Volume()
				require.NoError(t, err)
				removed := math.Pi*4*1.5 - v.Value.Base()
				require.LessOrEqual(t, math.Abs(removed-removedExternal(2, 1.5)), v.Bound.Base(),
					"the groove removes %v", removed)
				require.Equal(t, []*decad.Body{got}, doc.Bodies(), "the rod is retired")
			},
			sweep: func(t *testing.T) (func() func(context.Context) (*decad.Body, error), *decad.Document) {
				doc := decad.New()
				rod := circleBodyAtZ(t, doc, 0, 2, 0, 1.5)
				return func() func(context.Context) (*decad.Body, error) {
					return decad.Thread(rod, decad.Faces(decad.Cylindrical()), mm(1.5)).Length(mm(1.5)).Do
				}, doc
			},
			stride: 40000},
		{name: "Infill", kind: decad.OperationInfill,
			build: func(t *testing.T) (func() func(context.Context) (*decad.Body, error), *decad.Document) {
				doc := decad.New()
				block := boxBody(t, doc, 0, 0, 5, 5, 5)
				return func() func(context.Context) (*decad.Body, error) {
					return decad.Infill(block, decad.GyroidInfill, mm(5), units.Scalar(0.8)).Resolution(4).Do
				}, doc
			},
			check: func(t *testing.T, got *decad.Body, doc *decad.Document) {
				requireCompositeBounds(t, got, r3.Vec{}, r3.Vec{X: 5, Y: 5, Z: 5})
				v, err := got.Volume()
				require.NoError(t, err)
				require.Greater(t, v.Value.Base(), 10.0, "the lattice leaves material")
				require.Less(t, v.Value.Base(), 120.0, "the lattice leaves holes")
				require.Equal(t, []*decad.Body{got}, doc.Bodies(), "the block and the lattice are consumed")
			},
			stride: 1200},
		{name: "SheetMetal", kind: decad.OperationSheetMetal, noBody: true,
			build: func(t *testing.T) (func() func(context.Context) (*decad.Body, error), *decad.Document) {
				doc := decad.New()
				legs, bends := sheetMetalFixture()
				return func() func(context.Context) (*decad.Body, error) {
					c := decad.SheetMetal(doc, mm(2), mm(3), mm(10), legs, bends)
					return func(ctx context.Context) (*decad.Body, error) {
						b, _, err := c.Do(ctx)
						return b, err
					}
				}, doc
			},
			check: func(t *testing.T, got *decad.Body, doc *decad.Document) {
				requireCompositeBounds(t, got, r3.Vec{Y: -1}, r3.Vec{X: 35, Y: 24, Z: 10})
				require.Equal(t, []*decad.Body{got}, doc.Bodies())
			},
			stride: 1},
	}
}

func TestCompositeCommandsRunOnceAndConsume(t *testing.T) {
	t.Parallel()
	for _, tc := range compositeCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			next, doc := tc.build(t)
			do := next()
			got, err := do(t.Context())
			require.NoError(t, err)
			require.NotNil(t, got)
			tc.check(t, got, doc)
			if tc.recorded {
				operations := doc.OperationHistory().Operations()
				require.Equal(t, tc.kind, operations[len(operations)-1].Kind(), "the history records the command's own kind")
			}

			again, err := do(t.Context())
			require.ErrorIs(t, err, decad.ErrConsumed)
			require.Nil(t, again)
			requireOperationKind(t, err, tc.kind)
		})
	}
}

func TestCompositeCommandsRefuseNilContext(t *testing.T) {
	t.Parallel()
	for _, tc := range compositeCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			next, doc := tc.build(t)
			do := next()
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

func TestCompositeCommandsReturnCancellationByIdentity(t *testing.T) {
	t.Parallel()
	for _, tc := range compositeCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			next, doc := tc.build(t)
			before := doc.Bodies()
			ctx, cancel := context.WithCancel(t.Context())
			cancel()
			_, err := next()(ctx)
			require.Equal(t, context.Canceled, err)
			require.Equal(t, before, doc.Bodies())
		})
	}
}

// A cancel that lands inside the build returns context.Canceled by identity,
// not a sentinel wrapping it (agent-docs/command-api-design.md section 3). The
// sweep moves the cancel one check later each run, or by the member's stride
// where its build polls so often that one check at a time would cost minutes
// (section 14), until the command finishes. Every canceled run leaves the
// document unchanged, so the fixture is built once and each run makes a fresh
// command over it.
func TestCompositeCommandsReturnMidBuildCancellationByIdentity(t *testing.T) {
	t.Parallel()
	const maxChecks = 1000000
	for _, tc := range compositeCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			build := tc.build
			if tc.sweep != nil {
				build = tc.sweep
			}
			next, doc := build(t)
			step := max(tc.stride, 1)
			before := doc.Bodies()
			for n := int64(0); n < maxChecks; n += step {
				got, err := next()(newCancelAfter(t.Context(), n))
				if err == nil {
					require.NotNil(t, got)
					return
				}
				require.Equal(t, context.Canceled, err, "cancel after %d checks", n)
				require.Nil(t, got)
				require.True(t, slicesEqualBodies(before, doc.Bodies()), "cancel after %d checks leaves the document unchanged", n)
			}
			require.FailNow(t, "the command still polled the context", "after %d checks", maxChecks)
		})
	}
}

// Every member that takes a body refuses a retired one at its gate and tags the
// refusal with its own kind. The loop retires each live body of the member's
// fixture, then runs a fresh command over it.
func TestCompositeCommandsRefuseARetiredBody(t *testing.T) {
	t.Parallel()
	for _, tc := range compositeCommands() {
		if tc.noBody {
			continue
		}
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			next, doc := tc.build(t)
			do := next()
			for _, b := range doc.Bodies() {
				require.NoError(t, decad.Remove(b).Do(t.Context()))
			}
			got, err := do(t.Context())
			require.ErrorIs(t, err, decad.ErrRetiredBody)
			require.Nil(t, got)
			requireOperationKind(t, err, tc.kind)
		})
	}
}

// A nil body or document is recorded by the constructor and named after its
// parameter, and a nil ctx is still refused ahead of it.
func TestCompositeConstructorsRecordNilOperands(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	var nilBody *decad.Body
	face := decad.Faces(decad.Planar())
	legs, bends := sheetMetalFixture()
	rows := []struct {
		name    string
		kind    decad.OperationKind
		subject string
		// next builds a fresh command, so the table serves each subtest once.
		next func() func(context.Context) (*decad.Body, error)
	}{
		{"Hole", decad.OperationHole, subjectBody, func() func(context.Context) (*decad.Body, error) {
			return decad.Hole(nilBody, face, r3.Vec{}, mm(3), decad.ThroughHole{}).Do
		}},
		{"HoleShaped", decad.OperationHole, subjectBody, func() func(context.Context) (*decad.Body, error) {
			return decad.HoleShaped(nilBody, face, r3.Vec{}, decad.RectangularHole{}, decad.ThroughHole{}).Do
		}},
		{"Boss", decad.OperationBoss, subjectBody, func() func(context.Context) (*decad.Body, error) {
			return decad.Boss(nilBody, face, r3.Vec{}, mm(3), mm(3)).Do
		}},
		{"BossShaped", decad.OperationBoss, subjectBody, func() func(context.Context) (*decad.Body, error) {
			return decad.BossShaped(nilBody, face, r3.Vec{}, decad.RectangularHole{}, mm(3)).Do
		}},
		{"Thread", decad.OperationThread, subjectBody, func() func(context.Context) (*decad.Body, error) {
			return decad.Thread(nilBody, face, mm(1)).Do
		}},
		{"Infill", decad.OperationInfill, subjectBody, func() func(context.Context) (*decad.Body, error) {
			return decad.Infill(nilBody, decad.GyroidInfill, mm(10), units.Scalar(0.5)).Do
		}},
		{"SheetMetal", decad.OperationSheetMetal, subjectDocument, func() func(context.Context) (*decad.Body, error) {
			return func(ctx context.Context) (*decad.Body, error) {
				b, _, err := decad.SheetMetal(nil, mm(2), mm(3), mm(10), legs, bends).Do(ctx)
				return b, err
			}
		}},
	}
	for _, tc := range rows {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			got, err := tc.next()(t.Context())
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.Nil(t, got)
			diagnostics := requireOperationKind(t, err, tc.kind)
			require.Equal(t, tc.subject, diagnostics[0].Subject.Name)
		})
	}
	t.Run("nil context first", func(t *testing.T) {
		t.Parallel()
		var nilCtx context.Context
		for _, tc := range rows {
			_, err := tc.next()(nilCtx)
			require.ErrorIs(t, err, decad.ErrDegenerate, tc.name)
			diagnostics := requireOperationKind(t, err, tc.kind)
			require.Empty(t, diagnostics[0].Subject.Name, "%s: the nil ctx fault names no subject", tc.name)
		}
	})
}

// A selector of a body that is not a body of any document is the one case the
// constructor cannot see: the gate refuses it, and a nil selector is the
// selector refusal after the retired-body gate.
func TestCompositeNilSelectorsAreRefusedAfterTheBodyGates(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	doc, plate := drillPlate(t)
	_, err := decad.Hole(plate, nil, r3.Vec{}, mm(3), decad.ThroughHole{}).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	requireOperationKind(t, err, decad.OperationHole)
	_, err = decad.Boss(plate, nil, r3.Vec{}, mm(3), mm(3)).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Thread(plate, nil, mm(1)).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "a refusal leaves the plate live")
	require.NoError(t, decad.Remove(plate).Do(t.Context()))
	_, err = decad.Hole(plate, nil, r3.Vec{}, mm(3), decad.ThroughHole{}).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrRetiredBody, "the retired-body gate precedes the selector")
}

// Each stored method fault is refused where the old spec's decode refused it,
// naming the method, and a nil ctx and a consumed command come first.
func TestCompositeCommandsRefuseMethodFaultsAtTheirGate(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	deg := units.Degrees
	badAngle := units.Degrees(90)

	// run builds a fresh command over a fresh fixture for each of the three
	// rows of one method: the fault alone, the fault on a retired body, and
	// the fault with a nil ctx.
	type fixture struct {
		name   string
		kind   decad.OperationKind
		method string
		fault  error
		build  func(t *testing.T) (func(context.Context) (*decad.Body, error), []*decad.Body)
	}
	holeAt := r3.Vec{X: 10, Y: 10, Z: 10}
	plateHole := func(mod func(*decad.HoleCommand) *decad.HoleCommand) func(t *testing.T) (func(context.Context) (*decad.Body, error), []*decad.Body) {
		return func(t *testing.T) (func(context.Context) (*decad.Body, error), []*decad.Body) {
			_, plate := drillPlate(t)
			return mod(decad.Hole(plate, capFace(true, plate), holeAt, mm(4), decad.ThroughHole{})).Do, []*decad.Body{plate}
		}
	}
	plateBoss := func(mod func(*decad.BossCommand) *decad.BossCommand) func(t *testing.T) (func(context.Context) (*decad.Body, error), []*decad.Body) {
		return func(t *testing.T) (func(context.Context) (*decad.Body, error), []*decad.Body) {
			_, plate := bossPlate(t)
			return mod(decad.Boss(plate, capFace(true, plate), r3.Vec{Z: 5}, mm(8), mm(8))).Do, []*decad.Body{plate}
		}
	}
	rodThread := func(mod func(*decad.ThreadCommand) *decad.ThreadCommand) func(t *testing.T) (func(context.Context) (*decad.Body, error), []*decad.Body) {
		return func(t *testing.T) (func(context.Context) (*decad.Body, error), []*decad.Body) {
			doc := decad.New()
			rod := circleBodyAtZ(t, doc, 0, 5, 0, 20)
			return mod(decad.Thread(rod, decad.Faces(decad.Cylindrical()), mm(threadPitch))).Do, []*decad.Body{rod}
		}
	}
	blockInfill := func(mod func(*decad.InfillCommand) *decad.InfillCommand) func(t *testing.T) (func(context.Context) (*decad.Body, error), []*decad.Body) {
		return func(t *testing.T) (func(context.Context) (*decad.Body, error), []*decad.Body) {
			block := boxBody(t, decad.New(), 0, 0, 5, 5, 5)
			// Infill refuses a retired body only when its intersect runs, after the lattice and
			// the method values, so these rows have no retired-body leg.
			return mod(decad.Infill(block, decad.GyroidInfill, mm(5), units.Scalar(0.8))).Do, nil
		}
	}
	rows := []fixture{
		{name: "Hole.Head", kind: decad.OperationHole, method: methodHead, fault: decad.ErrNegativeMagnitude,
			build: plateHole(func(c *decad.HoleCommand) *decad.HoleCommand {
				return c.Head(decad.Counterbore{Diameter: mm(-1), Depth: mm(2)})
			})},
		{name: "Hole.Draft", kind: decad.OperationHole, method: methodDraft, fault: decad.ErrDegenerate,
			build: plateHole(func(c *decad.HoleCommand) *decad.HoleCommand { return c.Draft(badAngle) })},
		{name: "Hole.Draft kind", kind: decad.OperationHole, method: methodDraft, fault: decad.ErrUnitKind,
			build: plateHole(func(c *decad.HoleCommand) *decad.HoleCommand { return c.Draft(mm(1)) })},
		{name: "Hole.Thread", kind: decad.OperationHole, method: "Thread", fault: decad.ErrDegenerate,
			build: plateHole(func(c *decad.HoleCommand) *decad.HoleCommand {
				return c.Thread(decad.NewHoleThread(mm(0)))
			})},
		{name: "Boss.Draft", kind: decad.OperationBoss, method: methodDraft, fault: decad.ErrDegenerate,
			build: plateBoss(func(c *decad.BossCommand) *decad.BossCommand { return c.Draft(badAngle) })},
		{name: "Boss.RootFillet", kind: decad.OperationBoss, method: "RootFillet", fault: decad.ErrDegenerate,
			build: plateBoss(func(c *decad.BossCommand) *decad.BossCommand { return c.RootFillet(mm(0)) })},
		{name: "Boss.Ribs", kind: decad.OperationBoss, method: "Ribs", fault: decad.ErrDegenerate,
			build: plateBoss(func(c *decad.BossCommand) *decad.BossCommand {
				return c.Ribs(decad.NewBossRibs(0, mm(1), mm(3), mm(3)))
			})},
		{name: "Boss.Tilt", kind: decad.OperationBoss, method: "Tilt", fault: decad.ErrDegenerate,
			build: plateBoss(func(c *decad.BossCommand) *decad.BossCommand { return c.Tilt(deg(80), deg(0)) })},
		{name: "Boss.Bore", kind: decad.OperationBoss, method: "Bore", fault: decad.ErrDegenerate,
			build: plateBoss(func(c *decad.BossCommand) *decad.BossCommand {
				return c.Bore(decad.NewBossBore(mm(9), mm(3)))
			})},
		{name: "Thread.Length", kind: decad.OperationThread, method: "Length", fault: decad.ErrDegenerate,
			build: rodThread(func(c *decad.ThreadCommand) *decad.ThreadCommand { return c.Length(mm(25)) })},
		{name: "Thread.Form", kind: decad.OperationThread, method: "Form", fault: decad.ErrDegenerate,
			build: rodThread(func(c *decad.ThreadCommand) *decad.ThreadCommand {
				return c.Form(decad.TrapezoidThreadForm(deg(60), mm(0.5), units.Value{}))
			})},
		{name: "Thread.LeadIn", kind: decad.OperationThread, method: "LeadIn", fault: decad.ErrDegenerate,
			build: rodThread(func(c *decad.ThreadCommand) *decad.ThreadCommand { return c.LeadIn(mm(0)) })},
		{name: "Thread.LeadInAngle", kind: decad.OperationThread, method: "LeadInAngle", fault: decad.ErrDegenerate,
			build: rodThread(func(c *decad.ThreadCommand) *decad.ThreadCommand { return c.LeadInAngle(mm(0.5), deg(95)) })},
		{name: "Thread.LeadInAngle distance", kind: decad.OperationThread, method: "LeadInAngle", fault: decad.ErrDegenerate,
			build: rodThread(func(c *decad.ThreadCommand) *decad.ThreadCommand { return c.LeadInAngle(mm(0), deg(30)) })},
		{name: "Thread.LeadInAngle distance kind", kind: decad.OperationThread, method: "LeadInAngle", fault: decad.ErrUnitKind,
			build: rodThread(func(c *decad.ThreadCommand) *decad.ThreadCommand { return c.LeadInAngle(deg(1), deg(30)) })},
		{name: "Thread.Form all zero trapezoid", kind: decad.OperationThread, method: "Form", fault: decad.ErrDegenerate,
			build: rodThread(func(c *decad.ThreadCommand) *decad.ThreadCommand {
				return c.Form(decad.TrapezoidThreadForm(units.Value{}, units.Value{}, units.Value{}))
			})},
		{name: "Thread.Form all zero buttress", kind: decad.OperationThread, method: "Form", fault: decad.ErrDegenerate,
			build: rodThread(func(c *decad.ThreadCommand) *decad.ThreadCommand {
				return c.Form(decad.ButtressThreadForm(units.Value{}, units.Value{}, units.Value{}, units.Value{}))
			})},
		{name: "Boss.Bore pilot thread", kind: decad.OperationBoss, method: "Bore", fault: decad.ErrDegenerate,
			build: plateBoss(func(c *decad.BossCommand) *decad.BossCommand {
				return c.Bore(decad.NewBossBore(mm(3), mm(4)).Thread(decad.NewHoleThread(mm(0))))
			})},
		{name: "Infill.Resolution", kind: decad.OperationInfill, method: "Resolution", fault: decad.ErrDegenerate,
			build: blockInfill(func(c *decad.InfillCommand) *decad.InfillCommand { return c.Resolution(2) })},
		{name: "Infill.Skin", kind: decad.OperationInfill, method: "Skin", fault: decad.ErrNegativeMagnitude,
			build: blockInfill(func(c *decad.InfillCommand) *decad.InfillCommand { return c.Skin(mm(-1)) })},
		{name: "SheetMetal.KFactor", kind: decad.OperationSheetMetal, method: "KFactor", fault: decad.ErrDegenerate,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), []*decad.Body) {
				legs, bends := sheetMetalFixture()
				c := decad.SheetMetal(decad.New(), mm(2), mm(3), mm(10), legs, bends).KFactor(units.Scalar(2))
				return func(ctx context.Context) (*decad.Body, error) {
					b, _, err := c.Do(ctx)
					return b, err
				}, nil
			}},
	}
	for _, tc := range rows {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			do, bodies := tc.build(t)
			_, err := do(t.Context())
			require.ErrorIs(t, err, tc.fault)
			require.Equal(t, tc.method, requireOperationKind(t, err, tc.kind)[0].Subject.Name)

			var nilCtx context.Context
			do, _ = tc.build(t)
			_, err = do(nilCtx)
			require.ErrorIs(t, err, decad.ErrDegenerate, "a nil ctx precedes the method fault")
			require.Empty(t, requireOperationKind(t, err, tc.kind)[0].Subject.Name)

			// A retired body is refused before the method value is read, for every
			// member whose old spec decode ran after the liveness gate.
			if len(bodies) == 0 {
				return
			}
			do, retiredBodies := tc.build(t)
			for _, b := range retiredBodies {
				require.NoError(t, decad.Remove(b).Do(t.Context()))
			}
			_, err = do(t.Context())
			require.ErrorIs(t, err, decad.ErrRetiredBody, "the retired-body gate precedes the method fault")
		})
	}

	t.Run("SheetMetal.KFactor is refused after the lengths and before the legs", func(t *testing.T) {
		t.Parallel()
		legs, bends := sheetMetalFixture()
		_, _, err := decad.SheetMetal(decad.New(), mm(-1), mm(3), mm(10), legs, bends).KFactor(units.Scalar(2)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrNegativeMagnitude, "the thickness gate precedes the K-factor")
		_, _, err = decad.SheetMetal(decad.New(), mm(2), mm(3), mm(10), nil, bends).KFactor(units.Scalar(2)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Equal(t, "KFactor", requireOperationKind(t, err, decad.OperationSheetMetal)[0].Subject.Name,
			"the K-factor precedes the leg count")
		_, _, err = decad.SheetMetal(decad.New(), mm(2), mm(3), mm(10), legs, bends).KFactor(units.Millimeters(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnitKind)
	})
	t.Run("Hole.Draft precedes the diameter and the depth", func(t *testing.T) {
		t.Parallel()
		_, plate := drillPlate(t)
		_, err := decad.Hole(plate, capFace(true, plate), holeAt, mm(-4), nil).Draft(badAngle).Do(t.Context())
		require.Equal(t, methodDraft, requireOperationKind(t, err, decad.OperationHole)[0].Subject.Name)
		_, err = decad.Hole(plate, capFace(true, plate), holeAt, mm(-4), nil).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrNegativeMagnitude, "without the fault the diameter gate speaks")
	})
}

// A second call to a method replaces the first, so an invalid earlier call is
// forgotten along with its fault (agent-docs/command-api-design.md rule M2).
func TestCompositeMethodsLastCallWins(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	t.Run("Hole.Draft", func(t *testing.T) {
		t.Parallel()
		_, plate := drillPlate(t)
		got, err := decad.Hole(plate, capFace(true, plate), r3.Vec{X: 10, Y: 10, Z: 10}, mm(4), decad.BlindHole{Depth: mm(6)}).
			Draft(units.Degrees(90)).Draft(units.Degrees(0)).Do(t.Context())
		require.NoError(t, err, "a valid call replaces an invalid one")
		requireCompositeVolume(t, got, drillPlateVolume-math.Pi*4*6)

		_, plate2 := drillPlate(t)
		_, err = decad.Hole(plate2, capFace(true, plate2), r3.Vec{X: 10, Y: 10, Z: 10}, mm(4), decad.BlindHole{Depth: mm(6)}).
			Draft(units.Degrees(0)).Draft(units.Degrees(90)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate, "an invalid call replaces a valid one")
	})
	t.Run("Hole.Head", func(t *testing.T) {
		t.Parallel()
		_, plate := drillPlate(t)
		got, err := decad.Hole(plate, capFace(true, plate), r3.Vec{X: 10, Y: 10, Z: 10}, mm(4), decad.ThroughHole{}).
			Head(decad.Counterbore{Diameter: mm(3), Depth: mm(2)}).
			Head(decad.Counterbore{Diameter: mm(8), Depth: mm(3)}).Do(t.Context())
		require.NoError(t, err)
		requireCompositeVolume(t, got, drillPlateVolume-math.Pi*4*10-math.Pi*(16-4)*3)
	})
	t.Run("Boss.Draft", func(t *testing.T) {
		t.Parallel()
		_, plate := bossPlate(t)
		got, err := decad.Boss(plate, capFace(true, plate), r3.Vec{Z: 5}, mm(8), mm(8)).
			Draft(units.Degrees(90)).Draft(units.Degrees(0)).Do(t.Context())
		require.NoError(t, err)
		requireCompositeVolume(t, got, 40*40*5+math.Pi*16*8)
	})
	t.Run("Boss.Bore", func(t *testing.T) {
		t.Parallel()
		_, plate := bossPlate(t)
		got, err := decad.Boss(plate, capFace(true, plate), r3.Vec{Z: 5}, mm(8), mm(8)).
			Bore(decad.NewBossBore(mm(9), mm(3))).Bore(decad.NewBossBore(mm(3), mm(4))).Do(t.Context())
		require.NoError(t, err)
		requireCompositeVolume(t, got, 40*40*5+math.Pi*16*8-math.Pi*2.25*4)
	})
	t.Run("Thread.LeadIn and LeadInAngle share one slot", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		rod := circleBodyAtZ(t, doc, 0, 2, 0, 1.5)
		_, err := decad.Thread(rod, decad.Faces(decad.Cylindrical()), mm(1.5)).Length(mm(1.5)).
			LeadInAngle(mm(0.5), units.Degrees(95)).LeadIn(mm(0.5)).Do(t.Context())
		require.NoError(t, err, "LeadIn replaces the invalid angle with the 45 degree bevel")
		rod2 := circleBodyAtZ(t, doc, 0, 2, 0, 1.5)
		_, err = decad.Thread(rod2, decad.Faces(decad.Cylindrical()), mm(1.5)).Length(mm(1.5)).
			LeadIn(mm(0.5)).LeadInAngle(mm(0.5), units.Degrees(95)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate, "LeadInAngle replaces LeadIn")
		require.Equal(t, "LeadInAngle", requireOperationKind(t, err, decad.OperationThread)[0].Subject.Name)
	})
	t.Run("SheetMetal.KFactor", func(t *testing.T) {
		t.Parallel()
		legs, bends := sheetMetalFixture()
		_, report, err := decad.SheetMetal(decad.New(), mm(2), mm(3), mm(10), legs, bends).
			KFactor(units.Scalar(2)).KFactor(units.Scalar(0.5)).Do(t.Context())
		require.NoError(t, err)
		require.InDelta(t, math.Pi/2*(3+0.5*2), report.BendAllowances[0].Base(), 1e-12, "the latest K-factor is the one used")
		_, _, err = decad.SheetMetal(decad.New(), mm(2), mm(3), mm(10), legs, bends).
			KFactor(units.Scalar(0.5)).KFactor(units.Scalar(2)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("Infill.Resolution and Skin", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		block := boxBody(t, doc, 5, 5, 15, 15, 12)
		_, err := decad.Infill(block, decad.GyroidInfill, mm(10), units.Scalar(0.8)).Resolution(2).Resolution(8).Do(t.Context())
		require.NoError(t, err, "a valid resolution replaces an invalid one")
		block2 := boxBody(t, doc, 5, 5, 15, 15, 12)
		_, err = decad.Infill(block2, decad.GyroidInfill, mm(10), units.Scalar(0.8)).Resolution(8).Resolution(2).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate, "an invalid resolution replaces a valid one")
		_, err = decad.Infill(block2, decad.GyroidInfill, mm(10), units.Scalar(0.8)).Resolution(8).Skin(mm(-1)).Skin(mm(1)).Do(t.Context())
		require.NoError(t, err, "a valid skin replaces an invalid one")
	})
}

// A command copies the selector, slice and sealed values it is given, so a
// later change to the caller's value reaches only the original
// (agent-docs/command-api-design.md rules A2 and A3).
func TestCompositeCommandsCopyTheirArguments(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	t.Run("face selector", func(t *testing.T) {
		t.Parallel()
		for name, build := range map[string]func(plate *decad.Body, q *decad.FaceQuery) func(context.Context) (*decad.Body, error){
			"Hole": func(plate *decad.Body, q *decad.FaceQuery) func(context.Context) (*decad.Body, error) {
				return decad.Hole(plate, q, r3.Vec{X: 10, Y: 10, Z: 10}, mm(4), decad.ThroughHole{}).Do
			},
			"HoleShaped": func(plate *decad.Body, q *decad.FaceQuery) func(context.Context) (*decad.Body, error) {
				return decad.HoleShaped(plate, q, r3.Vec{X: 10, Y: 10, Z: 10},
					decad.RectangularHole{Width: mm(2), Height: mm(2), Angle: units.Degrees(0)}, decad.ThroughHole{}).Do
			},
			"Boss": func(plate *decad.Body, q *decad.FaceQuery) func(context.Context) (*decad.Body, error) {
				return decad.Boss(plate, q, r3.Vec{X: 10, Y: 10, Z: 10}, mm(4), mm(4)).Do
			},
			"BossShaped": func(plate *decad.Body, q *decad.FaceQuery) func(context.Context) (*decad.Body, error) {
				return decad.BossShaped(plate, q, r3.Vec{X: 10, Y: 10, Z: 10},
					decad.RectangularHole{Width: mm(2), Height: mm(2), Angle: units.Degrees(0)}, mm(4)).Do
			},
		} {
			_, plate := drillPlate(t)
			q := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate)))
			do := build(plate, q)
			// A query that now asserts exactly 7 faces would refuse the call.
			q.Exactly(7)
			got, err := do(t.Context())
			require.NoError(t, err, name)
			require.NotNil(t, got, name)
		}
	})
	t.Run("Thread face selector", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		rod := circleBodyAtZ(t, doc, 0, 2, 0, 1.5)
		q := decad.Faces(decad.Cylindrical())
		cmd := decad.Thread(rod, q, mm(1.5)).Length(mm(1.5))
		q.Exactly(7)
		got, err := cmd.Do(t.Context())
		require.NoError(t, err)
		require.NotNil(t, got)
	})
	t.Run("pointer variants stay refused", func(t *testing.T) {
		t.Parallel()
		_, plate := drillPlate(t)
		at := r3.Vec{X: 10, Y: 10, Z: 10}
		_, err := decad.Hole(plate, capFace(true, plate), at, mm(4), &decad.BlindHole{Depth: mm(6)}).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate, "a pointer is no depth variant")
		_, err = decad.Hole(plate, capFace(true, plate), at, mm(4), decad.ThroughHole{}).
			Head(&decad.Counterbore{Diameter: mm(8), Depth: mm(3)}).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate, "a pointer is no head variant")
		require.Equal(t, methodHead, requireOperationKind(t, err, decad.OperationHole)[0].Subject.Name)
		_, err = decad.HoleShaped(plate, capFace(true, plate), at,
			&decad.RectangularHole{Width: mm(4), Height: mm(6), Angle: units.Degrees(0)}, decad.ThroughHole{}).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate, "a pointer is no shape variant")
	})
	t.Run("outline corners", func(t *testing.T) {
		t.Parallel()
		_, plate := drillPlate(t)
		corners := []decad.Point2{{U: -2, V: -2}, {U: 2, V: -2}, {U: 2, V: 2}, {U: -2, V: 2}}
		cmd := decad.HoleShaped(plate, capFace(true, plate), r3.Vec{X: 10, Y: 10, Z: 10},
			decad.OutlineHole{Corners: corners}, decad.ThroughHole{})
		corners[0] = decad.Point2{U: 2, V: 2}
		got, err := cmd.Do(t.Context())
		require.NoError(t, err, "the caller's later write to its corner slice does not reach the command")
		requireCompositeVolume(t, got, drillPlateVolume-4*4*10)
	})
	t.Run("boss bore value", func(t *testing.T) {
		t.Parallel()
		_, plate := bossPlate(t)
		bore := decad.NewBossBore(mm(3), mm(4)).Head(decad.Counterbore{Diameter: mm(5), Depth: mm(1)})
		cmd := decad.Boss(plate, capFace(true, plate), r3.Vec{Z: 5}, mm(8), mm(8)).Bore(bore)
		_ = bore.Head(decad.Counterbore{Diameter: mm(-1), Depth: mm(1)})
		got, err := cmd.Do(t.Context())
		require.NoError(t, err, "the caller's later write to its bore does not reach the command")
		requireCompositeVolume(t, got, 40*40*5+math.Pi*16*8-math.Pi*2.25*4-math.Pi*(6.25-2.25)*1)
	})
	t.Run("sheet metal legs and bends", func(t *testing.T) {
		t.Parallel()
		legs, bends := sheetMetalFixture()
		cmd := decad.SheetMetal(decad.New(), mm(2), mm(3), mm(10), legs, bends)
		legs[0], bends[0] = mm(-5), units.Degrees(0)
		got, report, err := cmd.Do(t.Context())
		require.NoError(t, err, "the caller's later write to its slices does not reach the command")
		requireCompositeBounds(t, got, r3.Vec{Y: -1}, r3.Vec{X: 35, Y: 24, Z: 10})
		require.InDelta(t, 50+math.Pi/2*(3+0.44*2), report.FlatLength.Base(), 1e-12)
	})
}

// Hole and Boss take the same results as before: the pilot, head and draft
// values give the volumes the old spec gave, and a shaped hole takes the
// shape's own size.
func TestCompositeCommandsKeepTheirGeometry(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	t.Run("hole thread cuts the groove", func(t *testing.T) {
		t.Parallel()
		_, plate := drillPlate(t)
		plain, err := decad.Hole(plate, capFace(true, plate), r3.Vec{X: 10, Y: 10, Z: 10}, mm(4.2), decad.ThroughHole{}).Do(t.Context())
		require.NoError(t, err)
		_, plate2 := drillPlate(t)
		tapped, err := decad.Hole(plate2, capFace(true, plate2), r3.Vec{X: 10, Y: 10, Z: 10}, mm(4.2), decad.ThroughHole{}).
			Thread(decad.NewHoleThread(mm(1)).LeftHand().Length(mm(5))).Do(t.Context())
		require.NoError(t, err)
		pv, err := plain.Volume()
		require.NoError(t, err)
		tv, err := tapped.Volume()
		require.NoError(t, err)
		require.Less(t, tv.Value.Base(), pv.Value.Base()-1, "the tapped hole removes the groove as well")
	})
	t.Run("boss tilt leans the boss", func(t *testing.T) {
		t.Parallel()
		_, plate := drillPlate(t)
		got, err := decad.Boss(plate, capFace(true, plate), r3.Vec{X: 10, Y: 10, Z: 10}, mm(8), mm(8)).
			Tilt(units.Degrees(30), units.Degrees(0)).Do(t.Context())
		require.NoError(t, err)
		box, err := got.Bounds()
		require.NoError(t, err)
		// The tilted rim rises to 10 + 8 cos 30 + 4 sin 30 = 18.93 mm; an upright boss stops at 18.
		require.InDelta(t, 10+8*math.Cos(math.Pi/6)+4*math.Sin(math.Pi/6), box.Max.Z, 0.05)
	})
	t.Run("boss ribs and angle", func(t *testing.T) {
		t.Parallel()
		_, plate := bossPlate(t)
		got, err := decad.Boss(plate, capFace(true, plate), r3.Vec{Z: 5}, mm(8), mm(8)).
			Ribs(decad.NewBossRibs(2, mm(1), mm(5), mm(4)).Angle(units.Degrees(90))).Do(t.Context())
		require.NoError(t, err)
		box, err := got.Bounds()
		require.NoError(t, err)
		require.Greater(t, box.Max.Y, 4+4.5, "the first rib reaches out along +Y")
		decadtest.MeasuresBounds(t, got, r3.Vec{X: -20, Y: -20}, r3.Vec{X: 20, Y: 20, Z: 13})
	})
	t.Run("sheet metal k factor", func(t *testing.T) {
		t.Parallel()
		legs, bends := sheetMetalFixture()
		_, report, err := decad.SheetMetal(decad.New(), mm(2), mm(3), mm(10), legs, bends).KFactor(units.Scalar(0.5)).Do(t.Context())
		require.NoError(t, err)
		allowance := math.Pi / 2 * (3 + 0.5*2)
		require.InDelta(t, allowance, report.BendAllowances[0].Base(), 1e-12)
		require.InDelta(t, 50+allowance, report.FlatLength.Base(), 1e-12)
	})
}

// Infill tags the refusals of its nested Union, Intersect and Shell with its own
// kind, and a skin leaves the block live when it is refused.
func TestInfillTagsNestedRefusalsWithItsKind(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	doc := decad.New()
	block := boxBody(t, doc, 5, 5, 15, 15, 12)
	_, err := decad.Infill(block, decad.GyroidInfill, mm(10), units.Scalar(0.8)).Resolution(8).Skin(mm(-1)).Do(t.Context())
	require.Error(t, err)
	requireOperationKind(t, err, decad.OperationInfill)
	_, err = decad.Infill(block, decad.LatticeKind(99), mm(10), units.Scalar(0.8)).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	requireOperationKind(t, err, decad.OperationInfill)
	require.Equal(t, []*decad.Body{block}, doc.Bodies(), "a refusal leaves the block live")
}

// A method called with an explicit units.Value{} is validated as that value, not
// read as unset (agent-docs/command-api-design.md section 12): each is refused by
// Do naming the method.
func TestCompositeMethodsRefuseAnExplicitZeroValue(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	deg := units.Degrees
	zero := units.Value{}
	type row struct {
		name    string
		method  string
		kind    decad.OperationKind
		fault   error
		command func(t *testing.T) func(context.Context) (*decad.Body, error)
	}
	hole := func(mod func(*decad.HoleCommand) *decad.HoleCommand) func(t *testing.T) func(context.Context) (*decad.Body, error) {
		return func(t *testing.T) func(context.Context) (*decad.Body, error) {
			_, plate := drillPlate(t)
			return mod(decad.Hole(plate, capFace(true, plate), r3.Vec{X: 10, Y: 10, Z: 10}, mm(4), decad.ThroughHole{})).Do
		}
	}
	boss := func(mod func(*decad.BossCommand) *decad.BossCommand) func(t *testing.T) func(context.Context) (*decad.Body, error) {
		return func(t *testing.T) func(context.Context) (*decad.Body, error) {
			_, plate := bossPlate(t)
			return mod(decad.Boss(plate, capFace(true, plate), r3.Vec{Z: 5}, mm(8), mm(8))).Do
		}
	}
	rod := func(mod func(*decad.ThreadCommand) *decad.ThreadCommand) func(t *testing.T) func(context.Context) (*decad.Body, error) {
		return func(t *testing.T) func(context.Context) (*decad.Body, error) {
			rod := circleBodyAtZ(t, decad.New(), 0, 2, 0, 1.5)
			return mod(decad.Thread(rod, decad.Faces(decad.Cylindrical()), mm(1.5))).Do
		}
	}
	rows := []row{
		{"Hole.Draft", methodDraft, decad.OperationHole, decad.ErrUnitKind,
			hole(func(c *decad.HoleCommand) *decad.HoleCommand { return c.Draft(zero) })},
		{"Boss.Draft", methodDraft, decad.OperationBoss, decad.ErrUnitKind,
			boss(func(c *decad.BossCommand) *decad.BossCommand { return c.Draft(zero) })},
		{"Boss.RootFillet", "RootFillet", decad.OperationBoss, decad.ErrUnitKind,
			boss(func(c *decad.BossCommand) *decad.BossCommand { return c.RootFillet(zero) })},
		{"Boss.Tilt angle", "Tilt", decad.OperationBoss, decad.ErrUnitKind,
			boss(func(c *decad.BossCommand) *decad.BossCommand { return c.Tilt(zero, deg(0)) })},
		{"Boss.Tilt direction", "Tilt", decad.OperationBoss, decad.ErrUnitKind,
			boss(func(c *decad.BossCommand) *decad.BossCommand { return c.Tilt(deg(30), zero) })},
		{"Thread.Length", "Length", decad.OperationThread, decad.ErrUnitKind,
			rod(func(c *decad.ThreadCommand) *decad.ThreadCommand { return c.Length(zero) })},
		{"Thread.LeadIn", "LeadIn", decad.OperationThread, decad.ErrUnitKind,
			rod(func(c *decad.ThreadCommand) *decad.ThreadCommand { return c.LeadIn(zero) })},
		{"Thread.LeadInAngle angle", "LeadInAngle", decad.OperationThread, decad.ErrUnitKind,
			rod(func(c *decad.ThreadCommand) *decad.ThreadCommand { return c.LeadInAngle(mm(0.5), zero) })},
		{"Infill.Skin", "Skin", decad.OperationInfill, decad.ErrUnitKind,
			func(t *testing.T) func(context.Context) (*decad.Body, error) {
				block := boxBody(t, decad.New(), 0, 0, 5, 5, 5)
				return decad.Infill(block, decad.GyroidInfill, mm(5), units.Scalar(0.8)).Skin(zero).Do
			}},
		{"SheetMetal.KFactor", "KFactor", decad.OperationSheetMetal, decad.ErrDegenerate,
			func(t *testing.T) func(context.Context) (*decad.Body, error) {
				legs, bends := sheetMetalFixture()
				c := decad.SheetMetal(decad.New(), mm(2), mm(3), mm(10), legs, bends).KFactor(zero)
				return func(ctx context.Context) (*decad.Body, error) {
					b, _, err := c.Do(ctx)
					return b, err
				}
			}},
	}
	for _, tc := range rows {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, err := tc.command(t)(t.Context())
			require.Error(t, err)
			require.ErrorIs(t, err, tc.fault)
			require.Equal(t, tc.method, requireOperationKind(t, err, tc.kind)[0].Subject.Name)
		})
	}
}

// A nil shape is no shape variant: the shape switch refuses it as ErrDegenerate.
func TestCompositeShapedConstructorsRefuseANilShape(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	_, plate := drillPlate(t)
	_, err := decad.HoleShaped(plate, capFace(true, plate), r3.Vec{X: 10, Y: 10, Z: 10}, nil, decad.ThroughHole{}).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "unknown hole shape", "the shape switch refuses it, not a later gate")
	requireOperationKind(t, err, decad.OperationHole)

	_, plate = bossPlate(t)
	_, err = decad.BossShaped(plate, capFace(true, plate), r3.Vec{Z: 5}, nil, mm(8)).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "unknown boss shape", "the shape switch refuses it, not a later gate")
	requireOperationKind(t, err, decad.OperationBoss)
}

// A refused Infill leaves the document as it found it: the bodies, the history
// and the producer numbering. The skin route has published its copies and made
// history entries by the time the shell refuses the thickness, so only the
// snapshot restore puts them back. The next box built in the document gets the
// provenance it would get in a document that never ran the Infill.
func TestInfillRestoresHistoryAfterARefusal(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	doc := decad.New()
	block := boxBody(t, doc, 0, 0, 5, 5, 5)
	bodies := doc.Bodies()
	history := len(doc.OperationHistory().Operations())
	_, err := decad.Infill(block, decad.GyroidInfill, mm(5), units.Scalar(0.8)).Resolution(4).Skin(mm(-1)).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	require.True(t, slicesEqualBodies(bodies, doc.Bodies()), "the live bodies changed")
	require.Equal(t, history, len(doc.OperationHistory().Operations()), "the refused Infill left history entries")

	next := boxBody(t, doc, 20, 0, 25, 5, 5)
	fresh := decad.New()
	boxBody(t, fresh, 0, 0, 5, 5, 5)
	want := boxBody(t, fresh, 20, 0, 25, 5, 5)
	require.Equal(t, want.Faces()[0].Origins(), next.Faces()[0].Origins(), "the refused Infill used producer numbers")
}
