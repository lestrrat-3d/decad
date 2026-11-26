package apitest_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// surfaceCommand is one surface, mesh-to-solid or retirement command built over
// a fixture whose result is known. next makes a fresh command over the one
// fixture, so a canceled run, which leaves the document unchanged, costs one
// command and no fixture. check asserts the computed geometry of a successful
// result.
//
// stride tunes the mid-build cancellation sweep only: it moves the cancel that
// many checks later per run (agent-docs/command-api-design.md section 14).
// noOperand marks a command with no body operand to retire.
type surfaceCommand struct {
	name      string
	kind      decad.OperationKind
	build     func(t *testing.T) (next func() func(context.Context) ([]*decad.Body, error), doc *decad.Document)
	check     func(t *testing.T, got []*decad.Body, doc *decad.Document)
	stride    int64
	noOperand bool
}

func surfaceCommands() []surfaceCommand {
	return []surfaceCommand{
		{name: "PatchEdges", kind: decad.OperationPatchEdges,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc := decad.New()
				s, p := plateSketch(t)
				sheet, err := decad.PatchProfile(doc, s, p).Do(t.Context())
				require.NoError(t, err)
				return func() func(context.Context) ([]*decad.Body, error) {
					return single(decad.PatchEdges(sheet, decad.Edges(decad.Free()).Exactly(4)).Do)
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				require.Len(t, got, 1)
				require.Equal(t, decad.BodySheet, got[0].Kind())
				decadtest.MeasuresArea(t, got[0], units.SquareMillimeters(12000), decadtest.Exactly())
				require.Equal(t, got, doc.Bodies(), "the receiver is retired")
			}},
		{name: "Stitch", kind: decad.OperationStitch,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc := decad.New()
				walls, bottom, top := stitchBoxSheets(t, doc, 10)
				return func() func(context.Context) ([]*decad.Body, error) {
					return single(decad.Stitch(walls, bottom, top).Do)
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				require.Len(t, got, 1)
				require.Equal(t, decad.BodySolid, got[0].Kind())
				decadtest.MeasuresVolume(t, got[0], units.CubicMillimeters(60000), decadtest.Exactly())
				require.Equal(t, got, doc.Bodies(), "every operand is retired")
			}},
		{name: "Unstitch", kind: decad.OperationUnstitch,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc := decad.New()
				box := unstitchBox(t, doc)
				return func() func(context.Context) ([]*decad.Body, error) {
					return decad.Unstitch(box).Do
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				require.Len(t, got, 6)
				for _, b := range got {
					require.Equal(t, decad.BodySheet, b.Kind())
					require.Len(t, b.Faces(), 1)
				}
				require.Equal(t, got, doc.Bodies(), "the receiver is retired")
			}},
		{name: "Trim", kind: decad.OperationTrim,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc := decad.New()
				sheet := trimRectSheet(t, doc)
				tool := trimSpanningTool(t, doc, 40, -10, 60, 70)
				return func() func(context.Context) ([]*decad.Body, error) {
					return single(decad.Trim(sheet, tool, decad.KeepOutside).Do)
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				require.Len(t, got, 1)
				require.Len(t, got[0].Lumps(), 2, "the tool splits the sheet into two ribbons")
				decadtest.MeasuresArea(t, got[0], units.SquareMillimeters(2800))
				require.Equal(t, got, doc.Bodies(), "both operands are retired")
			}},
		{name: "Extend", kind: decad.OperationExtend,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc := decad.New()
				ribbon, _ := extendLeftRibbon(t, doc)
				tool := trimSpanningTool(t, doc, 70, -10, 90, 10)
				query := decad.Edges(decad.Free(), decad.ParallelTo(r3.NewVec(0, 0, 1)),
					decad.EndpointAt(r3.NewVec(40, 0, 0))).Exactly(1)
				return func() func(context.Context) ([]*decad.Body, error) {
					return single(decad.Extend(ribbon, query, tool).Do)
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				require.Len(t, got, 1)
				bounds, err := got[0].Bounds()
				require.NoError(t, err)
				require.InDelta(t, 70, bounds.Max.X, bounds.Bound.Base()+1e-9, "the ribbon reaches the tool")
				decadtest.MeasuresArea(t, got[0], units.SquareMillimeters(700))
				require.Equal(t, got, doc.Bodies(), "both operands are retired")
			}},
		{name: "Split", kind: decad.OperationSplit,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc := decad.New()
				target := splitBlock(t, doc)
				tool := splitRibbon(t, doc, 110)
				return func() func(context.Context) ([]*decad.Body, error) {
					return decad.Split(target, tool).Do
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				require.Len(t, got, 2)
				for _, piece := range got {
					require.Equal(t, decad.BodySolid, piece.Kind())
					decadtest.MeasuresVolume(t, piece, units.CubicMillimeters(30000))
				}
				require.Equal(t, got, doc.Bodies(), "both operands are retired")
			}},
		{name: "SolidFromMesh", kind: decad.OperationSolidFromMesh, noOperand: true,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				v, tris := cubeMesh(r3.Vec{}, r3.NewVec(10, 10, 10))
				m, err := decad.NewMeshBody(t.Context(), v, tris)
				require.NoError(t, err)
				doc := decad.New()
				return func() func(context.Context) ([]*decad.Body, error) {
					return single(decad.SolidFromMesh(doc, m).Do)
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				require.Len(t, got, 1)
				require.Len(t, got[0].Faces(), 6)
				requireVolume(t, got[0], 1000)
				require.Equal(t, got, doc.Bodies())
			}},
		{name: "Remove", kind: decad.OperationRemove,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc := decad.New()
				keep := decadtest.NewBlock(t, doc, 0, 0, 10, 10, units.Millimeters(3))
				drop := decadtest.NewBlock(t, doc, 20, 0, 30, 10, units.Millimeters(3))
				return func() func(context.Context) ([]*decad.Body, error) {
					cmd := decad.Remove(drop)
					return func(ctx context.Context) ([]*decad.Body, error) {
						if err := cmd.Do(ctx); err != nil {
							return nil, err
						}
						return []*decad.Body{keep}, nil
					}
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				require.Equal(t, got, doc.Bodies(), "only the removed body leaves the live set")
			}},
	}
}

func TestSurfaceCommandsRunOnceAndConsume(t *testing.T) {
	t.Parallel()
	for _, tc := range surfaceCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			next, doc := tc.build(t)
			do := next()
			got, err := do(t.Context())
			require.NoError(t, err)
			tc.check(t, got, doc)

			requireSecondDoConsumed(t, do, tc.kind, doc)
		})
	}
}

// requireSecondDoConsumed runs do a second time and requires ErrConsumed, tagged
// with kind and with no subject, and a document left as it was.
func requireSecondDoConsumed(t *testing.T, do func(context.Context) ([]*decad.Body, error),
	kind decad.OperationKind, doc *decad.Document) {
	t.Helper()
	bodies := doc.Bodies()
	again, err := do(t.Context())
	require.ErrorIs(t, err, decad.ErrConsumed)
	require.Nil(t, again)
	first := requireOperationKind(t, err, kind)[0]
	require.Equal(t, decad.OpDiagInputRejected, first.Code)
	require.Equal(t, decad.OperationStageInput, first.Stage)
	require.Empty(t, first.Subject.Name)
	require.Equal(t, bodies, doc.Bodies(), "a consumed command changes nothing")
}

func TestSurfaceCommandsRefuseNilContext(t *testing.T) {
	t.Parallel()
	for _, tc := range surfaceCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			next, doc := tc.build(t)
			do := next()
			before := doc.Bodies()
			var nilCtx context.Context
			_, err := do(nilCtx)
			require.ErrorIs(t, err, decad.ErrDegenerate)
			requireOperationKind(t, err, tc.kind)
			require.Equal(t, before, doc.Bodies())

			_, err = do(t.Context())
			require.ErrorIs(t, err, decad.ErrConsumed, "the refused Do consumed the command")
		})
	}
}

func TestSurfaceCommandsReturnCancellationByIdentity(t *testing.T) {
	t.Parallel()
	for _, tc := range surfaceCommands() {
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
func TestSurfaceCommandsReturnMidBuildCancellationByIdentity(t *testing.T) {
	t.Parallel()
	const maxChecks = 6000
	for _, tc := range surfaceCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			next, doc := tc.build(t)
			step := max(tc.stride, 1)
			before := doc.Bodies()
			for n := int64(0); n < maxChecks; n += step {
				got, err := next()(newCancelAfter(t.Context(), n))
				if err == nil {
					require.Greater(t, n, int64(0), "the cancel never reached the build")
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

// A nil body is recorded by the constructor and named after its parameter, a
// nil ctx is still refused ahead of it, and Stitch records nothing because its
// own gate reads the context first.
func TestSurfaceConstructorsRecordNilOperands(t *testing.T) {
	t.Parallel()
	var nilBody *decad.Body
	var nilDoc *decad.Document
	query := decad.Edges(decad.Free())
	rows := []struct {
		name    string
		kind    decad.OperationKind
		subject string
		// next builds a fresh command, so the table serves each subtest once.
		next func() func(context.Context) ([]*decad.Body, error)
	}{
		{"PatchEdges", decad.OperationPatchEdges, subjectBody, func() func(context.Context) ([]*decad.Body, error) {
			return single(decad.PatchEdges(nilBody, query).Do)
		}},
		{"Unstitch", decad.OperationUnstitch, subjectBody, func() func(context.Context) ([]*decad.Body, error) {
			return decad.Unstitch(nilBody).Do
		}},
		{"Trim", decad.OperationTrim, subjectBody, func() func(context.Context) ([]*decad.Body, error) {
			return single(decad.Trim(nilBody, nilBody, decad.KeepOutside).Do)
		}},
		{"Extend", decad.OperationExtend, subjectBody, func() func(context.Context) ([]*decad.Body, error) {
			return single(decad.Extend(nilBody, query, nilBody).Do)
		}},
		{"Split", decad.OperationSplit, subjectTarget, func() func(context.Context) ([]*decad.Body, error) {
			return decad.Split(nilBody, nilBody).Do
		}},
		{"SolidFromMesh", decad.OperationSolidFromMesh, "d", func() func(context.Context) ([]*decad.Body, error) {
			return single(decad.SolidFromMesh(nilDoc, nil).Do)
		}},
		{"Remove", decad.OperationRemove, subjectBody, func() func(context.Context) ([]*decad.Body, error) {
			cmd := decad.Remove(nilBody)
			return func(ctx context.Context) ([]*decad.Body, error) { return nil, cmd.Do(ctx) }
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
	t.Run("Stitch records nothing", func(t *testing.T) {
		t.Parallel()
		for name, cmd := range map[string]*decad.StitchCommand{
			"no body":     decad.Stitch(),
			"nil operand": decad.Stitch(nilBody),
		} {
			_, err := cmd.Do(t.Context())
			require.ErrorIs(t, err, decad.ErrDegenerate, name)
			require.Empty(t, requireOperationKind(t, err, decad.OperationStitch)[0].Subject.Name, name)
		}
	})
	t.Run("SolidFromMesh nil mesh", func(t *testing.T) {
		t.Parallel()
		_, err := decad.SolidFromMesh(decad.New(), nil).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		requireOperationKind(t, err, decad.OperationSolidFromMesh)
	})
}

// Every member refuses a retired body at its gate and tags the refusal with its
// own kind. The loop retires each live body of the member's fixture, then runs
// a fresh command over it (agent-docs/command-api-design.md section 14).
// SolidFromMesh has no body operand, so it has no such row.
func TestSurfaceCommandsRefuseARetiredBody(t *testing.T) {
	t.Parallel()
	for _, tc := range surfaceCommands() {
		if tc.noOperand {
			continue
		}
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			next, doc := tc.build(t)
			do := next()
			for _, b := range doc.Bodies() {
				require.NoError(t, decad.Remove(b).Do(t.Context()))
			}
			_, err := do(t.Context())
			require.ErrorIs(t, err, decad.ErrRetiredBody)
			requireOperationKind(t, err, tc.kind)
		})
	}
}

// A command copies the selector and slice it is given, so a later change to the
// caller's value reaches only the original (agent-docs/command-api-design.md
// rules A2 and A3).
func TestSurfaceCommandsCopyTheirArguments(t *testing.T) {
	t.Parallel()
	t.Run("PatchEdges selector", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		s, p := plateSketch(t)
		sheet, err := decad.PatchProfile(doc, s, p).Do(t.Context())
		require.NoError(t, err)
		sel := decad.Edges(decad.Free()).Exactly(4)
		cmd := decad.PatchEdges(sheet, sel)
		sel.Exactly(1) // a later change that would refuse the four edges.
		got, err := cmd.Do(t.Context())
		require.NoError(t, err)
		decadtest.MeasuresArea(t, got, units.SquareMillimeters(12000), decadtest.Exactly())
	})
	t.Run("Extend edge query", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ribbon, _ := extendLeftRibbon(t, doc)
		tool := trimSpanningTool(t, doc, 70, -10, 90, 10)
		query := decad.Edges(decad.Free(), decad.ParallelTo(r3.NewVec(0, 0, 1)),
			decad.EndpointAt(r3.NewVec(40, 0, 0))).Exactly(1)
		cmd := decad.Extend(ribbon, query, tool)
		query.Exactly(5)
		got, err := cmd.Do(t.Context())
		require.NoError(t, err)
		bounds, err := got.Bounds()
		require.NoError(t, err)
		require.InDelta(t, 70, bounds.Max.X, bounds.Bound.Base()+1e-9)
	})
	t.Run("Stitch bodies", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		walls, bottom, top := stitchBoxSheets(t, doc, 10)
		operands := []*decad.Body{walls, bottom, top}
		cmd := decad.Stitch(operands...)
		operands[0], operands[1], operands[2] = nil, nil, nil
		box, err := cmd.Do(t.Context())
		require.NoError(t, err, "the caller's later write to its slice does not reach the command")
		require.Equal(t, decad.BodySolid, box.Kind())
	})
}

// Each family member's refusal table holds at Do, in the old gate order: a nil
// body, a retired body, then the member's own checks.
func TestSurfaceCommandsKeepTheirGateOrder(t *testing.T) {
	t.Parallel()
	t.Run("Trim refuses an unknown side after the liveness gates", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		sheet := trimRectSheet(t, doc)
		tool := trimSpanningTool(t, doc, 40, -10, 60, 70)
		_, err := decad.Trim(sheet, tool, decad.TrimSide(7)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		requireOperationKind(t, err, decad.OperationTrim)
		require.Len(t, doc.Bodies(), 2, "a refusal retires neither operand")

		require.NoError(t, decad.Remove(tool).Do(t.Context()))
		_, err = decad.Trim(sheet, tool, decad.TrimSide(7)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrRetiredBody, "the retired-body gate precedes the side gate")
	})
	t.Run("Trim, Extend and Split refuse one body twice", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		sheet := trimRectSheet(t, doc)
		_, err := decad.Trim(sheet, sheet, decad.KeepOutside).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		_, err = decad.Extend(sheet, decad.Edges(decad.Free()), sheet).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		_, err = decad.Split(sheet, sheet).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("Remove of a retired body", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		block := decadtest.NewBlock(t, doc, 0, 0, 10, 10, units.Millimeters(3))
		require.NoError(t, decad.Remove(block).Do(t.Context()))
		err := decad.Remove(block).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrRetiredBody)
		requireOperationKind(t, err, decad.OperationRemove)
	})
}
