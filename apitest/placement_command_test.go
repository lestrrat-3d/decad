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

// The names a placement fault carries or the table rows share.
const (
	subjectBody   = "body"
	nameTransform = "Transform"
)

// placementCommand is one placement-and-patterns command built over a fixture
// whose result is known. next makes a fresh command over the one fixture, so a
// canceled run, which leaves the document unchanged, costs one command and no
// fixture. check asserts the computed geometry of a successful result.
//
// stride tunes the mid-build cancellation sweep only: it moves the cancel that
// many checks later per run, where one check at a time would cost minutes
// (agent-docs/command-api-design.md section 14).
type placementCommand struct {
	name   string
	kind   decad.OperationKind
	build  func(t *testing.T) (next func() func(context.Context) ([]*decad.Body, error), doc *decad.Document)
	check  func(t *testing.T, got []*decad.Body, doc *decad.Document)
	stride int64
}

// single adapts a one-result command to the shape of the table.
func single(do func(context.Context) (*decad.Body, error)) func(context.Context) ([]*decad.Body, error) {
	return func(ctx context.Context) ([]*decad.Body, error) {
		b, err := do(ctx)
		if b == nil {
			return nil, err
		}
		return []*decad.Body{b}, err
	}
}

func requirePlacementBounds(t *testing.T, b *decad.Body, lo, hi r3.Vec) {
	t.Helper()
	box, err := b.Bounds()
	require.NoError(t, err)
	require.True(t, box.Min.Equal(lo, 1e-9), "min %v, want %v", box.Min, lo)
	require.True(t, box.Max.Equal(hi, 1e-9), "max %v, want %v", box.Max, hi)
}

func requirePlacementVolume(t *testing.T, b *decad.Body, want float64) {
	t.Helper()
	v, err := b.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+1e-6)
}

func placementTranslation(t *testing.T, v r3.Vec) r3.Transform {
	t.Helper()
	tr, err := r3.Translation(v)
	require.NoError(t, err)
	return tr
}

func placementCommands() []placementCommand {
	mm := units.Millimeters
	return []placementCommand{
		{name: nameTransform, kind: decad.OperationTransform,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc := decad.New()
				block := decadtest.NewBlock(t, doc, 0, 0, 10, 10, mm(3))
				move := placementTranslation(t, r3.NewVec(5, 0, 0))
				return func() func(context.Context) ([]*decad.Body, error) {
					return single(decad.Transform(block, move).Do)
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				requirePlacementBounds(t, got[0], r3.NewVec(5, 0, 0), r3.NewVec(15, 10, 3))
				require.Equal(t, got, doc.Bodies(), "the source is retired")
			}},
		{name: "Copy", kind: decad.OperationCopy,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc := decad.New()
				block := decadtest.NewBlock(t, doc, 0, 0, 10, 10, mm(3))
				move := decad.CopyTransform{Transform: placementTranslation(t, r3.NewVec(20, 0, 0))}
				return func() func(context.Context) ([]*decad.Body, error) {
					return single(decad.Copy(block).Place(move).Do)
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				requirePlacementBounds(t, got[0], r3.NewVec(20, 0, 0), r3.NewVec(30, 10, 3))
				require.Len(t, doc.Bodies(), 2, "the source stays live")
			}},
		{name: "Mirrored", kind: decad.OperationMirrored,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				sc := newMirrorScene()
				box := sc.box(t, sc.w.XY(), 0, 0, 20, 10, mirrorAlong(10))
				plane := mirrorAtX30(t)
				return func() func(context.Context) ([]*decad.Body, error) {
					return single(decad.Mirrored(box, plane).Do)
				}, sc.doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				requirePlacementBounds(t, got[0], r3.NewVec(40, 0, 0), r3.NewVec(60, 10, 10))
				requirePlacementVolume(t, got[0], 2000)
				require.Equal(t, got, doc.Bodies(), "the source is retired")
			}},
		{name: "MirroredJoin", kind: decad.OperationMirrored,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				sc := newMirrorScene()
				l := sc.mirrorL(t)
				plane := decad.MirrorFace{Body: l, Face: decad.Faces(decad.Planar(), decad.Facing(joinMinusX))}
				return func() func(context.Context) ([]*decad.Body, error) {
					return single(decad.MirroredJoin(l, plane).Do)
				}, sc.doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				requirePlacementBounds(t, got[0], r3.NewVec(-20, 0, 0), r3.NewVec(20, 20, 10))
				requirePlacementVolume(t, got[0], 3500)
				require.Equal(t, got, doc.Bodies(), "the receiver is retired")
			}},
		{name: "MirroredCopy", kind: decad.OperationMirroredCopy,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				sc := newMirrorScene()
				box := sc.box(t, sc.w.XY(), 0, 0, 20, 10, mirrorAlong(10))
				plane := mirrorAtX30(t)
				return func() func(context.Context) ([]*decad.Body, error) {
					return single(decad.MirroredCopy(box, plane).Do)
				}, sc.doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				requirePlacementBounds(t, got[0], r3.NewVec(40, 0, 0), r3.NewVec(60, 10, 10))
				require.Len(t, doc.Bodies(), 2, "the source stays live")
			}},
		{name: "ScaledAt", kind: decad.OperationScaledAt,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc := decad.New()
				block := decadtest.NewBlock(t, doc, 0, 0, 10, 10, mm(3))
				return func() func(context.Context) ([]*decad.Body, error) {
					return single(decad.ScaledAt(block, r3.Vec{}, units.Scalar(2)).Do)
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				requirePlacementVolume(t, got[0], 10*10*3*8)
				require.Equal(t, got, doc.Bodies(), "the source is retired")
			}},
		{name: "QuarterTurn", kind: decad.OperationQuarterTurn,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc := decad.New()
				block := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
				return func() func(context.Context) ([]*decad.Body, error) {
					return single(decad.QuarterTurn(block, r3.NewVec(0, 0, 1), r3.Vec{}, 1).Do)
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				requirePlacementBounds(t, got[0], r3.NewVec(-8, 0, 0), r3.NewVec(0, 10, 6))
				requireVolume(t, got[0], 480)
				require.Equal(t, got, doc.Bodies(), "the source is retired")
			}},
		{name: "Stretch", kind: decad.OperationStretch,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc := decad.New()
				block := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
				return func() func(context.Context) ([]*decad.Body, error) {
					return single(decad.Stretch(block, r3.Vec{}, units.Scalar(2), units.Scalar(0.5), units.Scalar(3)).Do)
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				requirePlacementBounds(t, got[0], r3.NewVec(0, 0, 0), r3.NewVec(20, 4, 18))
				requireVolume(t, got[0], 480*3)
				require.Equal(t, got, doc.Bodies(), "the source is retired")
			}},
		{name: "PatternCopies", kind: decad.OperationPatternCopies, stride: 31,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc := decad.New()
				block := decadtest.NewBlock(t, doc, 0, 0, 10, 10, mm(3))
				spec := decad.LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: mm(20), Count: 3}
				return func() func(context.Context) ([]*decad.Body, error) {
					return decad.PatternCopies(block, spec).Do
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				require.Len(t, got, 2)
				requirePlacementBounds(t, got[0], r3.NewVec(20, 0, 0), r3.NewVec(30, 10, 3))
				requirePlacementBounds(t, got[1], r3.NewVec(40, 0, 0), r3.NewVec(50, 10, 3))
				require.Len(t, doc.Bodies(), 3, "the source stays live")
			}},
		{name: "Patterned", kind: decad.OperationPatterned, stride: 39,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc := decad.New()
				block := decadtest.NewBlock(t, doc, 0, 0, 10, 10, mm(3))
				spec := decad.LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: mm(20), Count: 3}
				return func() func(context.Context) ([]*decad.Body, error) {
					return single(decad.Patterned(block, spec).Do)
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				requirePlacementBounds(t, got[0], r3.NewVec(0, 0, 0), r3.NewVec(50, 10, 3))
				requirePlacementVolume(t, got[0], 3*10*10*3)
				require.Equal(t, got, doc.Bodies(), "the source is retired")
			}},
		{name: "PathPattern", kind: decad.OperationCopy, stride: 30,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc, block := patternBlock(t)
				path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 30}})
				require.NoError(t, err)
				return func() func(context.Context) ([]*decad.Body, error) {
					return decad.PathPattern(block, path, decad.PathCount{Count: 3}).Do
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				require.Len(t, got, 2)
				requirePlacementBounds(t, got[0], r3.NewVec(15, 0, 0), r3.NewVec(19, 4, 2))
				requirePlacementBounds(t, got[1], r3.NewVec(30, 0, 0), r3.NewVec(34, 4, 2))
				require.Len(t, doc.Bodies(), 3, "the source stays live")
			}},
		{name: "Patterned union fallback", kind: decad.OperationPatterned,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				// A mesh solid is no prism group, and its two instances overlap, so
				// the instances combine by Union (pattern.go, rule 3).
				v, tris := cubeMesh(r3.Vec{}, r3.NewVec(10, 10, 10))
				m, err := decad.NewMeshBody(t.Context(), v, tris)
				require.NoError(t, err)
				doc := decad.New()
				cube, err := decad.SolidFromMesh(doc, m).Do(t.Context())
				require.NoError(t, err)
				spec := decad.LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: mm(5), Count: 2}
				return func() func(context.Context) ([]*decad.Body, error) {
					return single(decad.Patterned(cube, spec).Do)
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				requirePlacementBounds(t, got[0], r3.NewVec(0, 0, 0), r3.NewVec(15, 10, 10))
				requirePlacementVolume(t, got[0], 1500)
				require.Equal(t, got, doc.Bodies(), "the source is retired")
			}},
		{name: "PathPattern follow", kind: decad.OperationCopy,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc, block := patternBlock(t)
				path := followPath(t)
				return func() func(context.Context) ([]*decad.Body, error) {
					return decad.PathPattern(block, path, decad.PathCount{Count: 3}).Follow().Do
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				require.Len(t, got, 2)
				lo, hi := turnedBlockBounds(math.Pi/2, r3.Vec{X: 20, Y: 10})
				requireBoundsNear(t, got[1], lo, hi)
				require.Len(t, doc.Bodies(), 3, "the source stays live")
			}},
		{name: "Arrange", kind: decad.OperationTransform, stride: 30,
			build: func(t *testing.T) (func() func(context.Context) ([]*decad.Body, error), *decad.Document) {
				doc := decad.New()
				blocks := arrangeBlocks(t, doc, [][2]float64{{10, 5}, {10, 8}})
				return func() func(context.Context) ([]*decad.Body, error) {
					return decad.Arrange(blocks, arrangeBed(30, 20, 2)).Do
				}, doc
			},
			check: func(t *testing.T, got []*decad.Body, doc *decad.Document) {
				require.Len(t, got, 2)
				for _, b := range got {
					box, err := b.Bounds()
					require.NoError(t, err)
					require.GreaterOrEqual(t, box.Min.X, 100.0)
					require.GreaterOrEqual(t, box.Min.Y, 200.0)
					require.InDelta(t, 5, box.Min.Z, 1e-9, "the bodies rest on the bed")
				}
				require.Len(t, doc.Bodies(), 2)
			}},
	}
}

func TestPlacementCommandsRunOnceAndConsume(t *testing.T) {
	t.Parallel()
	for _, tc := range placementCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			next, doc := tc.build(t)
			do := next()
			got, err := do(t.Context())
			require.NoError(t, err)
			tc.check(t, got, doc)

			bodies := doc.Bodies()
			again, err := do(t.Context())
			require.ErrorIs(t, err, decad.ErrConsumed)
			require.Nil(t, again)
			first := requireOperationKind(t, err, tc.kind)[0]
			require.Equal(t, decad.OpDiagInputRejected, first.Code)
			require.Equal(t, decad.OperationStageInput, first.Stage)
			require.Empty(t, first.Subject.Name)
			require.Equal(t, bodies, doc.Bodies(), "a consumed command changes nothing")
		})
	}
}

func TestPlacementCommandsRefuseNilContext(t *testing.T) {
	t.Parallel()
	for _, tc := range placementCommands() {
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

func TestPlacementCommandsReturnCancellationByIdentity(t *testing.T) {
	t.Parallel()
	for _, tc := range placementCommands() {
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
func TestPlacementCommandsReturnMidBuildCancellationByIdentity(t *testing.T) {
	t.Parallel()
	const maxChecks = 6000
	for _, tc := range placementCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			next, doc := tc.build(t)
			step := max(tc.stride, 1)
			before := doc.Bodies()
			for n := int64(0); n < maxChecks; n += step {
				got, err := next()(newCancelAfter(t.Context(), n))
				if err == nil {
					require.NotNil(t, got)
					require.Greater(t, n, int64(1), "the cancel never reached the build")
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

func slicesEqualBodies(a, b []*decad.Body) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

// A nil body is recorded by the constructor and named after its parameter, and
// a nil ctx is still refused ahead of it.
func TestPlacementConstructorsRecordNilOperands(t *testing.T) {
	t.Parallel()
	var nilBody *decad.Body
	plane := mirrorAtX30(t)
	spec := decad.LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: units.Millimeters(1), Count: 2}
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 30}})
	require.NoError(t, err)
	_, block := patternBlock(t)
	bed := arrangeBed(30, 20, 2)
	rows := []struct {
		name    string
		kind    decad.OperationKind
		subject string
		// next builds a fresh command, so the table serves each subtest once.
		next func() func(context.Context) ([]*decad.Body, error)
	}{
		{nameTransform, decad.OperationTransform, subjectBody,
			func() func(context.Context) ([]*decad.Body, error) {
				return single(decad.Transform(nilBody, r3.Identity()).Do)
			}},
		{"Mirrored", decad.OperationMirrored, subjectBody,
			func() func(context.Context) ([]*decad.Body, error) { return single(decad.Mirrored(nilBody, plane).Do) }},
		{"MirroredJoin", decad.OperationMirrored, subjectBody,
			func() func(context.Context) ([]*decad.Body, error) {
				return single(decad.MirroredJoin(nilBody, plane).Do)
			}},
		{"MirroredCopy", decad.OperationMirroredCopy, subjectBody,
			func() func(context.Context) ([]*decad.Body, error) {
				return single(decad.MirroredCopy(nilBody, plane).Do)
			}},
		{"ScaledAt", decad.OperationScaledAt, subjectBody, func() func(context.Context) ([]*decad.Body, error) {
			return single(decad.ScaledAt(nilBody, r3.Vec{}, units.Scalar(2)).Do)
		}},
		{"QuarterTurn", decad.OperationQuarterTurn, subjectBody, func() func(context.Context) ([]*decad.Body, error) {
			return single(decad.QuarterTurn(nilBody, r3.NewVec(0, 0, 1), r3.Vec{}, 1).Do)
		}},
		{"Stretch", decad.OperationStretch, subjectBody, func() func(context.Context) ([]*decad.Body, error) {
			return single(decad.Stretch(nilBody, r3.Vec{}, units.Scalar(1), units.Scalar(1), units.Scalar(1)).Do)
		}},
		{"PatternCopies", decad.OperationPatternCopies, subjectBody,
			func() func(context.Context) ([]*decad.Body, error) { return decad.PatternCopies(nilBody, spec).Do }},
		{"Patterned", decad.OperationPatterned, subjectBody,
			func() func(context.Context) ([]*decad.Body, error) { return single(decad.Patterned(nilBody, spec).Do) }},
		{"PathPattern body", decad.OperationCopy, subjectBody, func() func(context.Context) ([]*decad.Body, error) {
			return decad.PathPattern(nilBody, path, decad.PathCount{Count: 3}).Do
		}},
		{"PathPattern path", decad.OperationCopy, "path", func() func(context.Context) ([]*decad.Body, error) {
			return decad.PathPattern(block, nil, decad.PathCount{Count: 3}).Do
		}},
		{"Arrange none", decad.OperationTransform, "bodies",
			func() func(context.Context) ([]*decad.Body, error) { return decad.Arrange(nil, bed).Do }},
		{"Arrange nil first", decad.OperationTransform, "bodies", func() func(context.Context) ([]*decad.Body, error) {
			return decad.Arrange([]*decad.Body{nil, block}, bed).Do
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

// Every member refuses a retired body at its gate and tags the refusal with its
// own kind. The loop retires each live body of the member's fixture, then runs
// a fresh command over it (agent-docs/command-api-design.md section 14).
func TestPlacementCommandsRefuseARetiredBody(t *testing.T) {
	t.Parallel()
	for _, tc := range placementCommands() {
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

// Copy does not record a nil source: Copy's own gate order reads the placement
// first, so a bad placement beside a nil source reports the placement.
func TestCopyReportsItsPlacementBeforeANilSource(t *testing.T) {
	t.Parallel()
	_, err := decad.Copy(nil).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	requireOperationKind(t, err, decad.OperationCopy)

	_, err = decad.Copy(nil).Place(nil).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, "Place", requireOperationKind(t, err, decad.OperationCopy)[0].Subject.Name)
}

// Each stored method fault is refused where today's option decode refused it,
// and a nil ctx and a consumed command come first.
func TestPlacementCommandsRefuseMethodFaultsAtTheirGate(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	badPlacement := decad.CopyRotation{Axis: r3.Vec{}, Angle: units.Degrees(10)}

	t.Run("Copy.Place is refused before the source is read", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		block := decadtest.NewBlock(t, doc, 0, 0, 10, 10, mm(3))
		retired, err := decad.Transform(block, r3.Identity()).Do(t.Context())
		require.NoError(t, err)
		require.NotNil(t, retired)

		_, err = decad.Copy(block).Place(badPlacement).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate, "the placement is read before the retired-body gate")
		require.NotErrorIs(t, err, decad.ErrRetiredBody)
		require.Equal(t, "Place", requireOperationKind(t, err, decad.OperationCopy)[0].Subject.Name)

		_, err = decad.Copy(block).Place(decad.CopyTransform{Transform: r3.Identity()}).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrRetiredBody, "a valid placement reaches the retired-body gate")

		var nilCtx context.Context
		_, err = decad.Copy(block).Place(badPlacement).Do(nilCtx)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Empty(t, requireOperationKind(t, err, decad.OperationCopy)[0].Subject.Name, "a nil ctx precedes the placement fault")
	})

	t.Run("PathPattern.FollowAbout is refused after stations and before liveness", func(t *testing.T) {
		t.Parallel()
		doc, block := patternBlock(t)
		path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 30}})
		require.NoError(t, err)
		nan := r3.Vec{X: math.NaN()}

		_, err = decad.PathPattern(block, path, decad.PathCount{Count: 1}).FollowAbout(nan).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate, "the stations fault precedes the pivot fault")
		require.NotErrorIs(t, err, decad.ErrNotFinite)

		_, err = decad.PathPattern(block, path, decad.PathCount{Count: 3}).FollowAbout(nan).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrNotFinite)
		require.Equal(t, "FollowAbout", requireOperationKind(t, err, decad.OperationCopy)[0].Subject.Name)
		require.Equal(t, []*decad.Body{block}, doc.Bodies())

		retiredDoc, retiredBlock := patternBlock(t)
		moved, err := decad.Transform(retiredBlock, r3.Identity()).Do(t.Context())
		require.NoError(t, err)
		require.NotNil(t, moved)
		_, err = decad.PathPattern(retiredBlock, path, decad.PathCount{Count: 3}).FollowAbout(nan).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrNotFinite, "the pivot is read before the retired-body gate")
		_, err = decad.PathPattern(retiredBlock, path, decad.PathCount{Count: 3}).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrRetiredBody)
		require.Equal(t, []*decad.Body{moved}, retiredDoc.Bodies())

		var nilCtx context.Context
		_, err = decad.PathPattern(block, path, decad.PathCount{Count: 3}).FollowAbout(nan).Do(nilCtx)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.NotErrorIs(t, err, decad.ErrNotFinite, "a nil ctx precedes the pivot fault")
	})

	t.Run("Arrange.Angle is refused before liveness and the bed", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		blocks := arrangeBlocks(t, doc, [][2]float64{{10, 5}})
		badBed := decad.ArrangeBed{Width: mm(-1), Depth: mm(20), Gap: mm(1)}

		_, err := decad.Arrange(blocks, badBed).Angle(units.Degrees(0)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Equal(t, "Angle", requireOperationKind(t, err, decad.OperationTransform)[0].Subject.Name,
			"the Angle fault precedes the bed fault")
		_, err = decad.Arrange(blocks, badBed).Angle(mm(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnitKind)
		_, err = decad.Arrange(blocks, badBed).Angle(units.Degrees(-5)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
		_, err = decad.Arrange(blocks, badBed).Angle(units.Degrees(0.1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate, "a step finer than a quarter degree")
		_, err = decad.Arrange(blocks, badBed).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrNegativeMagnitude, "without the fault the bed's own gate speaks")
		require.Equal(t, blocks, doc.Bodies())

		moved, err := decad.Transform(blocks[0], r3.Identity()).Do(t.Context())
		require.NoError(t, err)
		require.NotNil(t, moved)
		_, err = decad.Arrange(blocks, arrangeBed(30, 20, 2)).Angle(units.Degrees(0)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.NotErrorIs(t, err, decad.ErrRetiredBody, "the Angle fault precedes the retired-body gate")
		_, err = decad.Arrange(blocks, arrangeBed(30, 20, 2)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrRetiredBody)

		var nilCtx context.Context
		_, err = decad.Arrange(blocks, badBed).Angle(units.Degrees(0)).Do(nilCtx)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Empty(t, requireOperationKind(t, err, decad.OperationTransform)[0].Subject.Name)
	})
}

// A second call to a method replaces the first, so an invalid earlier call is
// forgotten along with its fault (agent-docs/command-api-design.md rule M2).
func TestPlacementMethodsLastCallWins(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	t.Run("Copy.Place", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		block := decadtest.NewBlock(t, doc, 0, 0, 10, 10, mm(3))
		move := decad.CopyTransform{Transform: placementTranslation(t, r3.NewVec(20, 0, 0))}
		got, err := decad.Copy(block).Place(decad.CopyRotation{}).Place(move).Do(t.Context())
		require.NoError(t, err, "a valid call replaces an invalid one")
		requirePlacementBounds(t, got, r3.NewVec(20, 0, 0), r3.NewVec(30, 10, 3))

		_, err = decad.Copy(block).Place(move).Place(decad.CopyRotation{}).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate, "an invalid call replaces a valid one")
	})
	t.Run("PathPattern.Follow and FollowAbout", func(t *testing.T) {
		t.Parallel()
		_, block := patternBlock(t)
		nan := r3.Vec{X: math.NaN()}
		copies, err := decad.PathPattern(block, followPath(t), decad.PathCount{Count: 3}).FollowAbout(nan).Follow().Do(t.Context())
		require.NoError(t, err, "Follow replaces the invalid pivot")
		lo, hi := turnedBlockBounds(math.Pi/2, r3.Vec{X: 20, Y: 10})
		requireBoundsNear(t, copies[1], lo, hi)

		_, err = decad.PathPattern(block, followPath(t), decad.PathCount{Count: 3}).Follow().FollowAbout(nan).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrNotFinite, "FollowAbout replaces Follow")

		plain, err := decad.PathPattern(block, followPath(t), decad.PathCount{Count: 3}).FollowAbout(r3.Vec{X: 2, Y: 2}).Follow().Do(t.Context())
		require.NoError(t, err)
		requireBoundsNear(t, plain[1], r3.Vec{X: 16, Y: 10, Z: 0}, r3.Vec{X: 20, Y: 14, Z: 2})
	})
	t.Run("Arrange.Angle", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		blocks := arrangeBlocks(t, doc, [][2]float64{{10, 5}})
		out, err := decad.Arrange(blocks, arrangeBed(30, 20, 2)).Angle(units.Degrees(0)).Angle(units.Degrees(45)).Do(t.Context())
		require.NoError(t, err, "a valid step replaces an invalid one")
		require.Len(t, out, 1)
		_, err = decad.Arrange(out, arrangeBed(30, 20, 2)).Angle(units.Degrees(45)).Angle(mm(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnitKind, "an invalid step replaces a valid one")
	})
	t.Run("Arrange.Turns is idempotent", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		blocks := arrangeBlocks(t, doc, [][2]float64{{5, 20}, {5, 12}})
		out, err := decad.Arrange(blocks, arrangeBed(30, 20, 2)).Turns().Turns().Do(t.Context())
		require.NoError(t, err)
		require.Len(t, out, 2)
	})
}

// A command copies the selector, slice and sealed values it is given, so a
// later change to the caller's value reaches only the original
// (agent-docs/command-api-design.md rules A2 and A3).
func TestPlacementCommandsCopyTheirArguments(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	t.Run("MirrorFace selector", func(t *testing.T) {
		t.Parallel()
		for name, build := range map[string]func(sc *mirrorScene, l *decad.Body, plane decad.MirrorFace) func(context.Context) (*decad.Body, error){
			"Mirrored": func(_ *mirrorScene, l *decad.Body, plane decad.MirrorFace) func(context.Context) (*decad.Body, error) {
				return decad.Mirrored(l, plane).Do
			},
			"MirroredCopy": func(_ *mirrorScene, l *decad.Body, plane decad.MirrorFace) func(context.Context) (*decad.Body, error) {
				return decad.MirroredCopy(l, plane).Do
			},
			"MirroredJoin": func(_ *mirrorScene, l *decad.Body, plane decad.MirrorFace) func(context.Context) (*decad.Body, error) {
				return decad.MirroredJoin(l, plane).Do
			},
			"MirroredJoin pointer": func(_ *mirrorScene, l *decad.Body, plane decad.MirrorFace) func(context.Context) (*decad.Body, error) {
				return decad.MirroredJoin(l, &plane).Do
			},
		} {
			sc := newMirrorScene()
			l := sc.mirrorL(t)
			q := decad.Faces(decad.Planar(), decad.Facing(joinMinusX))
			plane := decad.MirrorFace{Body: l, Face: q}
			do := build(sc, l, plane)
			// A query that now asserts exactly 7 faces would refuse the call.
			q.Exactly(7)
			got, err := do(t.Context())
			require.NoError(t, err, name)
			require.NotNil(t, got, name)
		}
	})
	t.Run("MirrorFrame pointer", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		box := sc.box(t, sc.w.XY(), 0, 0, 20, 10, mirrorAlong(10))
		frame := mirrorAtX30(t)
		cmd := decad.Mirrored(box, &frame)
		frame.Frame = r3.Frame{}
		got, err := cmd.Do(t.Context())
		require.NoError(t, err, "the caller's later write to the frame does not reach the command")
		requirePlacementBounds(t, got, r3.NewVec(40, 0, 0), r3.NewVec(60, 10, 10))
	})
	t.Run("Arrange bodies", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		blocks := arrangeBlocks(t, doc, [][2]float64{{10, 5}, {10, 8}})
		keep := []*decad.Body{blocks[0], blocks[1]}
		cmd := decad.Arrange(blocks, arrangeBed(30, 20, 2))
		blocks[0], blocks[1] = nil, nil
		out, err := cmd.Do(t.Context())
		require.NoError(t, err, "the caller's later write to its slice does not reach the command")
		require.Len(t, out, 2)
		require.NotContains(t, doc.Bodies(), keep[0])
		require.NotContains(t, doc.Bodies(), keep[1])
	})
	t.Run("pattern spec pointer", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		block := decadtest.NewBlock(t, doc, 0, 0, 10, 10, mm(3))
		spec := &decad.LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: mm(20), Count: 3}
		copies := decad.PatternCopies(block, spec)
		joined := decad.Patterned(block, spec)
		spec.Count = 1
		got, err := copies.Do(t.Context())
		require.NoError(t, err)
		require.Len(t, got, 2, "the count the command was built with")
		_, err = joined.Do(t.Context())
		require.NoError(t, err)
	})
	t.Run("path stations pointer", func(t *testing.T) {
		t.Parallel()
		_, block := patternBlock(t)
		path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.Vec{X: 30}})
		require.NoError(t, err)
		stations := &decad.PathCount{Count: 3}
		cmd := decad.PathPattern(block, path, stations)
		stations.Count = 1
		got, err := cmd.Do(t.Context())
		require.NoError(t, err)
		require.Len(t, got, 2)
	})
	t.Run("copy placement pointer", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		block := decadtest.NewBlock(t, doc, 0, 0, 10, 10, mm(3))
		place := &decad.CopyTransform{Transform: placementTranslation(t, r3.NewVec(20, 0, 0))}
		cmd := decad.Copy(block).Place(place)
		place.Transform = r3.Transform{}
		got, err := cmd.Do(t.Context())
		require.NoError(t, err, "the placement is read when Place is called")
		requirePlacementBounds(t, got, r3.NewVec(20, 0, 0), r3.NewVec(30, 10, 3))
	})
}

// Copy takes each of the three placements, as a value and as a pointer, and a
// nil pointer is the same refusal a nil placement is.
func TestCopyPlacementVariants(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	doc := decad.New()
	block := decadtest.NewBlock(t, doc, 0, 0, 10, 10, mm(3))
	move := placementTranslation(t, r3.NewVec(20, 0, 0))
	for name, place := range map[string]decad.CopyPlacement{
		"transform":             decad.CopyTransform{Transform: move},
		"transform pointer":     &decad.CopyTransform{Transform: move},
		"rotation":              decad.CopyRotation{Center: r3.NewVec(5, 5, 0), Axis: r3.NewVec(0, 0, 1), Angle: units.Degrees(90)},
		"rotation pointer":      &decad.CopyRotation{Center: r3.NewVec(5, 5, 0), Axis: r3.NewVec(0, 0, 1), Angle: units.Degrees(90)},
		"lift translation":      decad.CopyLiftTranslation{Lift: r3.Identity(), From: r3.Vec{}, To: r3.NewVec(20, 0, 0)},
		"lift translation ptr":  &decad.CopyLiftTranslation{Lift: r3.Identity(), From: r3.Vec{}, To: r3.NewVec(20, 0, 0)},
		"identity translation":  decad.CopyLiftTranslation{Lift: r3.Identity(), From: r3.NewVec(1, 1, 1), To: r3.NewVec(1, 1, 1)},
		"zero angle is a no-op": decad.CopyRotation{Axis: r3.NewVec(0, 0, 1), Angle: units.Degrees(0)},
	} {
		got, err := decad.Copy(block).Place(place).Do(t.Context())
		require.NoError(t, err, name)
		requirePlacementVolume(t, got, 300)
	}
	for name, place := range map[string]decad.CopyPlacement{
		"nil":                  nil,
		"nil transform":        (*decad.CopyTransform)(nil),
		"nil rotation":         (*decad.CopyRotation)(nil),
		"nil lift translation": (*decad.CopyLiftTranslation)(nil),
		"zero rotation axis":   decad.CopyRotation{Angle: units.Degrees(10)},
		"bad angle unit":       decad.CopyRotation{Axis: r3.NewVec(0, 0, 1), Angle: mm(1)},
		"zero lift":            decad.CopyLiftTranslation{},
	} {
		_, err := decad.Copy(block).Place(place).Do(t.Context())
		require.Error(t, err, name)
		require.Equal(t, "Place", requireOperationKind(t, err, decad.OperationCopy)[0].Subject.Name, name)
	}
	// A zero transform is the source's gate to refuse, after the retired-body
	// gate, so it names no method.
	_, err := decad.Copy(block).Place(decad.CopyTransform{}).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Empty(t, requireOperationKind(t, err, decad.OperationCopy)[0].Subject.Name)
}
