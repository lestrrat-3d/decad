package apitest_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// booleanCommand is one boolean command built over a fresh pair of
// overlapping 10 mm cubes, the second shifted by 5 mm along each axis, so each
// command's result volume is known: the union 1875, the cut 875, the
// intersection 125 and the refinement of the union's mesh the same 1875.
type booleanCommand struct {
	name   string
	kind   decad.OperationKind
	volume float64
	build  func(t *testing.T, doc *decad.Document) (do func(context.Context) (*decad.Body, error), operands []*decad.Body)
}

func booleanCommands() []booleanCommand {
	pair := func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
		return boxBody(t, doc, 0, 0, 10, 10, 10), translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)
	}
	return []booleanCommand{
		{name: "Union", kind: decad.OperationUnion, volume: 1875,
			build: func(t *testing.T, doc *decad.Document) (func(context.Context) (*decad.Body, error), []*decad.Body) {
				a, b := pair(t, doc)
				return decad.Union(a, b).Do, []*decad.Body{a, b}
			}},
		{name: "Cut", kind: decad.OperationCut, volume: 875,
			build: func(t *testing.T, doc *decad.Document) (func(context.Context) (*decad.Body, error), []*decad.Body) {
				a, b := pair(t, doc)
				return decad.Cut(a, b).Do, []*decad.Body{a, b}
			}},
		{name: "Intersect", kind: decad.OperationIntersect, volume: 125,
			build: func(t *testing.T, doc *decad.Document) (func(context.Context) (*decad.Body, error), []*decad.Body) {
				a, b := pair(t, doc)
				return decad.Intersect(a, b).Do, []*decad.Body{a, b}
			}},
		{name: "Refine", kind: decad.OperationRefine, volume: 1875,
			build: func(t *testing.T, doc *decad.Document) (func(context.Context) (*decad.Body, error), []*decad.Body) {
				a, b := pair(t, doc)
				joined, err := decad.Union(a, b).Do(t.Context())
				require.NoError(t, err)
				return decad.Refine(joined, units.Millimeters(0.1)).Do, []*decad.Body{joined}
			}},
	}
}

// requireOperationKind asserts every diagnostic of err names kind, and that
// err carries one, without dumping the error graph on failure.
func requireOperationKind(t *testing.T, err error, kind decad.OperationKind) []decad.OperationDiagnostic {
	t.Helper()
	diagnosed, ok := errors.AsType[decad.OperationDiagnosticError](err)
	require.True(t, ok, "the error must carry operation diagnostics")
	diagnostics := diagnosed.OperationDiagnostics()
	require.NotEmpty(t, diagnostics)
	for _, d := range diagnostics {
		require.Equal(t, kind, d.Operation)
	}
	return diagnostics
}

func TestBooleanCommandsRunOnceAndConsume(t *testing.T) {
	t.Parallel()
	for _, tc := range booleanCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			do, _ := tc.build(t, doc)
			got, err := do(t.Context())
			require.NoError(t, err)
			vol, verr := got.Volume()
			require.NoError(t, verr)
			require.Equal(t, tc.volume, volumeMM(t, vol))
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

func TestBooleanCommandsRefuseNilContextBeforeAnyOperand(t *testing.T) {
	t.Parallel()
	for _, tc := range booleanCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			do, _ := tc.build(t, doc)
			before := doc.Bodies()
			var nilCtx context.Context
			got, err := do(nilCtx)
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.Nil(t, got)
			requireOperationKind(t, err, tc.kind)
			require.Equal(t, before, doc.Bodies())

			// The refused Do consumed the command.
			_, err = do(t.Context())
			require.ErrorIs(t, err, decad.ErrConsumed)
		})
	}

	t.Run("nil context precedes a recorded fault", func(t *testing.T) {
		t.Parallel()
		var nilCtx context.Context
		_, err := decad.Union().Do(nilCtx)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		diagnostics := requireOperationKind(t, err, decad.OperationUnion)
		require.Empty(t, diagnostics[0].Subject.Name, "a nil context is no fault of the bodies argument")
	})
}

func TestBooleanCommandsReturnCancellationByIdentity(t *testing.T) {
	t.Parallel()
	for _, tc := range booleanCommands() {
		if tc.name == "Refine" {
			continue // its operand is built by a Union that the canceled context would stop first.
		}
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			do, _ := tc.build(t, doc)
			before := doc.Bodies()
			ctx, cancel := context.WithCancel(t.Context())
			cancel()
			_, err := do(ctx)
			require.Equal(t, context.Canceled, err)
			require.Equal(t, before, doc.Bodies())
		})
	}
}

func TestUnionCommandRecordsFewerThanTwoBodies(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	for _, bodies := range [][]*decad.Body{nil, {a}} {
		got, err := decad.Union(bodies...).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Nil(t, got)
		diagnostics := requireOperationKind(t, err, decad.OperationUnion)
		require.Equal(t, "bodies", diagnostics[0].Subject.Name)
		require.Equal(t, []*decad.Body{a}, doc.Bodies())
	}
}

func TestUnionCommandCopiesItsBodies(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)
	c := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 100, 100, 100)
	operands := []*decad.Body{a, b}
	cmd := decad.Union(operands...)
	operands[1] = c // a later write to the caller's slice changes nothing.
	got, err := cmd.Do(t.Context())
	require.NoError(t, err)
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, 1875.0, volumeMM(t, vol))
	require.ElementsMatch(t, []*decad.Body{got, c}, doc.Bodies())
}

func TestUnionRefineToBoundFaults(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name  string
		bound units.Value
		want  error
	}{
		{"an angle", units.Radians(0.05), decad.ErrUnitKind},
		{"not a number", units.Millimeters(math.NaN()), decad.ErrNotFinite},
		{"below zero", units.Millimeters(-0.05), decad.ErrNegativeMagnitude},
		{"exactly zero", units.Millimeters(0), decad.ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			a := boxBody(t, doc, 0, 0, 10, 10, 10)
			b := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)
			before := doc.Bodies()
			got, err := decad.Union(a, b).RefineTo(tc.bound).Do(t.Context())
			require.ErrorIs(t, err, tc.want)
			require.Nil(t, got)
			diagnostics := requireOperationKind(t, err, decad.OperationUnion)
			require.Equal(t, "RefineTo", diagnostics[0].Subject.Name)
			require.Equal(t, before, doc.Bodies())
		})
	}

	t.Run("the constructor's fault wins", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		a := boxBody(t, doc, 0, 0, 10, 10, 10)
		_, err := decad.Union(a).RefineTo(units.Radians(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.NotErrorIs(t, err, decad.ErrUnitKind)
		diagnostics := requireOperationKind(t, err, decad.OperationUnion)
		require.Equal(t, "bodies", diagnostics[0].Subject.Name)
	})

	t.Run("a second RefineTo replaces the first", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		a := boxBody(t, doc, 0, 0, 10, 10, 10)
		b := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)
		// A pair is no sector join, so a valid bound reaches the build gate and is
		// refused there as unsupported, not as a bound fault.
		_, err := decad.Union(a, b).RefineTo(units.Radians(1)).RefineTo(units.Millimeters(0.05)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.NotErrorIs(t, err, decad.ErrUnitKind)
		requireOperationKind(t, err, decad.OperationUnion)
	})
}

func TestBooleanMatchingFacesIsAMethodOfCutAndIntersect(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)
	// The operands' faces do not meet within rounding, so the declaration
	// changes nothing: the result is the plain intersection.
	got, err := decad.Intersect(a, b).MatchingFaces().Do(t.Context())
	require.NoError(t, err)
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, 125.0, volumeMM(t, vol))

	c := boxBody(t, doc, 0, 0, 10, 10, 10)
	d := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)
	cut, err := decad.Cut(c, d).MatchingFaces().MatchingFaces().Do(t.Context())
	require.NoError(t, err)
	vol, err = cut.Volume()
	require.NoError(t, err)
	require.Equal(t, 875.0, volumeMM(t, vol))
}
