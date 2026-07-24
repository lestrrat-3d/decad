package decad_test

import (
	"context"
	"runtime"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type cancelWhenInFrameContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	target          string
	limit           int
	targetCalls     int
	entered         bool
}

func (c *cancelWhenInFrameContext) Err() error {
	pcs := make([]uintptr, 32)
	frames := runtime.CallersFrames(pcs[:runtime.Callers(2, pcs)])
	inTarget := false
	for {
		frame, more := frames.Next()
		inTarget = inTarget || strings.HasSuffix(frame.Function, "."+c.target)
		if !more {
			break
		}
	}
	if inTarget {
		c.entered = true
		c.targetCalls++
		if c.targetCalls >= c.limit {
			return context.Canceled
		}
	}
	return c.Context.Err()
}

type cancelOnSecondDirectCallContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	target          string
	targetCalls     int
}

func (c *cancelOnSecondDirectCallContext) Err() error {
	pcs := make([]uintptr, 32)
	frames := runtime.CallersFrames(pcs[:runtime.Callers(2, pcs)])
	frame, _ := frames.Next()
	if strings.HasSuffix(frame.Function, "."+c.target) {
		c.targetCalls++
		if c.targetCalls == 2 {
			return context.Canceled
		}
	}
	return c.Context.Err()
}

func facetedContextBody(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	tool := translated(t, diskBody(t, doc, 14, 6, 2), 0, 0, -6)
	body, err := decad.Cut(plate, tool)
	require.NoError(t, err)
	return doc, body
}

func TestPlacementContextVariantsCancelFacetedRebuild(t *testing.T) {
	shift, err := r3.Translation(r3.NewVec(100, 0, 0))
	require.NoError(t, err)

	tests := []struct {
		name string
		run  func(context.Context, *decad.Body) (*decad.Body, error)
	}{
		{
			name: "Placed",
			run: func(ctx context.Context, body *decad.Body) (*decad.Body, error) {
				return body.PlacedContext(ctx, shift)
			},
		},
		{
			name: "Duplicate",
			run: func(ctx context.Context, body *decad.Body) (*decad.Body, error) {
				return body.DuplicateContext(ctx)
			},
		},
		{
			name: "PlacedCopy",
			run: func(ctx context.Context, body *decad.Body) (*decad.Body, error) {
				return body.PlacedCopyContext(ctx, shift)
			},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			doc, body := facetedContextBody(t)
			beforeBodies := doc.Bodies()
			beforeRecipe := doc.Recipe()
			ctx := newCancelAfterContext(t.Context(), 3)

			got, err := test.run(ctx, body)
			require.ErrorIs(t, err, context.Canceled)
			require.Nil(t, got)
			require.GreaterOrEqual(t, ctx.calls.Load(), int32(3),
				`cancellation must reach the faceted payload rebuild`)
			require.Equal(t, beforeBodies, doc.Bodies(),
				`a canceled rebuild must not change live bodies`)
			require.Equal(t, beforeRecipe, doc.Recipe(),
				`a canceled rebuild must not append a recipe step`)
		})
	}
}

func TestPlacementContextChecksCancellationBeforeCommit(t *testing.T) {
	shift, err := r3.Translation(r3.NewVec(100, 0, 0))
	require.NoError(t, err)

	tests := []struct {
		name   string
		target string
		run    func(context.Context, *decad.Body) (*decad.Body, error)
	}{
		{
			name:   "Placed",
			target: "PlacedContext",
			run: func(ctx context.Context, body *decad.Body) (*decad.Body, error) {
				return body.PlacedContext(ctx, shift)
			},
		},
		{
			name:   "Duplicate",
			target: "copyUnder",
			run: func(ctx context.Context, body *decad.Body) (*decad.Body, error) {
				return body.DuplicateContext(ctx)
			},
		},
		{
			name:   "PlacedCopy",
			target: "copyUnder",
			run: func(ctx context.Context, body *decad.Body) (*decad.Body, error) {
				return body.PlacedCopyContext(ctx, shift)
			},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			doc, body := facetedContextBody(t)
			beforeBodies := doc.Bodies()
			beforeRecipe := doc.Recipe()
			ctx := &cancelOnSecondDirectCallContext{
				Context: t.Context(),
				target:  test.target,
			}

			got, err := test.run(ctx, body)
			require.ErrorIs(t, err, context.Canceled)
			require.Nil(t, got)
			require.Equal(t, beforeBodies, doc.Bodies(),
				`cancellation after rebuild must not change live bodies`)
			require.Equal(t, beforeRecipe, doc.Recipe(),
				`cancellation after rebuild must not append a recipe step`)
			require.Equal(t, 2, ctx.targetCalls,
				`the second direct gate must observe cancellation before commit`)
		})
	}
}

func TestPlacementContextCancelsAnalyticRebuilds(t *testing.T) {
	shift, err := r3.Translation(r3.NewVec(100, 0, 0))
	require.NoError(t, err)

	tests := []struct {
		name  string
		build func(*testing.T) (*decad.Document, *decad.Body)
	}{
		{
			name: "Extrude",
			build: func(t *testing.T) (*decad.Document, *decad.Body) {
				s, p := plateSketch(t)
				doc := decad.New()
				body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
				require.NoError(t, err)
				return doc, body
			},
		},
		{
			name: "Revolve",
			build: func(t *testing.T) (*decad.Document, *decad.Body) {
				s, p := solidSketch(t)
				doc := decad.New()
				body, err := doc.Revolve(s, p, decad.SketchLine{
					Start: decad.Point2{U: 0, V: 0},
					End:   decad.Point2{U: 1, V: 0},
				}, decad.FullRevolution{})
				require.NoError(t, err)
				return doc, body
			},
		},
		{
			name: "Shell",
			build: func(t *testing.T) (*decad.Document, *decad.Body) {
				doc, box := shellBox(t)
				body, err := box.Shell(topCap(box), units.Millimeters(5))
				require.NoError(t, err)
				return doc, body
			},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			doc, body := test.build(t)
			beforeBodies := doc.Bodies()
			beforeRecipe := doc.Recipe()
			ctx := newCancelAfterContext(t.Context(), 3)

			got, err := body.PlacedContext(ctx, shift)
			require.ErrorIs(t, err, context.Canceled)
			require.Nil(t, got)
			require.GreaterOrEqual(t, ctx.calls.Load(), int32(3),
				`cancellation must reach the analytic payload rebuild`)
			require.Equal(t, beforeBodies, doc.Bodies(),
				`a canceled analytic rebuild must not change live bodies`)
			require.Equal(t, beforeRecipe, doc.Recipe(),
				`a canceled analytic rebuild must not append a recipe step`)
		})
	}
}

func TestPlacementContextPollsAnalyticRebuildHelpers(t *testing.T) {
	shift, err := r3.Translation(r3.NewVec(100, 0, 0))
	require.NoError(t, err)

	tests := []struct {
		name   string
		target string
		build  func(*testing.T) (*decad.Document, *decad.Body)
	}{
		{
			name:   "PrismProfileIntegration",
			target: "integrateMomentRecordModeContext",
			build: func(t *testing.T) (*decad.Document, *decad.Body) {
				s, p := plateSketch(t)
				doc := decad.New()
				body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
				require.NoError(t, err)
				return doc, body
			},
		},
		{
			name:   "RevolveProfileIntegration",
			target: "integrateMomentRecordModeContext",
			build: func(t *testing.T) (*decad.Document, *decad.Body) {
				s, p := solidSketch(t)
				doc := decad.New()
				body, err := doc.Revolve(s, p, decad.SketchLine{
					Start: decad.Point2{U: 0, V: 0},
					End:   decad.Point2{U: 1, V: 0},
				}, decad.FullRevolution{})
				require.NoError(t, err)
				return doc, body
			},
		},
		{
			name:   "CupProfileIntegration",
			target: "integrateMomentRecordModeContext",
			build: func(t *testing.T) (*decad.Document, *decad.Body) {
				doc, box := shellBox(t)
				body, err := box.Shell(topCap(box), units.Millimeters(5))
				require.NoError(t, err)
				return doc, body
			},
		},
		{
			name:   "PrismWalkCoalescing",
			target: "coalesceWalksContext",
			build: func(t *testing.T) (*decad.Document, *decad.Body) {
				s, p := plateSketch(t)
				doc := decad.New()
				body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
				require.NoError(t, err)
				return doc, body
			},
		},
		{
			name:   "RevolveWalkCoalescing",
			target: "coalesceWalksContext",
			build: func(t *testing.T) (*decad.Document, *decad.Body) {
				s, p := solidSketch(t)
				doc := decad.New()
				body, err := doc.Revolve(s, p, decad.SketchLine{
					Start: decad.Point2{U: 0, V: 0},
					End:   decad.Point2{U: 1, V: 0},
				}, decad.FullRevolution{})
				require.NoError(t, err)
				return doc, body
			},
		},
		{
			name:   "CupWalkCoalescing",
			target: "coalesceWalksContext",
			build: func(t *testing.T) (*decad.Document, *decad.Body) {
				doc, box := shellBox(t)
				body, err := box.Shell(topCap(box), units.Millimeters(5))
				require.NoError(t, err)
				return doc, body
			},
		},
		{
			name:   "CupLoopReversal",
			target: "reverseLoopRecordContext",
			build: func(t *testing.T) (*decad.Document, *decad.Body) {
				doc, box := shellBox(t)
				body, err := box.Shell(topCap(box), units.Millimeters(5))
				require.NoError(t, err)
				return doc, body
			},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			doc, body := test.build(t)
			beforeBodies := doc.Bodies()
			beforeRecipe := doc.Recipe()
			ctx := &cancelWhenInFrameContext{
				Context: t.Context(),
				target:  test.target,
				limit:   2,
			}

			got, err := body.PlacedContext(ctx, shift)
			require.ErrorIs(t, err, context.Canceled)
			require.Nil(t, got)
			require.True(t, ctx.entered, `cancellation must reach %s`, test.target)
			require.GreaterOrEqual(t, ctx.targetCalls, ctx.limit,
				`%s must poll while processing recorded segments`, test.target)
			require.Equal(t, beforeBodies, doc.Bodies())
			require.Equal(t, beforeRecipe, doc.Recipe())
		})
	}
}

func TestPlacementContextVariantsMatchCompatibilityWrappers(t *testing.T) {
	shift, err := r3.Translation(r3.NewVec(100, 0, 0))
	require.NoError(t, err)

	tests := []struct {
		name       string
		wrapper    func(*decad.Body) (*decad.Body, error)
		contextual func(*decad.Body) (*decad.Body, error)
	}{
		{
			name:    "Placed",
			wrapper: func(body *decad.Body) (*decad.Body, error) { return body.Placed(shift) },
			contextual: func(body *decad.Body) (*decad.Body, error) {
				return body.PlacedContext(t.Context(), shift)
			},
		},
		{
			name:    "Duplicate",
			wrapper: func(body *decad.Body) (*decad.Body, error) { return body.Duplicate() },
			contextual: func(body *decad.Body) (*decad.Body, error) {
				return body.DuplicateContext(t.Context())
			},
		},
		{
			name:    "PlacedCopy",
			wrapper: func(body *decad.Body) (*decad.Body, error) { return body.PlacedCopy(shift) },
			contextual: func(body *decad.Body) (*decad.Body, error) {
				return body.PlacedCopyContext(t.Context(), shift)
			},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			s, p := plateSketch(t)
			oldDoc := decad.New()
			oldSource, err := oldDoc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			require.NoError(t, err)
			newDoc := decad.New()
			newSource, err := newDoc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			require.NoError(t, err)

			oldBody, err := test.wrapper(oldSource)
			require.NoError(t, err)
			newBody, err := test.contextual(newSource)
			require.NoError(t, err)

			oldVolume, err := oldBody.Volume()
			require.NoError(t, err)
			newVolume, err := newBody.Volume()
			require.NoError(t, err)
			require.Equal(t, oldVolume, newVolume)
			oldCentroid, err := oldBody.Centroid()
			require.NoError(t, err)
			newCentroid, err := newBody.Centroid()
			require.NoError(t, err)
			require.Equal(t, oldCentroid, newCentroid)
			require.Equal(t, oldDoc.Recipe(), newDoc.Recipe())
			require.Len(t, newDoc.Bodies(), len(oldDoc.Bodies()))
		})
	}
}
