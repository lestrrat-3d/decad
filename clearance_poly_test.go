package decad_test

import (
	"context"
	"runtime"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// sturmBuildCancelContext reports cancellation only while the Sturm chain
// build is on the call stack. A test using it proves the poll it observed is
// INSIDE that build, rather than at one of the phase boundaries the clearance
// kernel already polls before and after it.
type sturmBuildCancelContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	entered         bool
}

func (c *sturmBuildCancelContext) Err() error {
	pcs := make([]uintptr, 32)
	frames := runtime.CallersFrames(pcs[:runtime.Callers(2, pcs)])
	for {
		frame, more := frames.Next()
		if strings.HasSuffix(frame.Function, ".sturmChainContext") {
			c.entered = true
			return context.Canceled
		}
		if !more {
			return nil
		}
	}
}

// TestVerifyClearanceCancellationInsideSturmChainBuild is the public half of
// the chain build's cancellation proof: the §7 torus pair drives the degree-8
// circle × circle bracket, whose chain build is the longest single stretch of
// arithmetic in a clearance run, and a context cancelled there surfaces
// context.Canceled from Verify with the document untouched.
func TestVerifyClearanceCancellationInsideSturmChainBuild(t *testing.T) {
	doc := decad.New()
	torusBody(t, doc, 10, 2)
	b := torusBody(t, doc, 10, 2)
	shift, err := r3.Translation(r3.NewVec(0, 30, 0))
	require.NoError(t, err)
	_, err = b.Placed(shift)
	require.NoError(t, err)
	before := snapshotDocument(t, doc)
	ctx := &sturmBuildCancelContext{Context: t.Context()}

	report, err := doc.Verify(ctx, decad.WithClearances())

	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, report)
	require.True(t, ctx.entered, "the torus pair must reach the Sturm chain build")
	requireDocumentUnchanged(t, doc, before)
}
