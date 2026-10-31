package verifyrun

import (
	"context"
	"runtime"
	"sync/atomic"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/verifyoption"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type cancellationBodyDraft struct {
	entered  chan struct{}
	finished atomic.Int32
}

func (m *cancellationBodyDraft) VerifyNone(ctx context.Context, _ *topo.Body, _ float64) (*topo.Mesh, error) {
	defer m.finished.Add(1)
	m.entered <- struct{}{}
	<-ctx.Done()
	return nil, ctx.Err()
}

func TestParallelBodyChecksJoinWorkersAfterCancellation(t *testing.T) {
	previous := runtime.GOMAXPROCS(2)
	defer runtime.GOMAXPROCS(previous)
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	points := []sectionrecord.Point2{{U: 0, V: 0}, {U: 4, V: 0}, {U: 4, V: 4}, {U: 0, V: 4}}
	var segments []sectionrecord.CurveSegment
	for i, p := range points {
		segments = append(segments, sectionrecord.LineSeg{Start: p, End: points[(i+1)%len(points)], TEnd: 1})
	}
	payload := prismrecord.Payload{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segments}},
		Frame:   frame, Z1: 4, Xform: r3.Identity(),
	}
	env := topo.Env{Doc: &topo.Document{}, Ref: 1, Alloc: topotest.Alloc{}}
	var bodies []*topo.Body
	for range 2 {
		body, err := prismrecord.Build(t.Context(), env, payload, freeform.NewFreeformWork())
		require.NoError(t, err)
		require.Equal(t, 64.0, body.Volume.Value.Base())
		// This empty shell is a control fixture that routes the real prism
		// through the injectable winding mesher. Cancellation occurs there,
		// before this synthetic shell can publish any geometry or report.
		body.Lumps[0].Shells = append(body.Lumps[0].Shells, &topo.FaceShell{})
		bodies = append(bodies, body)
		env.Ref++
	}
	ctx, cancel := context.WithCancel(orderedwork.WithWorkers(t.Context(), 2))
	defer cancel()
	mesher := &cancellationBodyDraft{entered: make(chan struct{}, len(bodies))}
	type outcome struct {
		report *Report
		err    error
	}
	done := make(chan outcome, 1)
	go func() {
		report, err := Run(ctx, Hooks{Draft: mesher}, bodies, verifyConfig{
			Wall: &verifyoption.WallSpec{Tool: units.Millimeters(1)}, ToolMM: 1, Rel: 1e-3,
		})
		done <- outcome{report: report, err: err}
	}()
	watchdog := time.NewTimer(5 * time.Second)
	defer watchdog.Stop()
	for range bodies {
		select {
		case <-mesher.entered:
		case result := <-done:
			t.Fatalf("Verify returned before both workers entered: %v", result.err)
		case <-watchdog.C:
			cancel()
			t.Fatal("body workers did not enter the winding mesher")
		}
	}
	cancel()
	select {
	case result := <-done:
		require.ErrorIs(t, result.err, context.Canceled)
		require.Nil(t, result.report)
		require.Equal(t, int32(len(bodies)), mesher.finished.Load(), "Verify must join every started body worker")
	case <-watchdog.C:
		t.Fatal("Verify did not join its body workers after cancellation")
	}
}
