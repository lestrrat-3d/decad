package decad

import (
	"context"
	"errors"
	"runtime"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

//nolint:containedctx // The test context mutates the native result at the query callback.
type solidMutationContext struct {
	context.Context
	body       *Body
	changeKind bool
	cancel     context.CancelFunc
	fired      bool
}

func (c *solidMutationContext) Err() error {
	pc, _, _, ok := runtime.Caller(1)
	if ok && !c.fired && strings.HasSuffix(runtime.FuncForPC(pc).Name(), ".LocatePrism") {
		c.fired = true
		if c.changeKind {
			c.body.n.Kind = topo.BodySheet
		} else {
			c.body.n.Solid = false
		}
		if c.cancel != nil {
			c.cancel()
		}
	}
	return c.Context.Err()
}

func TestLocatePointRechecksSolidAfterExactQuery(t *testing.T) {
	makeBody := func(t *testing.T) *Body {
		t.Helper()
		doc := New()
		world := sketch.NewWorld()
		s, err := world.CreateSketch(world.XY())
		if err != nil {
			t.Fatal(err)
		}
		rect := s.CreateRectangle(0, 0, 10, 10)
		s.Fix(rect.A)
		if _, err = s.Solve(t.Context()); err != nil {
			t.Fatal(err)
		}
		body, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along})
		if err != nil {
			t.Fatal(err)
		}
		return body
	}
	t.Run("unchanged", func(t *testing.T) {
		body := makeBody(t)
		got, err := body.LocatePoint(t.Context(), r3.Vec{X: 5, Y: 5, Z: 20}, units.Millimeters(.05))
		if err != nil || got != PointOutside {
			t.Fatalf("result=%s error=%v", got, err)
		}
		if body.n.Tess.Load() != nil {
			t.Fatal("exact classification unexpectedly used mesh fallback")
		}
	})
	for _, fallback := range []bool{false, true} {
		for _, kind := range []bool{false, true} {
			name := "decided/solid"
			if fallback {
				name = "fallback/solid"
			}
			if kind {
				name += "-kind"
			}
			t.Run(name, func(t *testing.T) {
				body := makeBody(t)
				if fallback {
					p := body.n.Payload.(prismrecord.Payload)
					p.SectionDelta = 1
					body.n.Payload = p
				}
				probe := &solidMutationContext{Context: t.Context(), body: body, changeKind: kind}
				got, err := body.LocatePoint(probe, r3.Vec{X: 5, Y: 5, Z: 20}, units.Millimeters(.05))
				if !probe.fired || !errors.Is(err, ErrNotSolid) || got != PointUndecided {
					t.Fatalf("fired=%t result=%s error=%v", probe.fired, got, err)
				}
				if body.n.Tess.Load() != nil {
					t.Fatal("invalid body reached mesh fallback")
				}
			})
		}
	}
	t.Run("cancellation", func(t *testing.T) {
		body := makeBody(t)
		ctx, cancel := context.WithCancel(t.Context())
		defer cancel()
		probe := &solidMutationContext{Context: ctx, body: body, cancel: cancel}
		got, err := body.LocatePoint(probe, r3.Vec{X: 5, Y: 5, Z: 20}, units.Millimeters(.05))
		if !probe.fired || !errors.Is(err, context.Canceled) || got != PointUndecided {
			t.Fatalf("fired=%t result=%s error=%v", probe.fired, got, err)
		}
	})
}
