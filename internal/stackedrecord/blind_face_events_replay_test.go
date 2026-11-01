package stackedrecord

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

const replayTestCaseExact = "exact"

func TestReplayBlindFaceLogLimitsAndMutation(t *testing.T) {
	body := &topo.Body{Payload: Payload{Slabs: make([]Slab, 2)}}
	a, b := &topo.Face{Body: body, Surface: surfacegeom.Plane{}, Origins: []provenance.Ref{{Role: "a"}}}, &topo.Face{Body: body, Surface: surfacegeom.Plane{}, Origins: []provenance.Ref{{Role: "b"}}}
	shell := &topo.FaceShell{Faces: []*topo.Face{a, b}}
	body.Lumps = []*topo.Lump{{Shells: []*topo.FaceShell{shell}}}
	events := [2]ReplayBlindFaceEvent{{Face: a, Kind: ReplayBlindWall, Column: 0, Segment: 0}, {Face: b, Kind: ReplayBlindCap, Slab: 0, Region: 0}}
	run := func(ctx context.Context, meter *proofbound.ReplayBudget) (*ReplayBlindFaceLog, bool, error) {
		l, e := NewReplayBlindFaceLog(ctx, meter, len(events))
		if e != nil {
			return nil, false, e
		}
		for _, event := range events {
			if e = l.record(ctx, event); e != nil {
				return nil, false, e
			}
		}
		ok, e := l.Seal(ctx, body)
		if e != nil || !ok {
			return nil, ok, e
		}
		ok, e = l.Recheck(ctx, body, meter)
		return l, ok, e
	}
	meter := proofbound.NewReplayTrialBudget()
	l, ok, err := run(t.Context(), meter)
	if err != nil || !ok || l.Len() != 2 {
		t.Fatalf("sealed=%t error=%v", ok, err)
	}
	w, r := meter.Usage()
	z := meter.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, z uint64
		pass    bool
	}{{replayTestCaseExact, w, r, z, true}, {"work", w - 1, r, z, false}, {"records", w, r - 1, z, false}, {"bytes", w, r, z - 1, false}} {
		t.Run(tc.name, func(t *testing.T) {
			m, e := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.z)
			if e != nil {
				t.Fatal(e)
			}
			_, yes, e := run(t.Context(), m)
			if tc.pass && (e != nil || !yes) || !tc.pass && (e == nil || yes) {
				t.Fatalf("sealed=%t error=%v", yes, e)
			}
		})
	}
	if _, ok := l.Event(-1); ok {
		t.Fatal("invalid event index accepted")
	}
	if l.Matches(body, proofbound.NewReplayTrialBudget()) {
		t.Fatal("different budget accepted")
	}
	shell.Faces[0] = b
	if ok, e := l.Recheck(t.Context(), body, meter); e != nil || ok {
		t.Fatalf("duplicate face accepted=%t error=%v", ok, e)
	}
	shell.Faces[0] = a
	if err := l.record(t.Context(), events[0]); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("sealed append error=%v", err)
	}
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	if _, _, err := run(canceled, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("canceled=%v", err)
	}
}
