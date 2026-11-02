package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"runtime"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismboolean"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

const replayTestCaseChanged = "changed"
const replayTestCaseBytesShort = "bytes short"
const replayTestCaseRecordsShort = "records short"
const replayTestCaseWorkShort = "work short"
const replayTestCaseExact = "exact"

func TestBlindGroupFaceLedger(t *testing.T) {
	if !t.Run("source_seal_cancel", testBlindSourceSealCancellation) {
		return
	}
	if !t.Run("public_cut_budget", testBlindGroupPublicCutBudget) {
		return
	}
	if !t.Run("public_route", testBlindGroupPublicCut) {
		return
	}
	if !t.Run("public_container", testBlindGroupPublicContainer) {
		return
	}
	t.Run("rectangular_host", func(t *testing.T) { testBlindGroupFaceLedger(t, false) })
	t.Run("slotted_host", func(t *testing.T) { testBlindGroupFaceLedger(t, true) })
}

type sealBatchCancelContext struct {
	context.Context //nolint:containedctx // The cancellation probe lasts for one synchronous Extrude.
	reached         bool
}

func (c *sealBatchCancelContext) Err() error {
	if c.reached {
		return context.Canceled
	}
	var pcs [24]uintptr
	n := runtime.Callers(2, pcs[:])
	frames := runtime.CallersFrames(pcs[:n])
	for {
		frame, more := frames.Next()
		if strings.Contains(frame.Function, "SealPrismBuildFacesReplay.func1") {
			c.reached = true
			return context.Canceled
		}
		if !more {
			return c.Context.Err()
		}
	}
}

func testBlindSourceSealCancellation(t *testing.T) {
	doc := New()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		t.Fatal(err)
	}
	rect := s.CreateRectangle(0, 0, 4, 3)
	s.Fix(rect.A)
	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatal(err)
	}
	producer, live, history := doc.nextProducerID(), len(doc.n.LiveSet), len(doc.operations)
	ctx := &sealBatchCancelContext{Context: t.Context()}
	body, err := Extrude(ctx, doc, s, s.Profiles()[0], Distance{D: units.Millimeters(2), Dir: Along})
	if !ctx.reached || body != nil || !errors.Is(err, context.Canceled) {
		t.Fatalf("checkpoint reached=%t body=%t error=%v", ctx.reached, body != nil, err)
	}
	if doc.nextProducerID() != producer || len(doc.n.LiveSet) != live || len(doc.operations) != history {
		t.Fatal("canceled construction changed document membership or numbering")
	}
}

func testBlindGroupPublicContainer(t *testing.T) {
	doc, host, group := blindGroupSources(t, true)
	unrelated := internalBoxBody(t, doc, 30, 30, 2, 2, 2)
	container, err := doc.Root().NewChild("joint")
	if err != nil {
		t.Fatal(err)
	}
	if err := doc.MoveBody(host, container); err != nil {
		t.Fatal(err)
	}
	if err := doc.MoveBody(group, container); err != nil {
		t.Fatal(err)
	}
	result, err := Cut(t.Context(), host, group)
	if err != nil {
		t.Fatal(err)
	}
	got, err := doc.ContainerOf(result)
	if err != nil || got != container {
		t.Fatalf("result container matches=%t error=%v", got == container, err)
	}
	got, err = doc.ContainerOf(unrelated)
	if err != nil || got != doc.Root() {
		t.Fatalf("unrelated container matches root=%t error=%v", got == doc.Root(), err)
	}
}

func testBlindGroupPublicCutBudget(t *testing.T) {
	run := func(ctx context.Context, budget *proofbound.ReplayBudget) error {
		doc, host, group := blindGroupSources(t, true)        //nolint:contextcheck // The fixture uses t.Context().
		unrelated := internalBoxBody(t, doc, 30, 30, 2, 2, 2) //nolint:contextcheck // The fixture uses t.Context().
		beforeLive, beforeHistory, beforeProducer := len(doc.n.LiveSet), len(doc.operations), doc.nextProducerID()
		hostCache, groupCache := host.n.Tess.Load(), group.n.Tess.Load()
		candidate, admitted, err := groupBlindCutBodyWithBudget(ctx, host, group, beforeProducer, budget)
		if err == nil && admitted && candidate != nil {
			err = commitBlindGroupCut(ctx, doc, host, group, candidate, budget)
		}
		if err != nil || !admitted || candidate == nil {
			if len(doc.n.LiveSet) != beforeLive || len(doc.operations) != beforeHistory || doc.nextProducerID() != beforeProducer ||
				doc.requireLive(host) != nil || doc.requireLive(group) != nil || doc.requireLive(unrelated) != nil ||
				host.n.Tess.Load() != hostCache || group.n.Tess.Load() != groupCache {
				t.Fatal("refused Cut changed document or input caches")
			}
			if err == nil {
				return decaderr.ErrUnsupported
			}
			return err
		}
		if len(doc.n.LiveSet) != beforeLive-1 || len(doc.operations) != beforeHistory+1 ||
			doc.nextProducerID() != beforeProducer+1 || doc.requireLive(candidate) != nil || doc.requireLive(unrelated) != nil ||
			doc.requireLive(host) == nil || doc.requireLive(group) == nil {
			t.Fatal("committed Cut did not update document atomically")
		}
		return nil
	}
	meter := proofbound.NewReplayTrialBudget()
	if err := run(t.Context(), meter); err != nil {
		t.Fatal(err)
	}
	work, records := meter.Usage()
	bytes := meter.Bytes()
	t.Logf("public Cut source and commit work=%d records=%d bytes=%d", work, records, bytes)
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		want                 bool
	}{
		{replayTestCaseExact, work, records, bytes, true},
		{replayTestCaseWorkShort, work - 1, records, bytes, false},
		{replayTestCaseRecordsShort, work, records - 1, bytes, false},
		{replayTestCaseBytesShort, work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			limit, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
			if err != nil {
				t.Fatal(err)
			}
			err = run(t.Context(), limit)
			if (err == nil) != tc.want {
				t.Fatalf("budget %s error=%v", tc.name, err)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if err := run(ctx, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancelled Cut error=%v", err)
	}
	doc, host, group := blindGroupSources(t, true)
	beforeLive, beforeHistory, beforeProducer := len(doc.n.LiveSet), len(doc.operations), doc.nextProducerID()
	budget := proofbound.NewReplayTrialBudget()
	candidate, admitted, err := groupBlindCutBodyWithBudget(t.Context(), host, group, beforeProducer, budget)
	if err != nil || !admitted || candidate == nil {
		t.Fatalf("commit cancellation fixture admitted=%t error=%v", admitted, err)
	}
	late := newCancelAfterContext(t.Context(), 8)
	err = commitBlindGroupCut(late, doc, host, group, candidate, budget)
	if !errors.Is(err, context.Canceled) || len(doc.n.LiveSet) != beforeLive ||
		len(doc.operations) != beforeHistory || doc.nextProducerID() != beforeProducer ||
		doc.requireLive(host) != nil || doc.requireLive(group) != nil || candidate.blindProof.cut != nil {
		t.Fatalf("commit cancellation error=%v polls=%d", err, late.calls)
	}
}

func blindGroupSources(t *testing.T, withSlot bool) (*Document, *Body, *Body) {
	t.Helper()
	doc := New()
	var host *Body
	if withSlot {
		world := sketch.NewWorld()
		s, err := world.CreateSketch(world.XY())
		if err != nil {
			t.Fatal(err)
		}
		rect := s.CreateRectangle(-10, -10, 20, 20)
		s.Fix(rect.A)
		slot, err := s.CreateSlot(-8, -6, -8, -4, 0.5)
		if err != nil {
			t.Fatal(err)
		}
		s.Fix(slot.C1)
		s.Fix(slot.C2)
		if _, err = s.Solve(t.Context()); err != nil {
			t.Fatal(err)
		}
		var profile *sketch.Profile
		for _, p := range s.Profiles() {
			if p.Valid && len(p.Holes) == 1 {
				profile = p
			}
		}
		if profile == nil {
			t.Fatal("missing host profile with slot")
		}
		host, err = Extrude(t.Context(), doc, s, profile, Distance{D: units.Millimeters(10), Dir: Along})
		if err != nil {
			t.Fatal(err)
		}
	} else {
		host = internalBoxBody(t, doc, -10, -10, 10, 10, 10)
	}
	var leaves []*Body
	for _, x := range []float64{-6, -2, 2, 6} {
		leaves = append(leaves, internalCircleBody(t, doc, x, 1, 6, Distance{D: units.Millimeters(4), Dir: Along}))
	}
	group, err := Union(t.Context(), leaves[0], leaves[1:]...)
	if err != nil {
		t.Fatal(err)
	}
	return doc, host, group
}

func testBlindGroupPublicCut(t *testing.T) {
	doc, host, group := blindGroupSources(t, true)
	unrelated := internalBoxBody(t, doc, 30, 30, 2, 2, 2)
	result, err := Cut(t.Context(), host, group)
	if err != nil || result == nil || result.blindProof == nil {
		t.Fatalf("public Cut body=%t proof=%t error=%v", result != nil,
			result != nil && result.blindProof != nil, err)
	}
	if err := doc.requireLive(unrelated); err != nil {
		t.Fatalf("public Cut changed unrelated body: %v", err)
	}
	op := doc.producers[result]
	if op == nil || op.kind != OperationCut || len(op.inputs) != 2 ||
		op.inputs[0].Body != host || op.inputs[0].Role != "target" || op.inputs[0].Use != InputConsumed ||
		op.inputs[1].Body != group || op.inputs[1].Role != "tool" || op.inputs[1].Use != InputConsumed ||
		len(op.outputs) != 1 || op.outputs[0] != result {
		t.Fatal("public Cut did not preserve ordered input history")
	}
	if doc.requireLive(host) == nil || doc.requireLive(group) == nil {
		t.Fatal("public Cut left an input live")
	}
	mesh, err := result.Tessellate(t.Context(), units.Millimeters(.12), WithVerification(VerifyAll))
	if err != nil || mesh == nil || !mesh.BoundaryVerified() || !mesh.VolumeVerified() {
		meter := proofbound.NewReplayTrialBudget()
		topology, ok, topologyErr := stackedrecord.ReplayBlindMeshTopologyWithSource(t.Context(), &result.n, &host.n, .12, meter)
		var contactErr error
		if ok && topologyErr == nil && topology.Mesh != nil {
			contactErr = tessellation.ReplayRequireEmbeddedBoundary(t.Context(), topology.Mesh.Vertices, topology.Mesh.Triangles, meter)
		}
		work, records := meter.Usage()
		t.Fatalf("public VerifyAll mesh=%t error=%v topology=%t topologyError=%v contactError=%v work=%d records=%d bytes=%d",
			mesh != nil, err, ok, topologyErr, contactErr, work, records, meter.Bytes())
	}
	cached, err := result.Tessellate(t.Context(), units.Millimeters(.12), WithVerification(VerifyAll))
	if err != nil || cached != mesh {
		t.Fatalf("public cache hit matches=%t error=%v", cached == mesh, err)
	}
	faces := make(map[*topo.Face]struct{})
	for _, lump := range result.n.Lumps {
		for _, shell := range lump.Shells {
			for _, face := range shell.Faces {
				if face == nil || face.Body != &result.n || len(face.Upstream) == 0 {
					t.Fatal("public Cut has an unbound output face")
				}
				faces[face] = struct{}{}
			}
		}
	}
	if len(mesh.n.Source) != len(mesh.n.Triangles) {
		t.Fatal("public mesh has incomplete facet provenance")
	}
	for _, face := range mesh.n.Source {
		if _, ok := faces[face]; !ok {
			t.Fatal("public mesh names a foreign source face")
		}
	}
	entry := result.n.Tess.Load()
	if entry == nil {
		t.Fatal("public VerifyAll did not publish its cache")
	}
	oldOrigin := host.n.Origin
	host.n.Origin.Producer++
	_, err = result.Tessellate(t.Context(), units.Millimeters(.12), WithVerification(VerifyAll))
	host.n.Origin = oldOrigin
	if !errors.Is(err, decaderr.ErrUnsupported) || result.n.Tess.Load() != entry {
		t.Fatalf("changed source error=%v cacheChanged=%t", err, result.n.Tess.Load() != entry)
	}
	oldRole := op.inputs[0].Role
	op.inputs[0].Role = replayTestCaseChanged
	_, err = result.Tessellate(t.Context(), units.Millimeters(.12), WithVerification(VerifyAll))
	op.inputs[0].Role = oldRole
	if !errors.Is(err, decaderr.ErrUnsupported) || result.n.Tess.Load() != entry {
		t.Fatalf("changed history error=%v cacheChanged=%t", err, result.n.Tess.Load() != entry)
	}
	oldVertex := mesh.n.Vertices[0]
	mesh.n.Vertices[0].X++
	_, err = result.Tessellate(t.Context(), units.Millimeters(.12), WithVerification(VerifyAll))
	mesh.n.Vertices[0] = oldVertex
	if !errors.Is(err, decaderr.ErrUnsupported) || result.n.Tess.Load() != entry {
		t.Fatalf("changed mesh error=%v cacheChanged=%t", err, result.n.Tess.Load() != entry)
	}
	oldCachedNode := entry.Mesh
	entry.Mesh = &topo.Mesh{}
	_, err = result.Tessellate(t.Context(), units.Millimeters(.12), WithVerification(VerifyAll))
	entry.Mesh = oldCachedNode
	if !errors.Is(err, decaderr.ErrUnsupported) || result.n.Tess.Load() != entry {
		t.Fatalf("changed cache entry error=%v cacheChanged=%t", err, result.n.Tess.Load() != entry)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = result.Tessellate(ctx, units.Millimeters(.12), WithVerification(VerifyAll))
	if !errors.Is(err, context.Canceled) || result.n.Tess.Load() != entry {
		t.Fatalf("cancelled tessellation error=%v cacheChanged=%t", err, result.n.Tess.Load() != entry)
	}
	meter := proofbound.NewReplayTrialBudget()
	checked, err := tessellateContextWithBudget(t.Context(), result, units.Millimeters(.12), VerifyAll, meter)
	if err != nil || checked != mesh {
		t.Fatalf("metered cache hit matches=%t error=%v", checked == mesh, err)
	}
	work, records := meter.Usage()
	bytes := meter.Bytes()
	t.Logf("public VerifyAll cache hit work=%d records=%d bytes=%d vertices=%d facets=%d faces=%d",
		work, records, bytes, len(mesh.n.Vertices), len(mesh.n.Triangles), len(faces))
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		want                 bool
	}{
		{replayTestCaseExact, work, records, bytes, true},
		{replayTestCaseWorkShort, work - 1, records, bytes, false},
		{replayTestCaseRecordsShort, work, records - 1, bytes, false},
		{replayTestCaseBytesShort, work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			limit, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
			if err != nil {
				t.Fatal(err)
			}
			got, err := tessellateContextWithBudget(t.Context(), result, units.Millimeters(.12), VerifyAll, limit)
			if tc.want {
				if err != nil || got != mesh {
					t.Fatalf("exact cache hit matches=%t error=%v", got == mesh, err)
				}
			} else if got != nil || !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("short cache hit returned mesh=%t error=%v", got != nil, err)
			}
			if result.n.Tess.Load() != entry {
				t.Fatal("budgeted cache check changed the entry")
			}
		})
	}
	originalBinding := result.blindProof.cache.Load()
	clearCache := func() {
		result.n.Tess.Store(nil)
		result.blindProof.cache.Store(nil)
	}
	clearCache()
	missMeter := proofbound.NewReplayTrialBudget()
	missMesh, err := tessellateContextWithBudget(t.Context(), result, units.Millimeters(.12), VerifyAll, missMeter)
	if err != nil || missMesh == nil || !missMesh.BoundaryVerified() || !missMesh.VolumeVerified() {
		t.Fatalf("metered cache miss mesh=%t error=%v", missMesh != nil, err)
	}
	missWork, missRecords := missMeter.Usage()
	missBytes := missMeter.Bytes()
	t.Logf("public VerifyAll cache miss work=%d records=%d bytes=%d", missWork, missRecords, missBytes)
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		want                 bool
	}{
		{replayTestCaseExact, missWork, missRecords, missBytes, true},
		{replayTestCaseWorkShort, missWork - 1, missRecords, missBytes, false},
		{replayTestCaseRecordsShort, missWork, missRecords - 1, missBytes, false},
		{replayTestCaseBytesShort, missWork, missRecords, missBytes - 1, false},
	} {
		t.Run("miss "+tc.name, func(t *testing.T) {
			clearCache()
			limit, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
			if err != nil {
				t.Fatal(err)
			}
			got, err := tessellateContextWithBudget(t.Context(), result, units.Millimeters(.12), VerifyAll, limit)
			if tc.want {
				if err != nil || got == nil || !got.BoundaryVerified() || !got.VolumeVerified() || result.n.Tess.Load() == nil {
					t.Fatalf("exact cache miss mesh=%t error=%v", got != nil, err)
				}
			} else if got != nil || !errors.Is(err, decaderr.ErrUnsupported) || result.n.Tess.Load() != nil || result.blindProof.cache.Load() != nil {
				t.Fatalf("short cache miss mesh=%t error=%v", got != nil, err)
			}
		})
	}
	result.blindProof.cache.Store(originalBinding)
	result.n.Tess.Store(entry)
}

func testBlindGroupFaceLedger(t *testing.T, withSlot bool) {
	doc, host, group := blindGroupSources(t, withSlot)
	var err error
	var routed *topo.Body
	var publicCandidate *Body
	var ordinaryMesh *topo.Mesh
	if withSlot {
		cutMeter := proofbound.NewReplayTrialBudget()
		cutBody, admitted, e := groupBlindCutBodyWithBudget(t.Context(), host, group,
			doc.nextProducerID(), cutMeter)
		if e != nil || !admitted || cutBody == nil {
			t.Fatalf("source-bound four-circle Cut admitted=%t body=%t error=%v",
				admitted, cutBody != nil, e)
		}
		routed = bodyNode(cutBody)
		publicCandidate = cutBody
		ordinaryPayload := routed.Payload.(stackedrecord.Payload)
		ordinaryMesh, e = stackedrecord.Tessellate(t.Context(), routed, ordinaryPayload, .12, true)
		if e != nil || ordinaryMesh == nil {
			t.Fatalf("ordinary .12 mesh produced=%t error=%v", ordinaryMesh != nil, e)
		}
		t.Logf("ordinary .12 mesh vertices=%d triangles=%d volume-bound=%g",
			len(ordinaryMesh.Vertices), len(ordinaryMesh.Triangles), ordinaryMesh.VolSymDiff)
		work, records := cutMeter.Usage()
		bytes := cutMeter.Bytes()
		t.Logf("source-bound four-circle Cut work=%d records=%d bytes=%d",
			work, records, bytes)
		for _, tc := range []struct {
			name                 string
			work, records, bytes uint64
			want                 bool
		}{
			{replayTestCaseExact, work, records, bytes, true},
			{replayTestCaseWorkShort, work - 1, records, bytes, false},
			{replayTestCaseRecordsShort, work, records - 1, bytes, false},
			{replayTestCaseBytesShort, work, records, bytes - 1, false},
		} {
			t.Run("source-bound Cut/"+tc.name, func(t *testing.T) {
				limited, e := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
				if e != nil {
					t.Fatal(e)
				}
				got, yes, e := groupBlindCutBodyWithBudget(t.Context(), host, group,
					doc.nextProducerID(), limited)
				if tc.want {
					if e != nil || !yes || got == nil {
						t.Fatalf("exact Cut admitted=%t body=%t error=%v", yes, got != nil, e)
					}
				} else {
					var limit *proofbound.WorkLimitError
					if yes || got != nil || !errors.As(e, &limit) {
						t.Fatalf("short Cut admitted=%t body=%t error=%v", yes, got != nil, e)
					}
				}
				if e := doc.requireLive(host); e != nil {
					t.Fatal(e)
				}
				if e := doc.requireLive(group); e != nil {
					t.Fatal(e)
				}
			})
		}
		cancelled, cancel := context.WithCancel(t.Context())
		cancel()
		got, yes, e := groupBlindCutBodyWithBudget(cancelled, host, group,
			doc.nextProducerID(), proofbound.NewReplayTrialBudget())
		if !errors.Is(e, context.Canceled) || yes || got != nil {
			t.Fatalf("cancelled Cut admitted=%t body=%t error=%v", yes, got != nil, e)
		}
		sourceFace := bodyNode(host).Lumps[0].Shells[0].Faces[0]
		oldUpstream := sourceFace.Upstream
		sourceFace.Upstream = []provenance.Ref{{Producer: 1, Role: "unproved"}}
		got, yes, e = groupBlindCutBodyWithBudget(t.Context(), host, group,
			doc.nextProducerID(), proofbound.NewReplayTrialBudget())
		sourceFace.Upstream = oldUpstream
		if e != nil || yes || got != nil {
			t.Fatalf("mutated source ancestry admitted=%t body=%t error=%v", yes, got != nil, e)
		}
		originalPayload := host.n.Payload.(prismrecord.Payload)
		changedPayload := originalPayload
		changedPayload.Z1Denoted = prismplacement.DenoteLevel(
			changedPayload.DenotedZ1(), changedPayload.Z1, changedPayload.Z1Delta)
		host.n.Payload = changedPayload
		got, yes, e = groupBlindCutBodyWithBudget(t.Context(), host, group,
			doc.nextProducerID(), proofbound.NewReplayTrialBudget())
		host.n.Payload = originalPayload
		if e != nil || yes || got != nil {
			t.Fatalf("replaced source denotation admitted=%t body=%t error=%v", yes, got != nil, e)
		}
		history := doc.producers[host]
		originalKind := history.kind
		history.kind = OperationCut
		got, yes, e = groupBlindCutBodyWithBudget(t.Context(), host, group,
			doc.nextProducerID(), proofbound.NewReplayTrialBudget())
		history.kind = originalKind
		if e != nil || yes || got != nil {
			t.Fatalf("changed producer history admitted=%t body=%t error=%v", yes, got != nil, e)
		}
	}
	budget := proofbound.NewReplayTrialBudget()
	source, ok, err := groupUnionSourceSnapshot(t.Context(), group, budget)
	if err != nil || !ok {
		t.Fatalf("source ok=%t error=%v", ok, err)
	}
	hostPayload := host.n.Payload.(prismrecord.Payload)
	groupPayload := group.n.Payload.(stackedrecord.Payload)
	slab := groupPayload.Slabs[0]
	place := prismplacement.Operand{Frame: groupPayload.Frame, Xform: groupPayload.Xform, Z0: slab.Z0, Z1: slab.Z1, Z0Delta: slab.Z0Delta, Z1Delta: slab.Z1Delta, Z0Denoted: slab.Z0Denoted, Z1Denoted: slab.Z1Denoted}
	scene, ok, err := prismcells.ReplayBlindGroupScene(t.Context(), hostPayload.Profile, slab.Regions, hostPayload.PlacementOperand(), place, hostPayload.SectionDelta, groupPayload.SectionDelta, budget)
	if err != nil || !ok {
		t.Fatalf("scene ok=%t error=%v", ok, err)
	}
	output, _, ok, err := stackedrecord.ReplayBlindGroupPayload(t.Context(), hostPayload, groupPayload, scene, budget)
	if err != nil || !ok {
		t.Fatalf("payload ok=%t error=%v", ok, err)
	}
	result, events, ok, err := stackedrecord.ReplayBlindGroupBuildWithSource(t.Context(), envOf(documentNode(doc), doc.nextProducerID()), output, bodyNode(host), groupPayload, scene, budget)
	if err != nil || !ok {
		t.Fatalf("build ok=%t error=%v", ok, err)
	}
	nodes := make([]*topo.Body, len(source.Inputs))
	for i, v := range source.Inputs {
		nodes[i] = bodyNode(v)
	}
	input := prismboolean.BlindFaceLedgerInput{Output: result, Target: bodyNode(host), Group: bodyNode(group), Leaves: nodes, Owners: source.Owners, GroupCaps: source.GroupCaps, LeafCaps: source.LeafCaps, Scene: scene, Events: events}

	beforeW, beforeR := budget.Usage()
	beforeB := budget.Bytes()
	ledger, ok, err := prismboolean.ReplayBlindFaceLedger(t.Context(), input, budget)
	if err != nil || !ok {
		t.Fatalf("ledger ok=%t error=%v", ok, err)
	}
	if withSlot {
		entry, found := ledger.Entry(0)
		if !found {
			t.Fatal("missing first output face")
		}
		oldUpstream := entry.Output.Upstream
		entry.Output.Upstream = []provenance.Ref{{Producer: 1, Role: "unproved"}}
		published, e := prismboolean.ReplayPublishBlindUpstream(t.Context(), input, ledger, budget)
		entry.Output.Upstream = oldUpstream
		if e != nil || published {
			t.Fatalf("mutated output ancestry published=%t error=%v", published, e)
		}
		if len(result.Payload.(stackedrecord.Payload).Upstream) != 0 {
			t.Fatal("refused publication changed output payload")
		}
		for i := range ledger.Len() {
			row, _ := ledger.Entry(i)
			if len(row.Output.Upstream) != 0 {
				t.Fatalf("refused publication changed output face %d", i)
			}
		}
		published, e = prismboolean.ReplayPublishBlindUpstream(t.Context(), input, ledger, budget)
		if e != nil || !published || len(result.Payload.(stackedrecord.Payload).Upstream) != ledger.Len() {
			t.Fatalf("clean publication=%t rows=%d error=%v",
				published, len(result.Payload.(stackedrecord.Payload).Upstream), e)
		}
		for i := range ledger.Len() {
			row, _ := ledger.Entry(i)
			if !row.Output.HasOrigin(row.Immediate.Origins[0]) || !row.Output.HasOrigin(row.Leaf.Origins[0]) {
				t.Fatalf("published face %d lacks bound source origin", i)
			}
		}
		consumer := proofbound.NewReplayTrialBudget()
		walls, chordOK, chordErr := stackedrecord.ReplayBlindMeshWallsWithSource(t.Context(),
			routed, bodyNode(host), .12, consumer)
		if chordErr != nil || !chordOK {
			t.Fatalf("source-bound slotted wall stage ok=%t error=%v", chordOK, chordErr)
		}
		vertices := walls.Vertices
		if len(vertices.Vertices) != len(ordinaryMesh.Vertices) || len(vertices.Store) != len(vertices.Vertices) {
			t.Fatalf("vertex counts bounded=%d ordinary=%d stores=%d",
				len(vertices.Vertices), len(ordinaryMesh.Vertices), len(vertices.Store))
		}
		for i, got := range vertices.Vertices {
			want := ordinaryMesh.Vertices[i]
			if math.Float64bits(got.X) != math.Float64bits(want.X) ||
				math.Float64bits(got.Y) != math.Float64bits(want.Y) ||
				math.Float64bits(got.Z) != math.Float64bits(want.Z) ||
				proofbound.IsNonFinite(vertices.Store[i]) {
				t.Fatalf("vertex %d differs or has invalid store bound", i)
			}
		}
		if len(walls.Triangles) > len(ordinaryMesh.Triangles) || len(walls.Source) != len(walls.Triangles) ||
			len(walls.Faces) != len(walls.Triangles) {
			t.Fatalf("wall triangle count=%d ordinary=%d owners=%d faces=%d",
				len(walls.Triangles), len(ordinaryMesh.Triangles), len(walls.Source), len(walls.Faces))
		}
		for i, tri := range walls.Triangles {
			if tri != ordinaryMesh.Triangles[i] || walls.Faces[i] != ordinaryMesh.Source[i] {
				t.Fatalf("wall triangle or source face %d differs", i)
			}
		}
		assemblyMeter := proofbound.NewReplayTrialBudget()
		assembly, assembled, assemblyErr := stackedrecord.ReplayBlindMeshAssemblyWithSource(t.Context(),
			routed, bodyNode(host), .12, assemblyMeter)
		if assemblyErr != nil || !assembled || assembly.Mesh == nil {
			t.Fatalf("real mesh assembly admitted=%t mesh=%t error=%v",
				assembled, assembly.Mesh != nil, assemblyErr)
		}
		if len(assembly.Mesh.Triangles) != len(ordinaryMesh.Triangles) ||
			len(assembly.Mesh.Source) != len(ordinaryMesh.Source) {
			t.Fatalf("assembled triangles=%d ordinary=%d sources=%d",
				len(assembly.Mesh.Triangles), len(ordinaryMesh.Triangles), len(assembly.Mesh.Source))
		}
		for i, tri := range assembly.Mesh.Triangles {
			if tri != ordinaryMesh.Triangles[i] || assembly.Mesh.Source[i] != ordinaryMesh.Source[i] {
				t.Fatalf("assembled triangle or source face %d differs", i)
			}
		}
		assemblyWork, assemblyRecords := assemblyMeter.Usage()
		assemblyBytes := assemblyMeter.Bytes()
		t.Logf("real mesh assembly triangles=%d work=%d records=%d bytes=%d",
			len(assembly.Mesh.Triangles), assemblyWork, assemblyRecords, assemblyBytes)
		topologyMeter := proofbound.NewReplayTrialBudget()
		topology, proven, topologyErr := stackedrecord.ReplayBlindMeshTopologyWithSource(t.Context(),
			routed, bodyNode(host), .12, topologyMeter)
		if topologyErr != nil || !proven || topology.Mesh == nil {
			t.Fatalf("real mesh topology admitted=%t mesh=%t error=%v",
				proven, topology.Mesh != nil, topologyErr)
		}
		if len(topology.Mesh.FaceBound) != len(ordinaryMesh.FaceBound) ||
			math.Float64bits(topology.Mesh.Bound) != math.Float64bits(ordinaryMesh.Bound) {
			t.Fatalf("real mesh face bounds=%d ordinary=%d bound=%g ordinary=%g",
				len(topology.Mesh.FaceBound), len(ordinaryMesh.FaceBound), topology.Mesh.Bound, ordinaryMesh.Bound)
		}
		for face, want := range ordinaryMesh.FaceBound {
			got, found := topology.Mesh.FaceBound[face]
			if !found || math.Float64bits(got) != math.Float64bits(want) {
				t.Fatalf("real mesh bound differs for role=%s found=%t got=%.17g want=%.17g",
					face.Origins[0].Role, found, got, want)
			}
		}
		topologyWork, topologyRecords := topologyMeter.Usage()
		t.Logf("real mesh topology faces=%d work=%d records=%d bytes=%d",
			len(topology.Mesh.FaceBound), topologyWork, topologyRecords, topologyMeter.Bytes())
		volumeMeter := proofbound.NewReplayTrialBudget()
		volume, volumeOK, volumeErr := stackedrecord.ReplayBlindMeshVolumeWithSource(t.Context(),
			routed, bodyNode(host), .12, volumeMeter)
		if volumeErr != nil || !volumeOK || volume.Mesh == nil {
			t.Fatalf("real volume admitted=%t mesh=%t error=%v", volumeOK, volume.Mesh != nil, volumeErr)
		}
		if !volume.Mesh.SymDiffOK ||
			volume.Mesh.VolSymDiff < ordinaryMesh.VolSymDiff || volume.Mesh.AreaSlack < ordinaryMesh.AreaSlack {
			t.Fatalf("real volume admitted=%t mesh=%t volume=%g ordinary=%g area=%g ordinary=%g error=%v",
				volumeOK, volume.Mesh != nil, volume.Mesh.VolSymDiff, ordinaryMesh.VolSymDiff,
				volume.Mesh.AreaSlack, ordinaryMesh.AreaSlack, volumeErr)
		}
		volumeWork, volumeRecords := volumeMeter.Usage()
		t.Logf("real mesh volume work=%d records=%d bytes=%d symdiff=%g ordinary=%g",
			volumeWork, volumeRecords, volumeMeter.Bytes(), volume.Mesh.VolSymDiff, ordinaryMesh.VolSymDiff)
		for _, tc := range []struct {
			name                 string
			work, records, bytes uint64
			want                 bool
		}{
			{replayTestCaseExact, volumeWork, volumeRecords, volumeMeter.Bytes(), true},
			{replayTestCaseWorkShort, volumeWork - 1, volumeRecords, volumeMeter.Bytes(), false},
			{replayTestCaseRecordsShort, volumeWork, volumeRecords - 1, volumeMeter.Bytes(), false},
			{replayTestCaseBytesShort, volumeWork, volumeRecords, volumeMeter.Bytes() - 1, false},
		} {
			t.Run("real mesh volume/"+tc.name, func(t *testing.T) {
				limit, e := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
				if e != nil {
					t.Fatal(e)
				}
				got, admitted, e := stackedrecord.ReplayBlindMeshVolumeWithSource(t.Context(),
					routed, bodyNode(host), .12, limit)
				if tc.want {
					if e != nil || !admitted || got.Mesh == nil || !got.Mesh.SymDiffOK {
						t.Fatalf("exact volume admitted=%t mesh=%t error=%v", admitted, got.Mesh != nil, e)
					}
				} else if e == nil || admitted || got.Mesh != nil {
					t.Fatalf("short volume admitted=%t mesh=%t error=%v", admitted, got.Mesh != nil, e)
				}
			})
		}
		cancelledVolume, cancelVolume := context.WithCancel(t.Context())
		cancelVolume()
		if got, admitted, e := stackedrecord.ReplayBlindMeshVolumeWithSource(cancelledVolume,
			routed, bodyNode(host), .12, proofbound.NewReplayTrialBudget()); !errors.Is(e, context.Canceled) || admitted || got.Mesh != nil {
			t.Fatalf("cancelled volume admitted=%t mesh=%t error=%v", admitted, got.Mesh != nil, e)
		}
		for _, tc := range []struct {
			name                 string
			work, records, bytes uint64
			want                 bool
		}{
			{replayTestCaseExact, topologyWork, topologyRecords, topologyMeter.Bytes(), true},
			{replayTestCaseWorkShort, topologyWork - 1, topologyRecords, topologyMeter.Bytes(), false},
			{replayTestCaseRecordsShort, topologyWork, topologyRecords - 1, topologyMeter.Bytes(), false},
			{replayTestCaseBytesShort, topologyWork, topologyRecords, topologyMeter.Bytes() - 1, false},
		} {
			t.Run("real mesh topology/"+tc.name, func(t *testing.T) {
				limit, e := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
				if e != nil {
					t.Fatal(e)
				}
				got, admitted, e := stackedrecord.ReplayBlindMeshTopologyWithSource(t.Context(),
					routed, bodyNode(host), .12, limit)
				if tc.want {
					if e != nil || !admitted || got.Mesh == nil || len(got.Mesh.FaceBound) != len(ordinaryMesh.FaceBound) {
						t.Fatalf("exact topology admitted=%t mesh=%t error=%v", admitted, got.Mesh != nil, e)
					}
				} else if e == nil || admitted || got.Mesh != nil {
					t.Fatalf("short topology admitted=%t mesh=%t error=%v", admitted, got.Mesh != nil, e)
				}
			})
		}
		cancelledTopology, cancelTopology := context.WithCancel(t.Context())
		cancelTopology()
		if got, admitted, e := stackedrecord.ReplayBlindMeshTopologyWithSource(cancelledTopology,
			routed, bodyNode(host), .12, proofbound.NewReplayTrialBudget()); !errors.Is(e, context.Canceled) || admitted || got.Mesh != nil {
			t.Fatalf("cancelled topology admitted=%t mesh=%t error=%v", admitted, got.Mesh != nil, e)
		}
		for _, tc := range []struct {
			name                 string
			work, records, bytes uint64
			want                 bool
		}{
			{replayTestCaseExact, assemblyWork, assemblyRecords, assemblyBytes, true},
			{replayTestCaseWorkShort, assemblyWork - 1, assemblyRecords, assemblyBytes, false},
			{replayTestCaseRecordsShort, assemblyWork, assemblyRecords - 1, assemblyBytes, false},
			{replayTestCaseBytesShort, assemblyWork, assemblyRecords, assemblyBytes - 1, false},
		} {
			t.Run("real mesh assembly/"+tc.name, func(t *testing.T) {
				limit, e := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
				if e != nil {
					t.Fatal(e)
				}
				got, admitted, e := stackedrecord.ReplayBlindMeshAssemblyWithSource(t.Context(),
					routed, bodyNode(host), .12, limit)
				if tc.want {
					if e != nil || !admitted || got.Mesh == nil || len(got.Mesh.Triangles) != len(ordinaryMesh.Triangles) {
						t.Fatalf("exact assembly admitted=%t mesh=%t error=%v", admitted, got.Mesh != nil, e)
					}
				} else if e == nil || admitted || got.Mesh != nil {
					t.Fatalf("short assembly admitted=%t mesh=%t error=%v", admitted, got.Mesh != nil, e)
				}
			})
		}
		cancelledAssembly, cancelAssembly := context.WithCancel(t.Context())
		cancelAssembly()
		if got, admitted, e := stackedrecord.ReplayBlindMeshAssemblyWithSource(cancelledAssembly,
			routed, bodyNode(host), .12, proofbound.NewReplayTrialBudget()); !errors.Is(e, context.Canceled) || admitted || got.Mesh != nil {
			t.Fatalf("cancelled assembly admitted=%t mesh=%t error=%v", admitted, got.Mesh != nil, e)
		}
		rings := make([]tessellation.StackedRing[*topo.Face], len(vertices.Chords.Loops))
		for i, loop := range vertices.Chords.Loops {
			rings[i] = tessellation.StackedRing[*topo.Face]{Samples: loop.Samples,
				Bottom: vertices.Bottom[i], Top: vertices.Top[i], Sag: loop.MaxSag}
		}
		topRow := vertices.Chords.Rows[len(vertices.Chords.Rows)-1]
		topLoops := make([]tessellation.StackedPatchLoop, len(topRow))
		for i, row := range topRow {
			topLoops[i] = tessellation.StackedPatchLoop{Column: row.Column, Top: true,
				Outer: row.Loop == 0, ColumnOuter: vertices.Chords.Columns[row.Column].LoopIndex == 0}
		}
		patchMeter := proofbound.NewReplayTrialBudget()
		patch, e := stackedrecord.ReplayBlindPatchTriangles(t.Context(), rings, topLoops, false, patchMeter)
		if e != nil {
			t.Fatalf("real top cap replay error=%v", e)
		}
		ordinaryPatch, e := tessellation.StackedPatchTriangles(t.Context(), rings, topLoops, false,
			tessellation.RequireLoopClearance, triangulation.Triangulate)
		if e != nil || len(patch.Triangles) != len(ordinaryPatch.Triangles) {
			t.Fatalf("top cap parity replay=%d ordinary=%d error=%v",
				len(patch.Triangles), len(ordinaryPatch.Triangles), e)
		}
		for i, tri := range patch.Triangles {
			if tri != ordinaryPatch.Triangles[i] {
				t.Fatalf("top cap triangle %d differs", i)
			}
		}
		patchWork, patchRecords := patchMeter.Usage()
		patchBytes := patchMeter.Bytes()
		t.Logf("real top cap triangles=%d work=%d records=%d bytes=%d",
			len(patch.Triangles), patchWork, patchRecords, patchBytes)
		for _, tc := range []struct {
			name                 string
			work, records, bytes uint64
			want                 bool
		}{
			{replayTestCaseExact, patchWork, patchRecords, patchBytes, true},
			{replayTestCaseWorkShort, patchWork - 1, patchRecords, patchBytes, false},
			{replayTestCaseRecordsShort, patchWork, patchRecords - 1, patchBytes, false},
			{replayTestCaseBytesShort, patchWork, patchRecords, patchBytes - 1, false},
		} {
			t.Run("real top cap/"+tc.name, func(t *testing.T) {
				limit, e := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
				if e != nil {
					t.Fatal(e)
				}
				got, e := stackedrecord.ReplayBlindPatchTriangles(t.Context(), rings, topLoops, false, limit)
				if tc.want {
					if e != nil || len(got.Triangles) != len(patch.Triangles) {
						t.Fatalf("exact top cap triangles=%d error=%v", len(got.Triangles), e)
					}
				} else if e == nil || len(got.Triangles) != 0 {
					t.Fatalf("short top cap triangles=%d error=%v", len(got.Triangles), e)
				}
			})
		}
		cancelledPatch, cancelPatch := context.WithCancel(t.Context())
		cancelPatch()
		if got, e := stackedrecord.ReplayBlindPatchTriangles(cancelledPatch, rings, topLoops,
			false, proofbound.NewReplayTrialBudget()); !errors.Is(e, context.Canceled) || len(got.Triangles) != 0 {
			t.Fatalf("cancelled top cap triangles=%d error=%v", len(got.Triangles), e)
		}
		count := 0
		for _, loop := range vertices.Chords.Loops {
			count += len(loop.Samples)
		}
		work, records := consumer.Usage()
		bytes := consumer.Bytes()
		t.Logf("source-bound slotted wall stage samples=%d triangles=%d work=%d records=%d bytes=%d",
			count, len(walls.Triangles), work, records, bytes)
		for _, tc := range []struct {
			name                 string
			work, records, bytes uint64
			want                 bool
		}{
			{replayTestCaseExact, work, records, bytes, true},
			{replayTestCaseWorkShort, work - 1, records, bytes, false},
			{replayTestCaseRecordsShort, work, records - 1, bytes, false},
			{replayTestCaseBytesShort, work, records, bytes - 1, false},
		} {
			t.Run("source-bound chord/"+tc.name, func(t *testing.T) {
				limited, e := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
				if e != nil {
					t.Fatal(e)
				}
				got, admitted, e := stackedrecord.ReplayBlindMeshWallsWithSource(t.Context(),
					routed, bodyNode(host), .12, limited)
				if tc.want {
					if e != nil || !admitted || len(got.Triangles) != len(walls.Triangles) {
						t.Fatalf("exact walls admitted=%t count=%d error=%v", admitted, len(got.Triangles), e)
					}
				} else if e == nil || admitted || len(got.Triangles) != 0 {
					t.Fatalf("short walls admitted=%t count=%d error=%v", admitted, len(got.Triangles), e)
				}
			})
		}
		cancelled, cancel := context.WithCancel(t.Context())
		cancel()
		cancelMeter := proofbound.NewReplayTrialBudget()
		got, admitted, e := stackedrecord.ReplayBlindMeshWallsWithSource(cancelled,
			routed, bodyNode(host), .12, cancelMeter)
		if !errors.Is(e, context.Canceled) || admitted || len(got.Triangles) != 0 {
			t.Fatalf("cancelled walls admitted=%t count=%d error=%v", admitted, len(got.Triangles), e)
		}
	}
	hostFaces := 2 + len(hostPayload.Profile.Outer.Segments)
	for _, hole := range hostPayload.Profile.Holes {
		hostFaces += len(hole.Segments)
	}
	if ledger.Len() != hostFaces+8 {
		t.Fatalf("ledger faces=%d", ledger.Len())
	}
	work, records := budget.Usage()
	ledgerWork, ledgerRecords, ledgerBytes := work-beforeW, records-beforeR, budget.Bytes()-beforeB
	t.Logf("ledger faces=%d work=%d records=%d bytes=%d", ledger.Len(), ledgerWork, ledgerRecords, ledgerBytes)
	var leafCounts [4]int
	hostCount := 0
	for i := range ledger.Len() {
		entry, found := ledger.Entry(i)
		if !found || entry.Output == nil || entry.Immediate == nil || entry.Leaf == nil || entry.SourceBody == nil {
			t.Fatalf("entry %d incomplete", i)
		}
		if entry.SourceBody == bodyNode(host) {
			hostCount++
		}
		for j, leaf := range nodes {
			if entry.SourceBody == leaf {
				leafCounts[j]++
			}
		}
	}
	if hostCount != hostFaces || leafCounts != [4]int{2, 2, 2, 2} {
		t.Fatalf("source face counts host=%d leaves=%v", hostCount, leafCounts)
	}
	structureMeter := proofbound.NewReplayTrialBudget()
	columns, rows, patches, structured, structureErr :=
		stackedrecord.ReplayBlindTessStructure(t.Context(), result, structureMeter)
	if structureErr != nil || !structured || len(rows) != 2 || len(patches) != 4 ||
		len(columns) != 1+len(hostPayload.Profile.Holes)+4 {
		t.Fatalf("bounded structure admitted=%t columns=%d rows=%d patches=%d err=%v",
			structured, len(columns), len(rows), len(patches), structureErr)
	}
	structureWork, structureRecords := structureMeter.Usage()
	t.Logf("bounded structure work=%d records=%d bytes=%d",
		structureWork, structureRecords, structureMeter.Bytes())
	if withSlot {
		unrelated := internalBoxBody(t, doc, 30, 30, 2, 2, 2)
		publicCandidate, err = Cut(t.Context(), host, group)
		if err != nil || publicCandidate == nil || publicCandidate.blindProof == nil {
			t.Fatalf("public Cut candidate=%t proof=%t error=%v", publicCandidate != nil,
				publicCandidate != nil && publicCandidate.blindProof != nil, err)
		}
		if err := doc.requireLive(unrelated); err != nil {
			t.Fatalf("public Cut changed unrelated body: %v", err)
		}
		publicMesh, err := publicCandidate.Tessellate(t.Context(), units.Millimeters(.12), WithVerification(VerifyAll))
		if err != nil || publicMesh == nil || !publicMesh.BoundaryVerified() || !publicMesh.VolumeVerified() {
			t.Fatalf("public VerifyAll mesh=%t error=%v", publicMesh != nil, err)
		}
		cachedMesh, err := publicCandidate.Tessellate(t.Context(), units.Millimeters(.12), WithVerification(VerifyAll))
		if err != nil || cachedMesh != publicMesh {
			t.Fatalf("public cache hit matches=%t error=%v", cachedMesh == publicMesh, err)
		}
		publicMeter := proofbound.NewReplayTrialBudget()
		boundedMesh, bounded, err := groupBlindVerifyAllWithBudget(t.Context(), publicCandidate, .12, publicMeter)
		if err != nil || !bounded || boundedMesh == nil || !boundedMesh.SymDiffOK {
			t.Fatalf("metered public route admitted=%t mesh=%t error=%v", bounded, boundedMesh != nil, err)
		}
		publicWork, publicRecords := publicMeter.Usage()
		t.Logf("public .12 route work=%d records=%d bytes=%d",
			publicWork, publicRecords, publicMeter.Bytes())
		for _, tc := range []struct {
			name                 string
			work, records, bytes uint64
			want                 bool
		}{
			{replayTestCaseExact, publicWork, publicRecords, publicMeter.Bytes(), true},
			{replayTestCaseWorkShort, publicWork - 1, publicRecords, publicMeter.Bytes(), false},
			{replayTestCaseRecordsShort, publicWork, publicRecords - 1, publicMeter.Bytes(), false},
			{replayTestCaseBytesShort, publicWork, publicRecords, publicMeter.Bytes() - 1, false},
		} {
			t.Run("public .12 route/"+tc.name, func(t *testing.T) {
				limit, e := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
				if e != nil {
					t.Fatal(e)
				}
				mesh, admitted, e := groupBlindVerifyAllWithBudget(t.Context(), publicCandidate, .12, limit)
				if tc.want {
					if e != nil || !admitted || mesh == nil || !mesh.SymDiffOK {
						t.Fatalf("exact route admitted=%t mesh=%t error=%v", admitted, mesh != nil, e)
					}
				} else if e == nil || !admitted || mesh != nil {
					t.Fatalf("short route admitted=%t mesh=%t error=%v", admitted, mesh != nil, e)
				}
			})
		}
		cancelledPublic, cancelPublic := context.WithCancel(t.Context())
		cancelPublic()
		if mesh, admitted, e := groupBlindVerifyAllWithBudget(cancelledPublic, publicCandidate,
			.12, proofbound.NewReplayTrialBudget()); !errors.Is(e, context.Canceled) || !admitted || mesh != nil {
			t.Fatalf("cancelled route admitted=%t mesh=%t error=%v", admitted, mesh != nil, e)
		}
		publicOp := doc.producers[publicCandidate]
		cachedEntry := publicCandidate.n.Tess.Load()
		originalRole := publicOp.inputs[0].Role
		publicOp.inputs[0].Role = "wrong"
		if mesh, admitted, e := groupBlindVerifyAllWithBudget(t.Context(), publicCandidate,
			.12, proofbound.NewReplayTrialBudget()); e == nil || !admitted || mesh != nil {
			t.Fatalf("mutated public input admitted=%t mesh=%t error=%v", admitted, mesh != nil, e)
		}
		publicOp.inputs[0].Role = originalRole
		if publicCandidate.n.Tess.Load() != cachedEntry {
			t.Fatal("refused public route changed tessellation cache")
		}
		verified, err := publicCandidate.Tessellate(t.Context(), units.Millimeters(.12),
			WithVerification(VerifyAll))
		if err != nil || verified == nil || !verified.BoundaryVerified() || !verified.VolumeVerified() {
			t.Fatalf("public .12 VerifyAll mesh=%t error=%v", verified != nil, err)
		}
		if len(verified.Triangles()) != len(ordinaryMesh.Triangles) ||
			verified.n.VolSymDiff < ordinaryMesh.VolSymDiff {
			t.Fatalf("public .12 VerifyAll triangles=%d volume=%g ordinary=%g",
				len(verified.Triangles()), verified.n.VolSymDiff, ordinaryMesh.VolSymDiff)
		}
		t.Logf("public .12 VerifyAll triangles=%d volume=%g",
			len(verified.Triangles()), verified.n.VolSymDiff)
		return
	}
	original := input.Owners[0]
	input.Owners[0] = input.Owners[1]
	if got, yes, e := prismboolean.ReplayBlindFaceLedger(t.Context(), input, budget); e != nil || yes || got != nil {
		t.Fatalf("changed owner ok=%t error=%v", yes, e)
	}
	input.Owners[0] = original
	first, _ := ledger.Entry(0)
	area := first.Output.Area
	first.Output.Area++
	if got, yes, e := prismboolean.ReplayBlindFaceLedger(t.Context(), input, budget); e != nil || yes || got != nil {
		t.Fatalf("changed output face ok=%t error=%v", yes, e)
	}
	first.Output.Area = area
	var outputWall, outputCap, targetWall, targetCap *topo.Face
	for i := range ledger.Len() {
		entry, _ := ledger.Entry(i)
		if entry.Kind == stackedrecord.ReplayBlindWall && entry.SourceBody == bodyNode(host) {
			outputWall, targetWall = entry.Output, entry.Immediate
		}
		if entry.Kind == stackedrecord.ReplayBlindCap && entry.Output.CapRecord != nil && entry.Output.CapRecord.ZDenoted != nil {
			outputCap = entry.Output
			targetCap = entry.Immediate
		}
	}
	if outputWall == nil || targetWall == nil || outputCap == nil || targetCap == nil || targetCap.CapRecord == nil || len(outputWall.Loops) == 0 || len(outputWall.Loops[0].CoEdges) == 0 || len(targetWall.Loops) == 0 || len(targetWall.Loops[0].CoEdges) == 0 {
		t.Fatal("missing wall, cap or vertex")
	}
	vertex := outputWall.Loops[0].CoEdges[0].Edge.Start
	position := vertex.Position
	vertex.Position.X++
	if got, yes, e := prismboolean.ReplayBlindFaceLedger(t.Context(), input, budget); e != nil || yes || got != nil {
		t.Fatalf("changed output vertex ok=%t error=%v", yes, e)
	}
	vertex.Position = position
	level := outputCap.CapRecord.ZDenoted
	originalLevel := new(big.Rat).Set(level)
	level.Add(level, big.NewRat(1, 1))
	if got, yes, e := prismboolean.ReplayBlindFaceLedger(t.Context(), input, budget); e != nil || yes || got != nil {
		t.Fatalf("changed output cap level ok=%t error=%v", yes, e)
	}
	level.Set(originalLevel)
	capRecord := outputCap.CapRecord
	copyRecord := *capRecord
	outputCap.CapRecord = &copyRecord
	if got, yes, e := prismboolean.ReplayBlindFaceLedger(t.Context(), input, budget); e != nil || yes || got != nil {
		t.Fatalf("replaced output cap record ok=%t error=%v", yes, e)
	}
	outputCap.CapRecord = capRecord
	outputPayload := result.Payload.(stackedrecord.Payload)
	levelRecord := outputPayload.Slabs[0].Z0Denoted
	levelValue := prismplacement.DenotedLevel(outputPayload.Slabs[0].Z0, outputPayload.Slabs[0].Z0Delta, levelRecord)
	if levelValue == nil {
		t.Fatal("missing output slab level")
	}
	outputPayload.Slabs[0].Z0Denoted = prismplacement.DenoteLevel(levelValue, outputPayload.Slabs[0].Z0, outputPayload.Slabs[0].Z0Delta)
	if yes, e := ledger.Recheck(t.Context(), input, budget); e != nil || yes {
		t.Fatalf("replaced output slab level ok=%t error=%v", yes, e)
	}
	outputPayload.Slabs[0].Z0Denoted = levelRecord
	floor := outputPayload.Interfaces[0].LowerExposed[0].Outer.Segments[0]
	floorCircle := floor.(sectionrecord.CircleSeg)
	if floorCircle.Center.V != 0 || math.Signbit(floorCircle.Center.V) {
		t.Fatal("floor source is not positive zero")
	}
	floorCircle.Center.V = math.Copysign(0, -1)
	outputPayload.Interfaces[0].LowerExposed[0].Outer.Segments[0] = floorCircle
	if got, yes, e := prismboolean.ReplayBlindFaceLedger(t.Context(), input, budget); e != nil || yes || got != nil {
		t.Fatalf("changed floor sign bit ok=%t error=%v", yes, e)
	}
	outputPayload.Interfaces[0].LowerExposed[0].Outer.Segments[0] = floor
	surface := targetWall.Surface
	targetWall.Surface = surfacegeom.Cylinder{}
	if got, yes, e := prismboolean.ReplayBlindFaceLedger(t.Context(), input, budget); e != nil || yes || got != nil {
		t.Fatalf("changed target wall ok=%t error=%v", yes, e)
	}
	targetWall.Surface = surface
	sourceCap := targetCap.CapRecord
	sourceCapCopy := *sourceCap
	targetCap.CapRecord = &sourceCapCopy
	if yes, e := ledger.Recheck(t.Context(), input, budget); e != nil || yes {
		t.Fatalf("replaced source cap record ok=%t error=%v", yes, e)
	}
	targetCap.CapRecord = sourceCap
	sourceVertex := targetWall.Loops[0].CoEdges[0].Edge.Start
	sourcePosition := sourceVertex.Position
	if sourcePosition.Z != 0 || math.Signbit(sourcePosition.Z) {
		t.Fatal("source vertex is not positive zero")
	}
	sourceVertex.Position.Z = math.Copysign(0, -1)
	if yes, e := ledger.Recheck(t.Context(), input, budget); e != nil || yes {
		t.Fatalf("changed source vertex sign bit ok=%t error=%v", yes, e)
	}
	sourceVertex.Position = sourcePosition
	leafPayload := nodes[0].Payload.(prismrecord.Payload)
	segment := leafPayload.Profile.Outer.Segments[0]
	circle := segment.(sectionrecord.CircleSeg)
	circle.Center.U++
	leafPayload.Profile.Outer.Segments[0] = circle
	if got, yes, e := prismboolean.ReplayBlindFaceLedger(t.Context(), input, budget); e != nil || yes || got != nil {
		t.Fatalf("changed leaf source ok=%t error=%v", yes, e)
	}
	leafPayload.Profile.Outer.Segments[0] = segment
	prepare := func(t *testing.T, m *proofbound.ReplayBudget) prismboolean.BlindFaceLedgerInput {
		t.Helper()
		s, yes, e := groupUnionSourceSnapshot(t.Context(), group, m)
		if e != nil || !yes {
			t.Fatalf("source ok=%t error=%v", yes, e)
		}
		cert, yes, e := prismcells.ReplayBlindGroupScene(t.Context(), hostPayload.Profile, slab.Regions, hostPayload.PlacementOperand(), place, hostPayload.SectionDelta, groupPayload.SectionDelta, m)
		if e != nil || !yes {
			t.Fatalf("scene ok=%t error=%v", yes, e)
		}
		p, _, yes, e := stackedrecord.ReplayBlindGroupPayload(t.Context(), hostPayload, groupPayload, cert, m)
		if e != nil || !yes {
			t.Fatalf("payload ok=%t error=%v", yes, e)
		}
		body, log, yes, e := stackedrecord.ReplayBlindGroupBuildWithSource(t.Context(), envOf(documentNode(doc), doc.nextProducerID()), p, bodyNode(host), groupPayload, cert, m)
		if e != nil || !yes {
			t.Fatalf("build ok=%t error=%v", yes, e)
		}
		leaves := make([]*topo.Body, len(s.Inputs))
		for i, v := range s.Inputs {
			leaves[i] = bodyNode(v)
		}
		return prismboolean.BlindFaceLedgerInput{Output: body, Target: bodyNode(host), Group: bodyNode(group), Leaves: leaves, Owners: s.Owners, GroupCaps: s.GroupCaps, LeafCaps: s.LeafCaps, Scene: cert, Events: log}
	}
	for _, mode := range []string{replayTestCaseExact, "work", "records", "bytes"} {
		t.Run(mode, func(t *testing.T) {
			m := proofbound.NewReplayTrialBudget()
			current := prepare(t, m)
			usedWork, usedRecords := m.Usage()
			w := proofbound.ReplayTrialWork - usedWork - ledgerWork
			r := proofbound.ReplayTrialRecords - usedRecords - ledgerRecords
			z := proofbound.ReplayTrialBytes - m.Bytes() - ledgerBytes
			switch mode {
			case "work":
				w++
			case "records":
				r++
			case "bytes":
				z++
			}
			if e := m.SpendWork(t.Context(), w); e != nil {
				t.Fatal(e)
			}
			if e := m.ReserveRecords(t.Context(), r); e != nil {
				t.Fatal(e)
			}
			if e := m.ReserveBytes(t.Context(), z); e != nil {
				t.Fatal(e)
			}
			got, yes, e := prismboolean.ReplayBlindFaceLedger(t.Context(), current, m)
			if mode == replayTestCaseExact && (e != nil || !yes || got == nil) || mode != replayTestCaseExact && (e == nil || yes || got != nil) {
				t.Fatalf("ledger ok=%t error=%v", yes, e)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, _, e := prismboolean.ReplayBlindFaceLedger(ctx, input, budget); !errors.Is(e, context.Canceled) {
		t.Fatalf("cancellation=%v", e)
	}
}
