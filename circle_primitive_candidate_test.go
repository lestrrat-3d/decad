package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismtess"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func publicCirclePrimitive(t *testing.T) *Body {
	t.Helper()
	d := New()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		t.Fatal(err)
	}
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 3)
	if _, err = s.Solve(t.Context()); err != nil {
		t.Fatal(err)
	}
	source, err := Extrude(t.Context(), d, s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along})
	if err != nil {
		t.Fatal(err)
	}
	turn, err := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(.37))
	if err != nil {
		t.Fatal(err)
	}
	shift, err := r3.Translation(r3.Vec{X: 4, Y: -6, Z: 2})
	if err != nil {
		t.Fatal(err)
	}
	placement, err := turn.Then(shift)
	if err != nil {
		t.Fatal(err)
	}
	placed, err := Transform(t.Context(), source, placement)
	if err != nil {
		t.Fatal(err)
	}
	ops := d.OperationHistory().Operations()
	if len(ops) != 2 || ops[0].Kind() != OperationExtrude || ops[1].Kind() != OperationTransform {
		t.Fatalf("unexpected producer history: %d", len(ops))
	}
	if len(ops[0].Outputs()) != 1 || ops[0].Outputs()[0] != source ||
		len(ops[1].Inputs()) != 1 || ops[1].Inputs()[0].Body != source ||
		len(ops[1].Outputs()) != 1 || ops[1].Outputs()[0] != placed {
		t.Fatal("producer input/output identity changed")
	}
	return placed
}

func publicCircleCandidate(ctx context.Context, body *Body, b *proofbound.ReplayBudget) (*topo.Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if b == nil || body == nil {
		return nil, decaderr.ErrUnsupported
	}
	if err := b.SpendWork(ctx, 128); err != nil {
		return nil, err
	}
	p, ok := body.n.Payload.(prismrecord.Payload)
	if !ok || len(p.Profile.Holes) != 0 || len(p.Profile.Outer.Segments) != 1 {
		return nil, decaderr.ErrUnsupported
	}
	seg, ok := p.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok {
		return nil, decaderr.ErrUnsupported
	}
	var faces [3]*topo.Face
	for _, l := range body.n.Lumps {
		if err := b.SpendWork(ctx, 64); err != nil {
			return nil, err
		}
		if l == nil {
			return nil, decaderr.ErrUnsupported
		}
		for _, sh := range l.Shells {
			if err := b.SpendWork(ctx, 64); err != nil {
				return nil, err
			}
			if sh == nil {
				return nil, decaderr.ErrUnsupported
			}
			for _, f := range sh.Faces {
				if err := b.SpendWork(ctx, 128); err != nil {
					return nil, err
				}
				token, ok := f.PrismConstruction()
				if !ok {
					return nil, decaderr.ErrUnsupported
				}
				for i, k := range []topo.PrismFaceKind{topo.PrismFaceWall, topo.PrismFaceCapStart, topo.PrismFaceCapEnd} {
					loop, segIndex := -1, -1
					if i == 0 {
						loop, segIndex = 0, 0
					}
					if token.Matches(f, &body.n, body.n.Origin.Producer, k, loop, segIndex) {
						if faces[i] != nil {
							return nil, decaderr.ErrUnsupported
						}
						if f.CapRecord != nil && f.CapRecord.ZDenoted != nil {
							if f.CapRecord.ZDenoted.Num().BitLen() > 4096 || f.CapRecord.ZDenoted.Denom().BitLen() > 4096 {
								return nil, decaderr.ErrUnsupported
							}
							if err := b.SpendWork(ctx, 256); err != nil {
								return nil, err
							}
						}
						if !token.MatchesSealed(f) {
							return nil, decaderr.ErrUnsupported
						}
						valid, err := token.ReplayMatchesTopology(ctx, f, b)
						if err != nil {
							return nil, err
						}
						if !valid {
							return nil, decaderr.ErrUnsupported
						}
						faces[i] = f
					}
				}
			}
		}
	}
	if faces[0] == nil || faces[1] == nil || faces[2] == nil {
		return nil, decaderr.ErrUnsupported
	}
	local, lift, charge, err := massmoment.ReplayPrismChord(ctx, p.Frame, p.Xform, .2, b)
	if err != nil {
		return nil, err
	}
	samples, err := tessellation.CircleSamplesReplay(ctx, seg, faces[0], local, p.Z1-p.Z0, b)
	if err != nil {
		return nil, err
	}
	mesh, err := prismtess.CircleCandidateReplay(ctx, &body.n, p, seg, faces, samples, lift, charge, b)
	if err != nil {
		return nil, err
	}
	for _, f := range faces {
		if err := b.SpendWork(ctx, 256); err != nil {
			return nil, err
		}
		if f.CapRecord != nil && f.CapRecord.ZDenoted != nil &&
			(f.CapRecord.ZDenoted.Num().BitLen() > 4096 || f.CapRecord.ZDenoted.Denom().BitLen() > 4096) {
			return nil, decaderr.ErrUnsupported
		}
		token, ok := f.PrismConstruction()
		if !ok || !token.MatchesSealed(f) {
			return nil, decaderr.ErrUnsupported
		}
		valid, err := token.ReplayMatchesTopology(ctx, f, b)
		if err != nil {
			return nil, err
		}
		if !valid {
			return nil, decaderr.ErrUnsupported
		}
	}
	return mesh, nil
}

func TestPublicCirclePrimitiveCandidate(t *testing.T) {
	body := publicCirclePrimitive(t)
	b := proofbound.NewCircleReplayBudget()
	mesh, err := publicCircleCandidate(t.Context(), body, b)
	if err != nil {
		t.Fatal(err)
	}
	if mesh == nil || len(mesh.Vertices) == 0 || len(mesh.Triangles) == 0 || len(mesh.Source) != len(mesh.Triangles) || mesh.BoundaryOK || mesh.SymDiffOK || math.IsNaN(mesh.Bound) || math.IsInf(mesh.Bound, 0) {
		t.Fatal("invalid or published candidate")
	}
	for i, f := range mesh.Source {
		if f == nil || f.Body != &body.n {
			t.Fatalf("facet %d lost its source face", i)
		}
	}
	w, r := b.Usage()
	z := b.Bytes()
	t.Logf("public transformed source candidate: work=%d records=%d bytes=%d vertices=%d facets=%d", w, r, z, len(mesh.Vertices), len(mesh.Triangles))
	for _, mode := range []string{"exact", "work", "records", "bytes"} {
		t.Run(mode, func(t *testing.T) {
			m := proofbound.NewCircleReplayBudget()
			ww, rr, zz := proofbound.ReplayTrialWork-w, proofbound.ReplayTrialRecords-r, proofbound.ReplayTrialBytes-z
			switch mode {
			case "work":
				ww++
			case "records":
				rr++
			case "bytes":
				zz++
			}
			if err := m.SpendWork(t.Context(), ww); err != nil {
				t.Fatal(err)
			}
			if err := m.ReserveRecords(t.Context(), rr); err != nil {
				t.Fatal(err)
			}
			if err := m.ReserveBytes(t.Context(), zz); err != nil {
				t.Fatal(err)
			}
			got, err := publicCircleCandidate(t.Context(), body, m)
			if mode == "exact" && err != nil || mode != "exact" && (!errors.Is(err, decaderr.ErrUnsupported) || got != nil) {
				t.Fatalf("%s: %v", mode, err)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if got, err := publicCircleCandidate(ctx, body, proofbound.NewCircleReplayBudget()); !errors.Is(err, context.Canceled) || got != nil {
		t.Fatalf("cancel: %v", err)
	}
}

//nolint:containedctx // The context is confined to this synchronous replay.
type circlePrimitiveCancelContext struct {
	context.Context
	polls int
}

func (c *circlePrimitiveCancelContext) Err() error {
	c.polls++
	if c.polls > 500 {
		return context.Canceled
	}
	return nil
}

func TestPublicCirclePrimitiveActiveCancellation(t *testing.T) {
	body := publicCirclePrimitive(t)
	ctx := &circlePrimitiveCancelContext{Context: t.Context()}
	mesh, err := publicCircleCandidate(ctx, body, proofbound.NewCircleReplayBudget())
	if !errors.Is(err, context.Canceled) || mesh != nil || ctx.polls <= 500 {
		t.Fatalf("active cancellation: %v polls=%d", err, ctx.polls)
	}
}

func TestPublicCirclePrimitiveSourceMutation(t *testing.T) {
	body := publicCirclePrimitive(t)
	face := body.n.Lumps[0].Shells[0].Faces[0]
	if len(face.Loops) == 0 || len(face.Loops[0].CoEdges) == 0 {
		t.Fatal("missing source topology")
	}
	edge := face.Loops[0].CoEdges[0].Edge
	old := edge.Start.Position
	edge.Start.Position.X++
	mesh, err := publicCircleCandidate(t.Context(), body, proofbound.NewCircleReplayBudget())
	edge.Start.Position = old
	if !errors.Is(err, decaderr.ErrUnsupported) || mesh != nil {
		t.Fatalf("mutated source: %v", err)
	}
}

func TestPublicCirclePrimitiveFaceTokenMutations(t *testing.T) {
	body := publicCirclePrimitive(t)
	faces := body.n.Lumps[0].Shells[0].Faces
	if len(faces) != 3 {
		t.Fatalf("face count=%d", len(faces))
	}
	wall := faces[0]
	if len(wall.Loops) == 0 || len(wall.Loops[0].CoEdges) == 0 {
		t.Fatal("missing wall loop")
	}
	loop := wall.Loops[0]
	edge := loop.CoEdges[0].Edge
	refuse := func(name string) {
		t.Helper()
		m, err := publicCircleCandidate(t.Context(), body, proofbound.NewCircleReplayBudget())
		if !errors.Is(err, decaderr.ErrUnsupported) || m != nil {
			t.Fatalf("%s: %v", name, err)
		}
	}
	oldRole := wall.Origins[0].Role
	wall.Origins[0].Role = "changed"
	refuse("role")
	wall.Origins[0].Role = oldRole
	oldBody := wall.Body
	source := body.Document().OperationHistory().Operations()[1].Inputs()[0].Body
	wall.Body = &source.n
	refuse("body")
	wall.Body = oldBody
	oldProducer := body.n.Origin.Producer
	body.n.Origin.Producer++
	refuse("producer")
	body.n.Origin.Producer = oldProducer
	loop.CoEdges[0].Forward = !loop.CoEdges[0].Forward
	refuse("coedge")
	loop.CoEdges[0].Forward = !loop.CoEdges[0].Forward
	if len(edge.Faces) == 0 {
		t.Fatal("missing adjacency")
	}
	oldAdjacent := edge.Faces[0]
	edge.Faces[0] = nil
	refuse("adjacency")
	edge.Faces[0] = oldAdjacent
	var capRecord *topo.CapRecord
	for _, f := range faces {
		if f.CapRecord != nil {
			capRecord = f.CapRecord
			break
		}
	}
	if capRecord == nil || capRecord.ZDenoted == nil {
		t.Fatal("missing cap denotation")
	}
	oldDenoted := capRecord.ZDenoted
	capRecord.ZDenoted = new(big.Rat).Set(oldDenoted)
	refuse("cap rational identity")
	capRecord.ZDenoted = oldDenoted
}

//nolint:containedctx // The context injects one mutation during a synchronous replay.
type circlePrimitiveMutationContext struct {
	context.Context
	polls  int
	mutate func()
}

func (c *circlePrimitiveMutationContext) Err() error {
	c.polls++
	if c.polls == 500 {
		c.mutate()
	}
	return nil
}
func TestPublicCirclePrimitiveMutationDuringReplay(t *testing.T) {
	body := publicCirclePrimitive(t)
	face := body.n.Lumps[0].Shells[0].Faces[0]
	edge := face.Loops[0].CoEdges[0].Edge
	old := edge.Start.Position
	ctx := &circlePrimitiveMutationContext{Context: t.Context(), mutate: func() { edge.Start.Position.X++ }}
	mesh, err := publicCircleCandidate(ctx, body, proofbound.NewCircleReplayBudget())
	edge.Start.Position = old
	if ctx.polls < 500 || !errors.Is(err, decaderr.ErrUnsupported) || mesh != nil {
		t.Fatalf("mutation during replay: %v polls=%d", err, ctx.polls)
	}
}
