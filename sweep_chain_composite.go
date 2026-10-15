package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweeptransport"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// chainSweepSpanPayload keeps the two real chain reductions separate. A
// composite chain has no caps: each span's wall supplies its path-end rims.
type chainSweepSpanPayload struct {
	Chain          chainPayload
	Revolve        chainRevolvePayload
	Arc            bool
	ReverseArcRims bool
}

func (sp chainSweepSpanPayload) transform() r3.Transform {
	if sp.Arc {
		return sp.Revolve.Xform
	}
	return sp.Chain.Xform
}

func (sp *chainSweepSpanPayload) build(ctx context.Context, d *Document, ref producerID,
	work *freeform.FreeformWork) (*Body, error) {
	if sp.Arc {
		return bodyResult(revolveplan.BuildChain(ctx, envOf(documentNode(d), ref), sp.Revolve, work))
	}
	return bodyResult(chainrecord.Build(ctx, envOf(documentNode(d), ref), sp.Chain, work))
}

func (d *Document) sweepCompositeChain(ctx context.Context, chain chainRecord, plane planeRecord,
	path *sweepinput.Path) (*Body, error) {
	if err := sweepinput.CardinalChainSweepClass(chain, plane, path); err != nil {
		return nil, err
	}
	frames, err := sweeptransport.TransportSweepFramesContext(ctx, path, plane)
	if err != nil {
		return nil, err
	}
	if len(frames) != 3 {
		return nil, fmt.Errorf(`%w: composite chain Sweep has no complete transported frame chain`, ErrUnsupported)
	}
	for i, frame := range frames {
		if frame.OriginBound != 0 || frame.UBound != 0 || frame.VBound != 0 || frame.NBound != 0 {
			return nil, fmt.Errorf(`%w: composite chain Sweep frame %d has a nonzero displacement`, ErrUnsupported, i)
		}
	}
	work := freeform.NewFreeformWork()
	view := profileRecord{Outer: loopRecord(chain)}
	straight, err := compositeLineSweepSpan(view, frames[0].Frame, path.Records[0])
	if err != nil {
		return nil, err
	}
	arcPlane := planeRecord{Origin: frames[1].Frame.Origin(), U: frames[1].Frame.U(), V: frames[1].Frame.V()}
	arc, err := compositeArcSweepSpan(ctx, view, arcPlane, frames[1].Frame, path.Records[1], work)
	if err != nil {
		return nil, err
	}
	payload := chainSweepPayload{Path: path, Spans: []chainSweepSpanPayload{
		{Chain: chainPayload{
			Chains: []chainRecord{chain}, Frame: straight.Prism.Frame,
			Z0: straight.Prism.Z0, Z1: straight.Prism.Z1,
			Z0Delta: straight.Prism.Z0Delta, Z1Delta: straight.Prism.Z1Delta,
			Xform: r3.Identity(),
		}},
		{Revolve: chainRevolvePayload{
			Chains: []chainRecord{chain}, Frame: arc.Revolve.Frame, Ax: arc.Revolve.Ax,
			Phi0: arc.Revolve.Phi0, Phi1: arc.Revolve.Phi1,
			Den: arc.Revolve.Den, Xform: r3.Identity(),
		}, Arc: true, ReverseArcRims: arc.ReverseArcCaps},
	}}
	ref := d.nextProducerID()
	body, err := replayCompositeChainSweepWork(ctx, d, ref, payload, work)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

func replayCompositeChainSweepWork(ctx context.Context, d *Document, ref producerID,
	payload chainSweepPayload, work *freeform.FreeformWork) (*Body, error) {
	if len(payload.Spans) != 2 || payload.Spans[0].Arc || !payload.Spans[1].Arc {
		return nil, fmt.Errorf(`%w: composite chain Sweep has no admitted span sequence`, ErrUnsupported)
	}
	parts := make([]*Body, 2)
	for i := range payload.Spans {
		part, err := payload.Spans[i].build(ctx, d, ref, work)
		if err != nil {
			return nil, fmt.Errorf("sweep path span %d: %w", i, err)
		}
		if len(part.Faces()) != 1 || part.Kind() != BodySheet {
			return nil, fmt.Errorf(`%w: composite chain Sweep span %d has no single source wall`, ErrUnsupported, i)
		}
		parts[i] = part
	}
	if err := auditCompositeChainSeparation(ctx, payload, parts); err != nil {
		return nil, err
	}
	previous, err := chainSpanRims(parts[0], false, false)
	if err != nil {
		return nil, err
	}
	following, err := chainSpanRims(parts[1], true, payload.Spans[1].ReverseArcRims)
	if err != nil {
		return nil, err
	}
	if err := certifyChainRimPair(previous.end, following.start); err != nil {
		return nil, err
	}
	// Both edges denote the same two endpoints of the one recorded source
	// line. The coordinate comparison above only rejects a failed restatement;
	// it does not establish their source identity.
	compositesweep.RewriteVertex(bodyNode(parts[0]), previous.end.n.Start, following.start.n.Start)
	compositesweep.RewriteVertex(bodyNode(parts[0]), previous.end.n.End, following.start.n.End)
	compositesweep.RewriteEdge(bodyNode(parts[0]), edgeNode(previous.end), edgeNode(following.start))
	following.start.n.Faces = append(following.start.n.Faces, previous.end.n.Faces...)

	body := bornBody(&Body{n: topo.Body{Origin: provenance.Ref{Producer: ref, Role: provenance.RoleBody}, Kind: topo.BodySheet, Doc: documentNode(d)}})
	faces := []*Face{parts[0].Faces()[0], parts[1].Faces()[0]}
	for i, face := range faces {
		compositesweep.RewriteFaceSideRoles(faceNode(face), i)
		face.n.Body = bodyNode(body)
	}
	body.n.Lumps = topobuild.SheetLumps(handleAlloc{}, faceNodes(faces))
	if err := compositesweep.AuditBoundary(ctx, bodyNode(body), map[*topo.Edge]struct{}{edgeNode(following.start): {}}); err != nil {
		return nil, err
	}
	area := proofbound.BoundedAdd(
		proofbound.MeasuredScalar(faces[0].n.Area, faces[0].n.AreaBound),
		proofbound.MeasuredScalar(faces[1].n.Area, faces[1].n.AreaBound))
	body.n.Area = compositesweep.ScalarMeasurement(area, units.SquareMillimeter)
	body.n.Bounds = boxToInternal(compositeChainBox(boxFromInternal(parts[0].n.Bounds), boxFromInternal(parts[1].n.Bounds)))
	if err := topobuild.ValidateAnalyticMeasurements(bodyNode(body)); err != nil {
		return nil, err
	}
	body.n.Payload = payload
	return body, nil
}

type chainRimPair struct{ start, end *Edge }

func chainSpanRims(part *Body, arc, reverse bool) (chainRimPair, error) {
	face := part.Faces()[0]
	if len(face.n.Loops) != 1 || len(face.n.Loops[0].CoEdges) != 4 {
		return chainRimPair{}, fmt.Errorf(`%w: a composite chain span has no four-edge source wall`, ErrUnsupported)
	}
	coedges := face.n.Loops[0].CoEdges
	start, end := coedges[0].Edge, coedges[2].Edge
	if arc && reverse {
		start, end = end, start
	}
	if start == nil || end == nil || start == end {
		return chainRimPair{}, fmt.Errorf(`%w: a composite chain span has incomplete path rims`, ErrUnsupported)
	}
	if _, ok := start.Curve.(surfacegeom.Line3); !ok {
		return chainRimPair{}, fmt.Errorf(`%w: a composite chain start rim is not source-linear`, ErrUnsupported)
	}
	if _, ok := end.Curve.(surfacegeom.Line3); !ok {
		return chainRimPair{}, fmt.Errorf(`%w: a composite chain end rim is not source-linear`, ErrUnsupported)
	}
	return chainRimPair{start: edgeHandle(start), end: edgeHandle(end)}, nil
}

func certifyChainRimPair(from, to *Edge) error {
	return compositesweep.CertifyRimPair(
		[2]*compositesweep.RimPost{chainRimPost(vertexHandle(from.n.Start)), chainRimPost(vertexHandle(from.n.End))},
		[2]*compositesweep.RimPost{chainRimPost(vertexHandle(to.n.Start)), chainRimPost(vertexHandle(to.n.End))})
}

func chainRimPost(v *Vertex) *compositesweep.RimPost {
	if v == nil {
		return nil
	}
	return &compositesweep.RimPost{Position: v.n.Position, Bound: v.n.Bound.Base()}
}

type chainSweepExtent struct{ span chainSweepSpanPayload }

func (e chainSweepExtent) AuditExtent(ctx context.Context, direction r3.Vec,
	work *freeform.FreeformWork) (float64, float64, float64, error) {
	if e.span.Arc {
		return e.span.Revolve.Revolve().ExtentBoundedAlong(ctx, direction, work)
	}
	return e.span.Chain.Prism().ExtentBoundedAlong(ctx, direction, work, nil)
}

func auditCompositeChainSeparation(ctx context.Context, payload chainSweepPayload, parts []*Body) error {
	frames, err := sweeptransport.TransportSweepFramesContext(ctx, payload.Path, planeRecord{
		Origin: payload.Spans[0].Chain.Frame.Origin(),
		U:      payload.Spans[0].Chain.Frame.U(), V: payload.Spans[0].Chain.Frame.V(),
	})
	if err != nil {
		return err
	}
	audit := make([]compositesweep.AuditSpan, 2)
	for i := range audit {
		box := parts[i].n.Bounds
		// Audit takes the incoming start-cap normal. A chain span has no cap,
		// so derive that opposite-tangent plane from its certified frame.
		incoming, frameErr := r3.NewFrame(frames[i].Frame.Origin(),
			frames[i].Frame.U(), frames[i].Frame.V().Scale(-1))
		if frameErr != nil {
			return fmt.Errorf(`%w: composite chain join plane has no frame: %v`, ErrUnsupported, frameErr)
		}
		audit[i] = compositesweep.AuditSpan{
			StartPlane:  surfacegeom.Plane{Frame: incoming},
			StartPlanar: true, EndPlanar: true, EndpointSupports: true,
			Box:    compositesweep.AuditBox{Min: box.Min, Max: box.Max, Bound: box.Bound.Base()},
			Extent: chainSweepExtent{span: payload.Spans[i]},
		}
	}
	return compositesweep.Audit(ctx, audit)
}

func compositeChainBox(a, b Box) Box {
	bound := math.Max(a.Bound.Base(), b.Bound.Base())
	return Box{
		Min:   r3.NewVec(math.Min(a.Min.X, b.Min.X), math.Min(a.Min.Y, b.Min.Y), math.Min(a.Min.Z, b.Min.Z)),
		Max:   r3.NewVec(math.Max(a.Max.X, b.Max.X), math.Max(a.Max.Y, b.Max.Y), math.Max(a.Max.Z, b.Max.Z)),
		Bound: units.Millimeters(bound), Exactness: exactnessOf(bound),
	}
}
