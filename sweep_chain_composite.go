package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweeptransport"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// chainSweepSpanPayload keeps the two real chain reductions separate. A
// composite chain has no caps: each span's wall supplies its path-end rims.
type chainSweepSpanPayload struct {
	chain          chainPayload
	revolve        chainRevolvePayload
	arc            bool
	reverseArcRims bool
}

func (sp chainSweepSpanPayload) transform() r3.Transform {
	if sp.arc {
		return sp.revolve.xform
	}
	return sp.chain.xform
}

func (sp *chainSweepSpanPayload) build(ctx context.Context, d *Document, ref producerID,
	work *freeform.FreeformWork) (*Body, error) {
	if sp.arc {
		return evalChainRevolveContext(ctx, d, ref, sp.revolve, work)
	}
	return evalChainExtrudeContext(ctx, d, ref, sp.chain, work)
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
	payload := chainSweepPayload{path: path, spans: []chainSweepSpanPayload{
		{chain: chainPayload{
			chains: []chainRecord{chain}, frame: straight.prism.Frame,
			z0: straight.prism.Z0, z1: straight.prism.Z1,
			z0Delta: straight.prism.Z0Delta, z1Delta: straight.prism.Z1Delta,
			xform: r3.Identity(),
		}},
		{revolve: chainRevolvePayload{
			chains: []chainRecord{chain}, frame: arc.revolve.frame, ax: arc.revolve.ax,
			phi0: arc.revolve.phi0, phi1: arc.revolve.phi1,
			den: arc.revolve.den, xform: r3.Identity(),
		}, arc: true, reverseArcRims: arc.reverseArcCaps},
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
	if len(payload.spans) != 2 || payload.spans[0].arc || !payload.spans[1].arc {
		return nil, fmt.Errorf(`%w: composite chain Sweep has no admitted span sequence`, ErrUnsupported)
	}
	parts := make([]*Body, 2)
	for i := range payload.spans {
		part, err := payload.spans[i].build(ctx, d, ref, work)
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
	following, err := chainSpanRims(parts[1], true, payload.spans[1].reverseArcRims)
	if err != nil {
		return nil, err
	}
	if err := certifyChainRimPair(previous.end, following.start); err != nil {
		return nil, err
	}
	// Both edges denote the same two endpoints of the one recorded source
	// line. The coordinate comparison above only rejects a failed restatement;
	// it does not establish their source identity.
	rewriteCompositeVertex(parts[0], vertexHandle(previous.end.n.Start), vertexHandle(following.start.n.Start))
	rewriteCompositeVertex(parts[0], vertexHandle(previous.end.n.End), vertexHandle(following.start.n.End))
	rewriteCompositeEdge(parts[0], previous.end, following.start)
	following.start.n.Faces = append(following.start.n.Faces, previous.end.n.Faces...)

	body := bornBody(&Body{doc: d, n: topo.Body{Origin: provenance.Ref{Producer: ref, Role: provenance.RoleBody}, Kind: topo.BodySheet}})
	faces := []*Face{parts[0].Faces()[0], parts[1].Faces()[0]}
	for i, face := range faces {
		rewriteCompositeSideRoles(face, i)
		face.n.Body = bodyNode(body)
	}
	body.lumps = sheetLumps(faces)
	if err := auditCompositeBoundary(ctx, body, map[*Edge]struct{}{following.start: {}}); err != nil {
		return nil, err
	}
	area := proofbound.BoundedAdd(
		proofbound.MeasuredScalar(faces[0].n.Area, faces[0].n.AreaBound),
		proofbound.MeasuredScalar(faces[1].n.Area, faces[1].n.AreaBound))
	body.n.Area = measurementToInternal(scalarMeasurement(area, units.SquareMillimeter))
	body.n.Bounds = boxToInternal(compositeChainBox(boxFromInternal(parts[0].n.Bounds), boxFromInternal(parts[1].n.Bounds)))
	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}
	body.payload = payload
	return body, nil
}

type chainRimPair struct{ start, end *Edge }

func chainSpanRims(part *Body, arc, reverse bool) (chainRimPair, error) {
	face := part.Faces()[0]
	if len(face.loops) != 1 || len(face.loops[0].n.CoEdges) != 4 {
		return chainRimPair{}, fmt.Errorf(`%w: a composite chain span has no four-edge source wall`, ErrUnsupported)
	}
	coedges := face.loops[0].n.CoEdges
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
	if e.span.arc {
		return e.span.revolve.revolve().extentBoundedAlong(ctx, direction, work)
	}
	return e.span.chain.prism().extentBoundedAlong(ctx, direction, work, nil)
}

func auditCompositeChainSeparation(ctx context.Context, payload chainSweepPayload, parts []*Body) error {
	frames, err := sweeptransport.TransportSweepFramesContext(ctx, payload.path, planeRecord{
		Origin: payload.spans[0].chain.frame.Origin(),
		U:      payload.spans[0].chain.frame.U(), V: payload.spans[0].chain.frame.V(),
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
			Extent: chainSweepExtent{span: payload.spans[i]},
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
