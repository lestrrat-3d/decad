package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/occupancy"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// refineWorkLimit is Refine's fixed exact-work ceiling, in occupancy.Budget
// units. No caller sets it.
const refineWorkLimit = 1 << 28

// Refine returns a new body holding source's held mesh with a surface bound
// no larger than maxBound, and retires source
// (agent-docs/body-refine-design.md §7). It moves no vertex: the result has
// source's vertices, triangles, topology, Volume, Area and every vertex and
// edge reading, and fresh face identities. What changes is the bound a
// public Tessellate publishes: every point of the held mesh lies within it of
// the true boundary, and every point of the true boundary within it of the
// held mesh, so Tessellate succeeds at any tolerance at or above
// Mesh.Bound().
//
// source MUST be a live solid made by mesh Unions whose every operand traces
// back to a solid of one whole circle (an extruded cylinder, a Revolve clear of
// its axis, a Sweep along lines and arcs, or a Coil, placed or not), each Union taken
// on its own two operands, optionally followed by Cuts whose tools are
// extruded cylinders. Any other body, or a mesh
// whose bound cannot be proven within maxBound inside the fixed work ceiling,
// is ErrUnsupported and changes nothing. A Union result that was placed after
// its last Union is outside the class: placement drops the record Refine
// reads. Placing the refined result instead keeps its bound, grown by the
// rounding the placement charges. A later Boolean on the result reads
// the bounds source published, so refining never loosens what a chain admits.
//
// maxBound is a length: wrong kind is ErrUnitKind, non-finite ErrNotFinite,
// negative ErrNegativeMagnitude and zero ErrDegenerate. A nil context or body
// is ErrDegenerate. A canceled context stops the certificate before the
// document changes.
func Refine(ctx context.Context, source *Body, maxBound units.Value) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationRefine, retErr) }()
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a refinement`, ErrDegenerate)
	}
	if source == nil || source.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(source.n.Doc)
	if err := d.requireLive(source); err != nil {
		return nil, err
	}
	target, err := sectionrecord.MagnitudeIn(maxBound, units.Length, units.Millimeter, "the refinement bound")
	if err != nil {
		return nil, err
	}
	if target == 0 {
		return nil, fmt.Errorf(`%w: a zero bound admits no held mesh`, ErrDegenerate)
	}
	if source.Kind() != BodySolid {
		return nil, fmt.Errorf(`%w: Refine needs a solid body`, ErrNotSolid)
	}
	fp, ok := source.n.Payload.(facetedPayload)
	if !ok {
		return nil, fmt.Errorf(`%w: Refine takes a mesh Union result`, ErrUnsupported)
	}
	sigma, err := fp.RefinedSurfaceBounds(ctx, target, refineWorkLimit)
	if err != nil {
		return nil, err
	}
	fp.SurfaceBound = sigma
	fp.SurfaceMesh = occupancy.Bind(fp.Verts, fp.Tris)
	ectx, err := d.evalContext(ctx)
	if err != nil {
		return nil, err
	}
	node, err := facetedrecord.Build(ectx, envOf(documentNode(d), d.producerID(ectx)), fp)
	if err != nil {
		return nil, err
	}
	return commitModifyResult(ctx, OperationRefine, source, bodyHandle(node))
}
