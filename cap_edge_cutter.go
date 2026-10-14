package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/capedge"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// tryObliqueCapEdgeFillet rounds one straight edge of a convex prism cap when
// an oblique neighbour prevents route E's planar-wall restatement. A quarter
// cylinder cutter runs beyond the prism in the edge direction. The boolean
// trims it against the actual neighbouring walls, including their slanted
// terminal curves, and publishes the resulting faceted body with mesh proof.
// A false result leaves the ordinary analytic route in charge.
func tryObliqueCapEdgeFillet(ctx context.Context, body *Body, pp prismPayload, edges []*Edge,
	radius, radiusDelta float64) (*Body, bool, error) {
	return tryObliqueCapEdgeCut(ctx, body, pp, edges, radius, radiusDelta, capedge.QuarterCutter(radius))
}

// tryObliqueCapEdgeChamfer assigns setbacks before using the same bounded
// boolean cutter route as a fillet. The caller has already identified one
// selected straight edge of a prism cap.
func tryObliqueCapEdgeChamfer(ctx context.Context, body *Body, pp prismPayload, edges []*Edge,
	d, dDelta float64, asym *asymmetricChamfer) (*Body, bool, error) {
	dc, ds, delta := d, d, dDelta
	if asym != nil {
		ref := asym.refs[edges[0]]
		caps := prismCapsOf(body)
		if ref == caps.start || ref == caps.end {
			dc, ds = asym.d, asym.other
		} else {
			dc, ds = asym.other, asym.d
		}
		delta = max(asym.dDelta, asym.otherDelta)
	}
	reach := max(dc, ds)
	return tryObliqueCapEdgeCut(ctx, body, pp, edges, reach, delta, capedge.ChamferCutter(dc, ds))
}

// tryObliqueCapEdgeCut trims a convex prism with a profile whose selected
// edge wedge crosses both adjacent planes. A false result leaves route E in
// charge of straight end walls and its own refusals.
func tryObliqueCapEdgeCut(ctx context.Context, body *Body, pp prismPayload, edges []*Edge,
	reach, reachDelta float64, profile profileRecord) (*Body, bool, error) {
	if len(edges) != 1 || reachDelta != 0 || pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 {
		return nil, false, nil
	}
	if _, ok := edges[0].n.Curve.(surfacegeom.Line3); !ok || edges[0].n.Start == nil || edges[0].n.End == nil {
		return nil, false, nil
	}
	if len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) < 3 ||
		pp.Z1-pp.Z0 <= 4*reach || !obliqueCapEdgeProfile(pp, edges[0], reach) {
		return nil, false, nil
	}
	facing := edges[0].Faces()
	if len(facing) != 2 {
		return nil, false, nil
	}
	var sideNormal, capNormal r3.Vec
	mid := edges[0].n.Start.Position.Add(edges[0].n.End.Position).Scale(0.5)
	axis := pp.Dir(0, 0, 1)
	for _, face := range facing {
		if _, ok := face.n.Surface.(surfacegeom.Plane); !ok {
			return nil, false, nil
		}
		n, err := face.NormalAt(mid)
		if err != nil {
			return nil, true, err
		}
		if math.Abs(n.Value.Dot(axis)) > 1-1e-12 {
			capNormal = n.Value
		} else {
			sideNormal = n.Value
		}
	}
	if capNormal.Len() == 0 || sideNormal.Len() == 0 || math.Abs(sideNormal.Dot(capNormal)) > 1e-12 {
		return nil, false, nil
	}
	frame, err := r3.NewFrame(mid, sideNormal, capNormal)
	if err != nil {
		return nil, true, fmt.Errorf(`%w: the cap-edge cutter has no frame: %v`, ErrUnsupported, err)
	}
	span := proofbound.AbsSumUpper(edges[0].n.End.Position.Sub(edges[0].n.Start.Position).Len(), 8*reach)
	if proofbound.IsNonFinite(span) {
		return nil, true, fmt.Errorf(`%w: the cap-edge cutter has no finite sweep span`, ErrNotFinite)
	}
	d := documentHandle(body.n.Doc)
	ref := d.nextProducerID()
	tool, err := bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), prismrecord.Payload{
		Profile: profile, Frame: frame, Z0: -span, Z1: span, Xform: r3.Identity(),
	}, freeform.NewFreeformWork()))
	if err != nil {
		return nil, true, err
	}
	out, err := booleanBody(ctx, meshbool.OpCut, body, tool, ref+1)
	if err != nil {
		return nil, true, err
	}
	if err := ctx.Err(); err != nil {
		return nil, true, err
	}
	d.commitSpan(out, 2, body)
	return out, true, nil
}

// obliqueCapEdgeProfile adapts the selected topology edge to its recorded
// section edge, then applies the cutter's convexity and clearance gates.
func obliqueCapEdgeProfile(pp prismPayload, edge *Edge, radius float64) bool {
	vertices, ok := capedge.ConvexVertices(pp.Profile)
	if !ok {
		return false
	}
	match := -1
	for i, a := range vertices {
		b := vertices[(i+1)%len(vertices)]
		for _, level := range []float64{pp.Z0, pp.Z1} {
			if !matchEndpoints(edge.n.Start.Position, edge.n.End.Position,
				pp.Point(a.U, a.V, level), pp.Point(b.U, b.V, level), 1e-6) {
				continue
			}
			if match >= 0 {
				return false
			}
			match = i
		}
	}
	return capedge.Eligible(vertices, match, radius)
}
