package brepmodify

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/capedge"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// ObliqueCapEdgeFillet rounds one straight edge of a convex prism cap when
// an oblique neighbour prevents route E's planar-wall restatement. A quarter
// cylinder cutter runs beyond the prism in the edge direction. The boolean
// trims it against the actual neighbouring walls, including their slanted
// terminal curves, and publishes the resulting faceted body with mesh proof.
// A false result leaves the ordinary analytic route in charge.
func ObliqueCapEdgeFillet(ctx context.Context, env topo.Env, hooks Hooks, body *topo.Body, pp prismrecord.Payload,
	edges []*topo.Edge, radius, radiusDelta float64) (Result, bool, error) {
	return obliqueCapEdgeCut(ctx, env, hooks, body, pp, edges, radius, radiusDelta, capedge.QuarterCutter(radius))
}

// ObliqueCapEdgeChamfer assigns setbacks before using the same bounded
// boolean cutter route as a fillet. The caller has already identified one
// selected straight edge of a prism cap.
func ObliqueCapEdgeChamfer(ctx context.Context, env topo.Env, hooks Hooks, body *topo.Body, pp prismrecord.Payload,
	edges []*topo.Edge, d, dDelta float64, asym *Asymmetric) (Result, bool, error) {
	dc, ds, delta := d, d, dDelta
	if asym != nil {
		ref := asym.Refs[edges[0]]
		caps := CapsOf(body)
		if ref == caps.Start || ref == caps.End {
			dc, ds = asym.D, asym.Other
		} else {
			dc, ds = asym.Other, asym.D
		}
		delta = max(asym.DDelta, asym.OtherDelta)
	}
	reach := max(dc, ds)
	return obliqueCapEdgeCut(ctx, env, hooks, body, pp, edges, reach, delta, capedge.ChamferCutter(dc, ds))
}

// obliqueCapEdgeCut trims a convex prism with a profile whose selected
// edge wedge crosses both adjacent planes. A false result leaves route E in
// charge of straight end walls and its own refusals.
func obliqueCapEdgeCut(ctx context.Context, env topo.Env, hooks Hooks, body *topo.Body, pp prismrecord.Payload,
	edges []*topo.Edge, reach, reachDelta float64, profile momentinput.Profile) (Result, bool, error) {
	if len(edges) != 1 || reachDelta != 0 || pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 {
		return Result{}, false, nil
	}
	if _, ok := edges[0].Curve.(surfacegeom.Line3); !ok || edges[0].Start == nil || edges[0].End == nil {
		return Result{}, false, nil
	}
	if len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) < 3 ||
		pp.Z1-pp.Z0 <= 4*reach || !capedge.ObliqueProfile(pp, edges[0], reach) {
		return Result{}, false, nil
	}
	facing := edges[0].Faces
	if len(facing) != 2 {
		return Result{}, false, nil
	}
	var sideNormal, capNormal r3.Vec
	mid := edges[0].Start.Position.Add(edges[0].End.Position).Scale(0.5)
	axis := pp.Dir(0, 0, 1)
	for _, face := range facing {
		if _, ok := face.Surface.(surfacegeom.Plane); !ok {
			return Result{}, false, nil
		}
		n, err := topobuild.NormalAt(face, mid)
		if err != nil {
			return Result{}, true, err
		}
		if math.Abs(n.Value.Dot(axis)) > 1-1e-12 {
			capNormal = n.Value
		} else {
			sideNormal = n.Value
		}
	}
	if capNormal.Len() == 0 || sideNormal.Len() == 0 || math.Abs(sideNormal.Dot(capNormal)) > 1e-12 {
		return Result{}, false, nil
	}
	frame, err := r3.NewFrame(mid, sideNormal, capNormal)
	if err != nil {
		return Result{}, true, fmt.Errorf(`%w: the cap-edge cutter has no frame: %v`, decaderr.ErrUnsupported, err)
	}
	span := proofbound.AbsSumUpper(edges[0].End.Position.Sub(edges[0].Start.Position).Len(), 8*reach)
	if proofbound.IsNonFinite(span) {
		return Result{}, true, fmt.Errorf(`%w: the cap-edge cutter has no finite sweep span`, decaderr.ErrNotFinite)
	}
	tool, err := prismrecord.Build(ctx, env, prismrecord.Payload{
		Profile: profile, Frame: frame, Z0: -span, Z1: span, Xform: r3.Identity(),
	}, freeform.NewFreeformWork())
	if err != nil {
		return Result{}, true, err
	}
	out, err := hooks.Boolean(ctx, meshbool.OpCut, topo.Env{Doc: env.Doc, Ref: env.Ref + 1, Alloc: env.Alloc}, body, tool)
	if err != nil {
		return Result{}, true, err
	}
	if err := ctx.Err(); err != nil {
		return Result{}, true, err
	}
	return Result{Body: out, Span: 2}, true, nil
}
