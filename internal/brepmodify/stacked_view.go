package brepmodify

import (
	"context"
	"errors"
	"strings"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// stackedViewBlend runs blend on the faceted view of a BRep that keeps the
// slabs it was built from (an A1 union, general-boolean §3 "A1 as a brep") or
// that a blind Cut built through the same build (A7, A9). matched=false with a
// nil error means b is not such a record, or the blend does not recognize the
// selection; the caller keeps its own refusal. The view states the body's held
// mesh with each analytic face kept as a retained tag, and the retained edge
// blend proves its own result there, as it does for a mesh Boolean result.
func stackedViewBlend(ctx context.Context, env topo.Env, hooks Hooks, b *topo.Body, edges []*topo.Edge,
	blend viewBlendFunc,
) (*topo.Body, bool, error) {
	bp, ok := b.Payload.(breprecord.Payload)
	if !ok || (bp.Stack == nil && !bp.SlabCut) || len(edges) == 0 {
		return nil, false, nil
	}
	// A planar record face (a cap, an interface face, a wall on an axis-aligned
	// plane) lies on its recorded plane, so it keeps its retained tag. A swept
	// wall (wall(k): a circular or oblique wall) is rewritten between canonical
	// vertices: an arc takes its radius from its first vertex and an oblique
	// plane passes through its walked ends, so it lies only within the record's
	// section displacement of the wall it denotes, and a holed union's merged
	// carrier lies CarrierCharge from it. Where either is nonzero:
	//   - a circular wall keeps its Cylinder tag and states that displacement on
	//     it, for the retained edge blend to charge or refuse (stackedViewCharge);
	//   - an oblique planar wall keeps its Plane tag when its tilt is certified
	//     (stackedViewTilt) and states it: NormalBound as its departure and
	//     ExtentDelta as how far the true patch lies from the tag over the face;
	//   - every other swept wall stays in the view untagged, a plain faceted
	//     obstacle as a displaced prism's side faces are (brep-modify §2.2).
	tilt := stackedViewTilt(b, bp)
	out, matched, err := retainedViewBlend(ctx, env, hooks, b, edges, nil, stackedViewUntag(b, bp, tilt),
		stackedViewCharge(b, bp), tilt, blend)
	if viewMissed(ctx, err) {
		// The view never read the selection: the receivers' refusal stands.
		return nil, false, nil
	}
	return out, matched, err
}

// stackedViewUntag lists, by index in topobuild.Faces order, the faces of b
// whose surface the view must not keep as a retained tag: every face but a
// planar record face, a charged circular wall (stackedViewCharge) and a tilted
// oblique wall (tilt, from stackedViewTilt) once the record bp carries a section
// displacement or a carrier charge, and none otherwise.
func stackedViewUntag(b *topo.Body, bp breprecord.Payload, tilt map[int]viewTilt) map[int]struct{} {
	if bp.SectionDelta() == 0 && bp.CarrierCharge == 0 {
		return nil
	}
	charge := stackedViewCharge(b, bp)
	untag := map[int]struct{}{}
	for i, f := range topobuild.Faces(b) {
		_, charged := charge[i]
		_, tilted := tilt[i]
		if charged || tilted || recordPlanarFace(b, f) {
			continue
		}
		untag[i] = struct{}{}
	}
	return untag
}

// viewMissed reports whether err is a refusal raised while building the
// faceted view, before any blend read the selection, on a live context.
func viewMissed(ctx context.Context, err error) bool {
	_, ok := errors.AsType[viewBuildError](err)
	return ok && ctx.Err() == nil
}

// recordPlanarFace reports whether f is one of b's planar record faces,
// face(k) under b's own producer.
func recordPlanarFace(b *topo.Body, f *topo.Face) bool {
	for _, origin := range f.Origins {
		if origin.Producer == b.Origin.Producer && strings.HasPrefix(origin.Role, "face(") {
			return true
		}
	}
	return false
}
