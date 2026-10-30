package brepmodify

import (
	"context"

	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// shellViewBlend is a Fillet or Chamfer of a brep body whose record carries a
// stacked shell's rounded cavity (breprecord.Payload BossShell or PocketShell;
// agent-docs/brep-modify-design.md §2.1). The record has no exact route for
// any modify, so the body is restated as its faceted view (booleaneval.View):
// its own tessellation, with each face's exact surface kept as a retained tag.
// The selected edges map to the view's edges, and blend runs the retained edge
// blend (agent-docs/retained-edge-blend-design.md) on the view, whose strip
// check reads every other facet of the view, the cavity walls included, as an
// obstacle.
//
// matched is false when b's record carries no shell cavity, or when some
// selected edge maps to no single edge of the view, or when blend does not
// recognize the mapped selection; the caller then keeps its own refusal.
// refs, when non-nil, maps each selected edge to its reference face of b, and
// blend receives the image of each on the view.
func shellViewBlend(ctx context.Context, env topo.Env, hooks Hooks, b *topo.Body, edges []*topo.Edge,
	refs map[*topo.Edge]*topo.Face,
	blend func(view *topo.Body, fp facetedrecord.Payload, edges []*topo.Edge, refs map[*topo.Edge]*topo.Face) (*topo.Body, bool, error),
) (*topo.Body, bool, error) {
	bp, ok := b.Payload.(breprecord.Payload)
	if !ok || (bp.BossShell == nil && bp.PocketShell == nil) || len(edges) == 0 {
		return nil, false, nil
	}
	fp, audit, err := booleaneval.View(ctx, b, hooks.Mesher)
	if err != nil {
		return nil, true, err
	}
	volF, volBound, volRat, err := booleaneval.VolumeReading(ctx, audit, fp.VolSymDiff)
	if err != nil {
		return nil, true, err
	}
	volume := measurement.Measurement{Value: units.CubicMillimeters(volF),
		Exactness: measurement.ExactnessOf(volBound), Bound: units.CubicMillimeters(volBound)}
	view, err := facetedrecord.BuildWithProof(ctx, env, fp, audit, volume, volRat)
	if err != nil {
		return nil, true, err
	}
	fp, ok = view.Payload.(facetedrecord.Payload)
	if !ok {
		return nil, false, nil
	}
	faceMap, ok := viewFaces(b, view, fp)
	if !ok {
		return nil, false, nil
	}
	mapped := make([]*topo.Edge, len(edges))
	var viewRefs map[*topo.Edge]*topo.Face
	if refs != nil {
		viewRefs = make(map[*topo.Edge]*topo.Face, len(edges))
	}
	for i, e := range edges {
		ve, ok := viewEdge(view, faceMap, e)
		if !ok {
			return nil, false, nil
		}
		mapped[i] = ve
		if refs == nil {
			continue
		}
		ref, ok := viewFace(faceMap, ve, refs[e])
		if !ok {
			return nil, false, nil
		}
		viewRefs[ve] = ref
	}
	return blend(view, fp, mapped, viewRefs)
}

// viewFaces maps each face of b to the faces of its view that the face's own
// group built (facetedrecord.Payload FaceOf and Src). The view's groups are b's
// faces in topobuild.Faces order (booleaneval.View). ok is false when the
// view's facets name a group or face outside the two bodies.
func viewFaces(b, view *topo.Body, fp facetedrecord.Payload) (map[*topo.Face][]*topo.Face, bool) {
	source := topobuild.Faces(b)
	built := topobuild.Faces(view)
	if len(fp.FaceOf) != len(fp.Src) {
		return nil, false
	}
	out := make(map[*topo.Face][]*topo.Face, len(source))
	seen := make(map[*topo.Face]struct{}, len(built))
	for i, fi := range fp.FaceOf {
		g := fp.Src[i]
		if g < 0 || g >= len(source) || fi < 0 || fi >= len(built) {
			return nil, false
		}
		f := built[fi]
		if _, ok := seen[f]; ok {
			continue
		}
		seen[f] = struct{}{}
		out[source[g]] = append(out[source[g]], f)
	}
	return out, true
}

// viewEdge is the one edge of view between the images of e's two faces whose
// end vertices sit at e's own end positions, compared exactly. ok is false
// when e has no two faces or no such edge, or more than one.
func viewEdge(view *topo.Body, faceMap map[*topo.Face][]*topo.Face, e *topo.Edge) (*topo.Edge, bool) {
	if len(e.Faces) != 2 || e.Start == nil || e.End == nil {
		return nil, false
	}
	imageA, imageB := faceSet(faceMap[e.Faces[0]]), faceSet(faceMap[e.Faces[1]])
	var found *topo.Edge
	for _, ve := range topobuild.Edges(view) {
		if len(ve.Faces) != 2 || ve.Start == nil || ve.End == nil {
			continue
		}
		_, a0 := imageA[ve.Faces[0]]
		_, b1 := imageB[ve.Faces[1]]
		_, a1 := imageA[ve.Faces[1]]
		_, b0 := imageB[ve.Faces[0]]
		if between := (a0 && b1) || (a1 && b0); !between {
			continue
		}
		s, t := ve.Start.Position, ve.End.Position
		same := s == e.Start.Position && t == e.End.Position
		swapped := s == e.End.Position && t == e.Start.Position
		if !same && !swapped {
			continue
		}
		if found != nil {
			return nil, false
		}
		found = ve
	}
	return found, found != nil
}

// viewFace is the face of the view edge ve that is an image of ref.
func viewFace(faceMap map[*topo.Face][]*topo.Face, ve *topo.Edge, ref *topo.Face) (*topo.Face, bool) {
	if ref == nil {
		return nil, false
	}
	images := faceSet(faceMap[ref])
	for _, f := range ve.Faces {
		if _, ok := images[f]; ok {
			return f, true
		}
	}
	return nil, false
}

func faceSet(faces []*topo.Face) map[*topo.Face]struct{} {
	out := make(map[*topo.Face]struct{}, len(faces))
	for _, f := range faces {
		out[f] = struct{}{}
	}
	return out
}
