package export

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/step/ap214"
)

// errPatchUnoriented reports a patch face whose STEP walk the shared edges
// do not decide: no neighbour reaches it, or two neighbours disagree.
var errPatchUnoriented = errors.New("a patch face's walk is not decided by its neighbours")

// loopVerticesExact reports whether every vertex a loop walks is Exact. It
// only refuses: a patch vertex whose position carries a bound is not proven
// to lie on the arcs, generators and surface the file states through it, so
// the body takes the faceted writer.
func loopVerticesExact(loop *decad.Loop) bool {
	for _, ce := range loop.CoEdges() {
		if ce.Start().Position().Exactness != decad.Exact {
			return false
		}
	}
	return true
}

// arcInItsPlane reports whether both of an arc edge's vertices lie in the
// plane through the arc's Center normal to its Axis, by an exactly zero dot
// product. It only refuses: an arc record whose Axis does not hold its own
// vertices states a circle the edge does not run on.
func arcInItsPlane(edge *decad.Edge, arc decad.Arc3) bool {
	return edge.Start().Position().Value.Sub(arc.Center).Dot(arc.Axis) == 0 &&
		edge.End().Position().Value.Sub(arc.Center).Dot(arc.Axis) == 0
}

// isWalkedPatch reports whether a face is written by the edge-use rule
// (patchWalks): a cone or sphere face bounded by one loop.
func isWalkedPatch(face *decad.Face) bool {
	switch face.Surface().(type) {
	case decad.Cone, decad.Sphere:
		return len(face.Loops()) == 1
	}
	return false
}

// ownWalkReversed is whether a face that is not a walked patch writes its
// loops against their source walk, as its own writer decides: a plane when
// its outward normal opposes its placement axis (addPlanarFace), a partial
// cylinder wall by its (θ, z) area (partialWallSense), a torus patch by its
// (θ, φ) area (torusPatchReversed), and a full wall of circles never.
func ownWalkReversed(face *decad.Face) (bool, error) {
	loops := face.Loops()
	switch surface := face.Surface().(type) {
	case decad.Plane:
		_, axis, _, err := planarSTEPPlacement(face, loops[0])
		if err != nil {
			return false, err
		}
		normal, err := face.NormalAt(loops[0].CoEdges()[0].Start().Position().Value)
		if err != nil {
			return false, err
		}
		return normal.Value.Dot(axis) <= 0, nil
	case decad.Cylinder:
		if len(loops) != 1 {
			return false, nil
		}
		reference, ok := radialAbout(surface.Origin, surface.Axis, loops[0].CoEdges()[0].Start().Position().Value)
		if !ok {
			return false, fmt.Errorf("%w: cylinder has no radial reference", decad.ErrDegenerate)
		}
		_, reverse, err := partialWallSense(face, surface, loops[0], reference)
		return reverse, err
	case decad.Torus:
		if len(loops) != 1 {
			return false, nil
		}
		major, _, ok := torusRadii(surface)
		if !ok {
			return false, fmt.Errorf("%w: torus has no positive radii", decad.ErrDegenerate)
		}
		sameSense, err := torusFaceSense(face, surface, major, loops[0].CoEdges()[0].Start().Position().Value)
		if err != nil {
			return false, err
		}
		return torusPatchReversed(surface, major, loops[0], sameSense)
	}
	return false, nil
}

// patchWalks decides the STEP walk of every walked patch (isWalkedPatch) by
// the edge uses it shares (decideWalks), never by a parameter-plane area,
// which a cone's apex vertex leaves undefined and a sphere has no seam-free
// chart for. Every other face's walk is its own writer's (ownWalkReversed).
// The result maps each patch to whether its loop is written against its
// source walk; it is empty, and no other face's walk is computed, when the
// body has no patch.
func patchWalks(ctx context.Context, body *decad.Body) (map[*decad.Face]bool, error) {
	steps := func(face *decad.Face) []walkStep[*decad.Edge] {
		var out []walkStep[*decad.Edge]
		for _, loop := range face.Loops() {
			for _, ce := range loop.CoEdges() {
				out = append(out, walkStep[*decad.Edge]{edge: ce.Edge(), forward: ce.IsForward()})
			}
		}
		return out
	}
	var fixed []walkFace[*decad.Face, *decad.Edge]
	var pending []walkFace[*decad.Face, *decad.Edge]
	for _, face := range body.Faces() {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if isWalkedPatch(face) {
			pending = append(pending, walkFace[*decad.Face, *decad.Edge]{face: face, steps: steps(face)})
			continue
		}
		fixed = append(fixed, walkFace[*decad.Face, *decad.Edge]{face: face, steps: steps(face)})
	}
	if len(pending) == 0 {
		return map[*decad.Face]bool{}, nil
	}
	for i := range fixed {
		reverse, err := ownWalkReversed(fixed[i].face)
		if err != nil {
			return nil, err
		}
		fixed[i].reverse = reverse
	}
	return decideWalks(fixed, pending)
}

// walkStep is one directed use of an edge by a face's loops: forward when it
// runs along the edge's own direction.
type walkStep[E comparable] struct {
	edge    E
	forward bool
}

// walkFace is a face's edge uses and, once decided, whether they are written
// reversed.
type walkFace[F, E comparable] struct {
	face    F
	steps   []walkStep[E]
	reverse bool
}

// decideWalks decides each pending face's walk from the edges it shares with
// faces already decided, starting from fixed. A closed shell uses each edge
// once in each direction, so a pending face is walked so that every edge it
// shares with a decided face runs opposite to that face's use; the decided set
// grows until no face is pending. A pending face that no decided face reaches,
// or one whose shared edges ask for both walks, is errPatchUnoriented: the
// check only refuses, and every shared edge of a decided face is compared,
// never only the first.
func decideWalks[F, E comparable](fixed, pending []walkFace[F, E]) (map[F]bool, error) {
	// uses holds, for each edge, the face that first walked it and whether
	// that walk, as written, runs along the edge's own direction.
	type use struct {
		face    F
		forward bool
	}
	uses := make(map[E]use)
	record := func(f walkFace[F, E]) {
		for _, st := range f.steps {
			if _, ok := uses[st.edge]; !ok {
				uses[st.edge] = use{face: f.face, forward: st.forward != f.reverse}
			}
		}
	}
	for _, f := range fixed {
		record(f)
	}
	walks := make(map[F]bool, len(pending))
	for len(pending) > 0 {
		var next []walkFace[F, E]
		for _, f := range pending {
			decided, reverse := false, false
			for _, st := range f.steps {
				u, ok := uses[st.edge]
				if !ok || u.face == f.face {
					continue
				}
				// This face's use must oppose u: st.forward != reverse == !u.forward.
				want := st.forward == u.forward
				if decided && want != reverse {
					return nil, fmt.Errorf("%w: its shared edges ask for both walks", errPatchUnoriented)
				}
				decided, reverse = true, want
			}
			if !decided {
				next = append(next, f)
				continue
			}
			f.reverse = reverse
			walks[f.face] = reverse
			record(f)
		}
		if len(next) == len(pending) {
			return nil, fmt.Errorf("%w: no decided face shares an edge with it", errPatchUnoriented)
		}
		pending = next
	}
	return walks, nil
}

// addWalkedLoopFace writes a patch face of one loop on surface: the loop is
// walked as patchWalks decided, so it runs opposite to every neighbour's use
// of a shared edge, and the face sense is the face's outward normal against
// the surface's own.
func (b *analyticSTEPBuilder) addWalkedLoopFace(
	ctx context.Context, face *decad.Face, surface step.Reference, sameSense bool,
) (step.Reference, error) {
	reverse, ok := b.walks[face]
	if !ok {
		return 0, fmt.Errorf("%w: a patch face has no decided walk", decad.ErrUnsupported)
	}
	coedges := face.Loops()[0].CoEdges()
	oriented := make([]step.Reference, 0, len(coedges))
	for i := range coedges {
		if err := ctx.Err(); err != nil {
			return 0, err
		}
		ce := coedges[i]
		if reverse {
			ce = coedges[len(coedges)-1-i]
		}
		edge, err := b.addEdge(ce.Edge())
		if err != nil {
			return 0, err
		}
		oriented = append(oriented, b.add(ap214.OrientedEdge(0, "", edge, ce.IsForward() != reverse)))
	}
	bound := b.add(ap214.FaceOuterBound(0, "", b.add(ap214.EdgeLoop(0, "", oriented...)), true))
	return b.add(ap214.AdvancedFace(0, "", surface, sameSense, bound)), nil
}
