package export

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/step/ap214"
	"github.com/lestrrat-3d/units"
)

// supportsAnalyticSpherePatch admits a sphere face of one loop of at least
// three Arc3 edges, each a great circle: its Center the sphere's and its
// Radius the sphere's, both exactly, with both vertices in its plane
// (arcInItsPlane), and every vertex Exact (loopVerticesExact). A box corner's fillet ball is such a
// loop, three quarter arcs about the corner's three axes. A sphere with no
// loop, a whole revolved ball, is written faceted.
func supportsAnalyticSpherePatch(loop *decad.Loop, sphere decad.Sphere) bool {
	coedges := loop.CoEdges()
	if len(coedges) < 3 || !loopVerticesExact(loop) {
		return false
	}
	radius, err := sphere.Radius.In(units.Millimeter)
	if err != nil || radius <= 0 {
		return false
	}
	for _, ce := range coedges {
		arc, ok := ce.Edge().Curve().(decad.Arc3)
		if !ok || arc.Center != sphere.Center || !arcInItsPlane(ce.Edge(), arc) {
			return false
		}
		r, err := arc.Radius.In(units.Millimeter)
		if err != nil || r != radius {
			return false
		}
	}
	return true
}

// addSphereFace writes a sphere patch (supportsAnalyticSpherePatch) as a
// SPHERICAL_SURFACE at its Center with its Radius, placed about the world's
// z axis. The STEP surface's normal points away from the centre, so the
// face sense is the face's outward normal at its first vertex against that
// direction, and its loop is walked as patchWalks decided.
func (b *analyticSTEPBuilder) addSphereFace(ctx context.Context, face *decad.Face, sphere decad.Sphere) (step.Reference, error) {
	radius, err := sphere.Radius.In(units.Millimeter)
	if err != nil {
		return 0, err
	}
	p := face.Loops()[0].CoEdges()[0].Start().Position().Value
	own, ok := p.Sub(sphere.Center).Normalize()
	if !ok {
		return 0, fmt.Errorf("%w: sphere patch vertex is its centre", decad.ErrDegenerate)
	}
	normal, err := face.NormalAt(p)
	if err != nil {
		return 0, err
	}
	placement := b.addPlacement(sphere.Center, r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0))
	surface := b.add(ap214.SphericalSurface(0, "", placement, step.Real(radius)))
	return b.addWalkedLoopFace(ctx, face, surface, normal.Value.Dot(own) > 0)
}
