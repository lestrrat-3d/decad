package retainedblend

import (
	"context"
	"fmt"
	"math"
	"slices"

	clr "github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file holds the charged cylinder wall. A retained Cylinder face (or
// faceted group) may state HasAxialDelta/AxialDelta delta >= 0: the true wall
// is a cylinder parallel to the tag whose axis offset plus radius difference
// is at most delta. Only the closed-rim route (R2) reads it; every other route
// refuses it. A blend result keeps the tag with its delta (publishChargedWalls),
// and the readers outside this package charge it or refuse it
// (agent-docs/retained-edge-blend-design.md §6.1, "Publishing a charged wall").

// rimAreaSlope bounds how fast a closed rim's exact area change moves with the
// wall radius: (pi^2 + 4*pi) < 22.44, per unit of the larger setback.
const rimAreaSlope = 22.44

// chargedCurve reports whether f is a curved tag that states a nonzero
// displacement. A plane states its displacement through AxialDelta and the
// plane routes charge it, so only a non-plane surface counts. NaN is nonzero:
// such a tag is charged here and then refused by faceDelta.
func chargedCurve(f *topo.Face) bool {
	if _, plane := f.Surface.(surfacegeom.Plane); plane {
		return false
	}
	return f.HasAxialDelta && f.AxialDelta != 0
}

// wallRecord reports whether f is a cylinder tag whose stated displacement is
// usable (finite and not negative) and whose tag is the surface its record
// states. It is the wall test of the closed-rim route, which charges the
// displacement; tagIsRecord refuses the same face.
func wallRecord(f *topo.Face) bool {
	if _, ok := f.Surface.(surfacegeom.Cylinder); !ok {
		return false
	}
	if _, ok := faceDelta(f); !ok {
		return false
	}
	return recordTag(f)
}

// cylCharge is the slack a displaced cylinder wall adds to a rim's allowance.
// The rim circle is plane meet cylinder; a parallel true wall within delta
// moves the circle, and the blend built about it, rigidly by at most delta,
// under planeAmp*delta = 4*delta/minSine, the plane rule's amplification.
func cylCharge(delta float64) float64 { return proofbound.ProductUpper(planeAmp, delta) }

// rimAreaCharge bounds how far a charged wall moves the exact area change of
// the rim's blend; it is zero for a rim whose wall states no displacement.
// Every term of an R2 rim's exact area change is linear in the wall radius R:
// the annulus pi|(R+kappa*t)^2 - R^2| has slope 2*pi*t, the band 2*pi*R*tb has
// slope 2*pi*tb, the torus pi^2*rr*rhoC - 2*pi*kappa*rr^2 has slope pi^2*rr,
// and the cone pi(|rho1| + R)*L has slope 2*pi*L <= 2*sqrt(2)*pi*max(ta, tb).
// So a true radius within delta moves it by at most (pi^2 + 4*pi)*max*delta,
// which is below 22.44*max*delta. A closed rim's areas do not depend on the
// plane's offset or the axis offset.
func rimAreaCharge(r rim, spec blendSpec) float64 {
	if r.wallDelta == 0 {
		return 0
	}
	size := max(spec.planeSetback(), spec.wallSetback(), spec.r)
	return proofbound.ProductUpper(proofbound.ProductUpper(rimAreaSlope, size), r.wallDelta)
}

// refuseChargedArc refuses an open arc a face of which is a curved tag that
// states a displacement: the displacement moves the arc's ends and span, which
// no arc route charges yet (agent-docs/boss-root-overflow-design.md §5.4, P6).
func refuseChargedArc(edge *topo.Edge) error { return refuseChargedArcProven(edge, nil, nil) }

// refuseChargedArcProven is refuseChargedArc, except that an arc passes when
// both its ends are four-face corners whose (B, G, K) tangent proves
// (agent-docs/boss-root-overflow-design.md §5.5.6). A nil tangent refuses as
// before.
func refuseChargedArcProven(edge *topo.Edge, all []*topo.Edge, tangent TangentEnds) error {
	if provenArc(tangent, all, edge) {
		return nil
	}
	if !slices.ContainsFunc(edge.Faces, func(f *topo.Face) bool { return f != nil && chargedCurve(f) }) {
		return nil
	}
	return fmt.Errorf("%w: an arc on a wall whose tag states a displacement is not built", decaderr.ErrUnsupported)
}

// chargedGroup reports whether g is a curved group that states a nonzero
// displacement (chargedCurve, for a group).
func chargedGroup(g facetedrecord.Group) bool {
	if g.Surface == nil {
		return false
	}
	if _, plane := g.Surface.(surfacegeom.Plane); plane {
		return false
	}
	return g.HasAxialDelta && g.AxialDelta != 0
}

// keptWall reports whether g is a charged Cylinder group the blend publishes
// with its tag: a retained Cylinder whose displacement is finite and not
// negative.
func keptWall(g facetedrecord.Group) bool {
	if _, cyl := g.Surface.(surfacegeom.Cylinder); !cyl || !g.Retained {
		return false
	}
	return g.AxialDelta >= 0 && !math.IsInf(g.AxialDelta, 0)
}

// publishChargedWalls publishes the charged walls of body. A retained Cylinder
// group that states a finite displacement delta >= 0 keeps its tag and delta, as
// a displaced plane does, so a later Fillet, Chamfer or Boolean reads the wall
// again (agent-docs/retained-edge-blend-design.md §6.1, "Publishing a charged
// wall"). It clears the group's ViewCharge mark, so the rebuilt body's edge
// curves take the published rule. Any other curved group that states a
// displacement is untagged: the tag, its record and its displacement drop, and
// the faces keep their facets and bounds. It clears the fields brepmodify's
// untagged clears. body is rebuilt only when some group changed.
func publishChargedWalls(ctx context.Context, env topo.Env, body *topo.Body) (*topo.Body, error) {
	fp, ok := body.Payload.(facetedrecord.Payload)
	if !ok || !slices.ContainsFunc(fp.Groups, func(g facetedrecord.Group) bool {
		return g.ViewCharge || chargedGroup(g) && !keptWall(g)
	}) {
		return body, nil
	}
	oldGeom, err := groupGeom(ctx, faceIndexOf(body), fp, true)
	if err != nil {
		return nil, err
	}
	fp.Groups = slices.Clone(fp.Groups)
	for i, g := range fp.Groups {
		g.ViewCharge = false
		if chargedGroup(g) && !keptWall(g) {
			g.Surface = nil
			g.Retained = false
			g.Denoted = nil
			g.NormalBound = 0
			g.CapRecord = nil
			g.AxialDelta = 0
			g.HasAxialDelta = false
			g.ExtentDelta = 0
		}
		fp.Groups[i] = g
	}
	return buildCharged(ctx, env, fp, oldGeom)
}

// refuseChargedJunctions reads the tangent junctions of the selected chain
// edges as if a cylinder tag that states a displacement were exact, for
// refusal only. Every outcome is a refusal or today's fall-through; a charged
// reading never admits a chain. An edge that only the charged reading makes a
// chain edge, at a junction that reading proves or cannot decide, is refused
// with the reason, so the per-edge routes never blend half a chain.
func refuseChargedJunctions(all, edges []*topo.Edge, selected map[*topo.Edge]struct{}) error {
	return refuseChargedJunctionsSkipping(all, edges, selected, nil)
}

// refuseChargedJunctionsSkipping is refuseChargedJunctions that reads no
// junction at an end of an edge that tangent proves a C3 corner
// (agent-docs/boss-root-overflow-design.md §5.5.7).
func refuseChargedJunctionsSkipping(all, edges []*topo.Edge, selected map[*topo.Edge]struct{},
	tangent TangentEnds) error {
	for _, e := range edges {
		if !chainShape(e) || Chainable(e) || !chainableMode(e, true) {
			continue
		}
		for _, v := range []*topo.Vertex{e.Start, e.End} {
			if provenEnd(tangent, all, e, v) {
				continue
			}
			for _, c := range all {
				if c == e || c.Start == c.End || (c.Start != v && c.End != v) || len(c.Faces) != 2 {
					continue
				}
				_, state, near := tangentStateMode(e, c, v, true)
				pos := selectorquery.RenderVec(v.Position)
				if near {
					return fmt.Errorf("%w: the junction at %s is nearly tangent and the tags cannot decide it "+
						"(read on tags that state a displacement)", decaderr.ErrUnsupported, pos)
				}
				if state != clr.DegYes {
					continue
				}
				if _, ok := selected[c]; ok {
					return fmt.Errorf("%w: a tangent chain whose walls state a displacement is not built "+
						"(retained-edge-blend §6.1)", decaderr.ErrUnsupported)
				}
				return fmt.Errorf("%w: the selected edge ends at the tangent junction %s whose continuation is not "+
					"selected; WithTangentChain selects the whole chain (read on tags that state a displacement)",
					decaderr.ErrUnsupported, pos)
			}
		}
	}
	return nil
}
