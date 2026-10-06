package decad

import (
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/units"
)

// This file admits a proven exact planar relation to a contact manifold
// (docs/multibody-dynamics-design.md §9.3) and maps the kernel's face ids
// back to the bodies' live topology. internal/pair builds the exact points;
// contact_faceted_patch.go turns them into bounded public witnesses.

// publishPlanarManifold computes the manifold of a Touching pair, or the
// shallow-penetration patch of an Overlapping pair, and publishes it on
// report. An Overlapping pair of two convex bodies tries §9.3's patch first;
// when that publishes nothing, a convex body poking through one face of the
// other takes §9.6's face-local patch. A Touching pair with no convex body
// publishes the contact set of a host face that holds every contact
// (pair.PlanarGuestTouch, §10.5). A withheld manifold leaves report.Manifold
// nil with its reason: AmbiguousFeature for a contact set outside §9.3's
// table or an overlap through two faces, NonConvex for a touch of two
// non-convex bodies that no host face covers, PointTooCoarse or NoNormalProof
// for a witness or normal over the request. A penetration that
// neither patch can certify otherwise keeps the reason report already carries.
// A positive band appends the lifted set of each support plane after the
// exact points (docs/multibody-dynamics-design.md §10.5).
func publishPlanarManifold(budget *workBudget, report *ContactReport, a, b *pair.PlanarSolid,
	result pair.PlanarResult, convexA, convexB bool, band proofarith.Dyadic) error {
	var points []pair.PatchPoint
	var planes []pair.SupportPlane
	overlap := false
	switch result.Relation {
	case pair.Touching:
		if !convexA && !convexB {
			// §10.5: a non-convex guest whose contacts all lie on one host face.
			manifold, err := pair.PlanarGuestTouch(a, b, result.Contacts, budget.step)
			if err != nil {
				return err
			}
			if manifold.Points == nil {
				report.Reason = ContactNonConvex
				return nil
			}
			points, planes = manifold.Points, manifold.Supports
			break
		}
		manifold, err := pair.PlanarTouchManifold(a, b, result.Contacts, convexA, convexB, budget.step)
		if err != nil {
			return err
		}
		if manifold.Points == nil {
			report.Reason = sourceBoxReason(manifold.Reason)
			return nil
		}
		points, planes = manifold.Points, manifold.Supports
	case pair.Overlapping:
		var plane *pair.SupportPlane
		if convexA && convexB {
			var err error
			points, plane, err = pair.PlanarPenetrationSupport(a, b, budget.step)
			if err != nil {
				return err
			}
		}
		if points == nil {
			// §9.6: a convex body poking through one face of any planar body.
			local, err := pair.PlanarFacePenetration(a, b, result.Crossings, convexA, convexB, budget.step)
			if err != nil {
				return err
			}
			if local.Points == nil {
				if local.Reason != pair.NoReason {
					report.Reason = sourceBoxReason(local.Reason)
				}
				return nil
			}
			points, plane = local.Points, &local.Supports[0]
		}
		if plane != nil {
			planes, overlap = []pair.SupportPlane{*plane}, true
		}
	default:
		return nil
	}
	features, err := newPlanarFeatureMap(report.A, report.B, a, b)
	if err != nil {
		return err
	}
	manifold, reason := planarPatchManifold(report.Request, points, features)
	if manifold == nil {
		report.Reason = reason
		return nil
	}
	if band.Sign() > 0 {
		lifted, err := planarLiftedSet(budget, a, b, planes, band, overlap)
		if err != nil {
			return err
		}
		if lifted != nil {
			extra, reason := planarPatchManifold(report.Request, lifted, features)
			if extra == nil {
				report.Reason = reason
				return nil
			}
			manifold.Points = append(manifold.Points, extra.Points...)
		}
	}
	report.Manifold, report.Reason = manifold, ContactNoReason
	return nil
}

// planarLiftedSet gathers the lifted points of each support plane in order.
// Every guest is read, convex or not (docs/multibody-dynamics-design.md
// §10.5): every point of the guest is a convex combination of its vertices,
// so a point within the band over the plane has a vertex within it, and each
// lifted point's claims (an exact vertex, its exact foot inside the host
// face, its exact height) hold for any guest.
func planarLiftedSet(budget *workBudget, a, b *pair.PlanarSolid, planes []pair.SupportPlane,
	band proofarith.Dyadic, overlap bool) ([]pair.PatchPoint, error) {
	var out []pair.PatchPoint
	for _, plane := range planes {
		points, err := pair.PlanarSupportSet(a, b, plane, band, overlap, budget.step)
		if err != nil {
			return nil, err
		}
		out = append(out, points...)
	}
	return out, nil
}

// planarSupportBand publishes an exact separated pair apart by at most the
// request's SupportBand as ContactBand (§10.5): its gap's upper end must be
// within the band and some support plane must hold a nonempty lifted set.
// The planes are read with the B body's faces first, then A's, each in face
// order. The gap becomes [0 ± g], g its upper end, and the manifold is every
// such plane's lifted set. A pair that meets neither stays Separated.
func planarSupportBand(budget *workBudget, report *ContactReport, a, b *pair.PlanarSolid,
	result pair.PlanarResult, band proofarith.Dyadic) error {
	if band.Sign() <= 0 || result.Gap == nil {
		return nil
	}
	upper := new(big.Rat).Add(proofarith.FloatRat(result.Gap.ValueMM), proofarith.FloatRat(result.Gap.BoundMM))
	if upper.Cmp(band.Rat()) > 0 {
		return nil
	}
	var planes []pair.SupportPlane
	for _, host := range []struct {
		isA   bool
		solid *pair.PlanarSolid
	}{{false, b}, {true, a}} {
		faces := slices.Clone(host.solid.Faces)
		slices.Sort(faces)
		for _, face := range slices.Compact(faces) {
			planes = append(planes, pair.SupportPlane{HostIsA: host.isA, Face: face})
		}
	}
	lifted, err := planarLiftedSet(budget, a, b, planes, band, false)
	if err != nil || len(lifted) == 0 {
		return err
	}
	g := ratFloatUp(upper)
	if !finiteMeasurementValues(g) {
		return nil
	}
	report.Relation, report.Reason = ContactBand, ContactNoReason
	report.Gap = &Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(g), Exactness: exactnessFromBound(g)}
	features, err := newPlanarFeatureMap(report.A, report.B, a, b)
	if err != nil {
		return err
	}
	manifold, reason := planarPatchManifold(report.Request, lifted, features)
	report.Manifold, report.Reason = manifold, reason
	return nil
}

// supportBandOf is the request's SupportBand as an exact dyadic, zero for the
// zero Value.
func supportBandOf(req ContactRequest) proofarith.Dyadic {
	band, ok := proofarith.DyOf(req.SupportBand.Base())
	if !ok || band.Sign() < 0 {
		return proofarith.DyZero()
	}
	return band
}

// planarFeatureMap resolves kernel face ids to the two bodies' live
// topology, and orders features by that topology (§9.5).
type planarFeatureMap struct {
	sides [2]planarTopology
}

type planarTopology struct {
	faces    []*Face
	faceAt   map[*Face]int
	edgeAt   map[*Edge]int
	vertexAt map[*Vertex]int
}

func newPlanarFeatureMap(a, b *Body, sa, sb *pair.PlanarSolid) (*planarFeatureMap, error) {
	m := &planarFeatureMap{}
	for i, side := range []struct {
		body  *Body
		solid *pair.PlanarSolid
	}{{a, sa}, {b, sb}} {
		topology := planarTopology{faces: side.body.Faces(), faceAt: make(map[*Face]int),
			edgeAt: make(map[*Edge]int), vertexAt: make(map[*Vertex]int)}
		for _, id := range side.solid.Faces {
			if id < 0 || id >= len(topology.faces) {
				return nil, fmt.Errorf("%w: planar snapshot names face %d of a body with %d faces",
					ErrDegenerate, id, len(topology.faces))
			}
		}
		for at, face := range topology.faces {
			topology.faceAt[face] = at
		}
		for at, edge := range side.body.Edges() {
			topology.edgeAt[edge] = at
		}
		for at, vertex := range side.body.Vertices() {
			topology.vertexAt[vertex] = at
		}
		m.sides[i] = topology
	}
	return m, nil
}

// feature resolves one side's kernel feature. A facet is its face; an edge is
// the one live Edge both of its faces share; a vertex is the one live Vertex
// every one of its faces holds. Anything else has no single source identity.
func (m *planarFeatureMap) feature(side int, f pair.PatchFeature) (ContactFeature, bool) {
	topology := &m.sides[side]
	if len(f.Faces) == 0 {
		return ContactFeature{}, false
	}
	faces := make([]*Face, len(f.Faces))
	for i, id := range f.Faces {
		faces[i] = topology.faces[id]
	}
	switch f.Kind {
	case pair.FeatureFacet:
		if len(faces) != 1 {
			return ContactFeature{}, false
		}
		return ContactFeature{Face: faces[0]}, true
	case pair.FeatureEdge:
		if len(faces) != 2 {
			return ContactFeature{}, false
		}
		var found *Edge
		for _, edge := range faces[0].Edges() {
			if edge == found || !slices.Contains(edge.Faces(), faces[1]) {
				continue
			}
			if found != nil {
				return ContactFeature{}, false
			}
			found = edge
		}
		return ContactFeature{Edge: found}, found != nil
	case pair.FeatureVertex:
		var found *Vertex
		for _, edge := range faces[0].Edges() {
			for _, vertex := range []*Vertex{edge.Start(), edge.End()} {
				if vertex == nil || vertex == found || !vertexOnFaces(vertex, faces[1:]) {
					continue
				}
				if found != nil {
					return ContactFeature{}, false
				}
				found = vertex
			}
		}
		return ContactFeature{Vertex: found}, found != nil
	}
	return ContactFeature{}, false
}

// order is the feature's rank in its body's topology: faces, then edges, then
// vertices, each in the body's own order.
func (m *planarFeatureMap) order(side int, f ContactFeature) [2]int {
	topology := &m.sides[side]
	switch {
	case f.Face != nil:
		return [2]int{0, topology.faceAt[f.Face]}
	case f.Edge != nil:
		return [2]int{1, topology.edgeAt[f.Edge]}
	default:
		return [2]int{2, topology.vertexAt[f.Vertex]}
	}
}

func vertexOnFaces(vertex *Vertex, faces []*Face) bool {
	for _, face := range faces {
		held := false
		for _, edge := range face.Edges() {
			if edge.Start() == vertex || edge.End() == vertex {
				held = true
				break
			}
		}
		if !held {
			return false
		}
	}
	return true
}
