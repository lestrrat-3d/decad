package revolveloftfaces

import (
	"context"
	"fmt"
	"maps"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// capExtensionLength is the length in millimetres of the prism that the
// tangent heel Boolean glues to a tangent end cap. The exact equality
// (T ∪ E) ∩ K = T ∩ K does not depend on it, and Gate E5 decides the held
// clearance for whatever length is used.
const capExtensionLength = 1

// ExtendTangentCap returns a copy of held with the prism glued to the end
// cap, with the prism's clearance data. It refuses with ErrUnsupported when
// held is not a verified mesh, a ring vertex is not a station vertex of the
// toe record's last cap, or the extension fails its audit.
//
// The proxy carries the unextended operand's VolSymDiff and AreaSlack. That is
// valid only for the heel Boolean under Gate E5, which proves the prism
// changes no held contact and no retained piece, so every result facet is a
// piece of a non-extension facet of the operands. The proxy is never published.
func ExtendTangentCap(ctx context.Context, held *topo.Mesh, toe *facetedrecord.LoftFacesToeRecord,
	plane loftface.TangentPlane) (*topo.Mesh, loftface.Slab, error) {
	refuse := func(why string) (*topo.Mesh, loftface.Slab, error) {
		return nil, loftface.Slab{}, fmt.Errorf("%w: the tangent tooth %s", decaderr.ErrUnsupported, why)
	}
	vertices, triangles, sources, capFace := held.Vertices, held.Triangles, held.Source, toe.CapFace
	if !held.BoundaryOK || !held.SymDiffOK {
		return refuse("mesh is not verified")
	}
	link, err := TangentToeStationLink(vertices, toe)
	if err != nil {
		return nil, loftface.Slab{}, err
	}
	beta, err := topobuild.VertexBounds(held)
	if err != nil {
		return nil, loftface.Slab{}, err
	}
	var faces []*topo.Face
	ids := map[*topo.Face]int{}
	face := make([]int, len(sources))
	for i, f := range sources {
		id, ok := ids[f]
		if !ok {
			id = len(faces)
			ids[f] = id
			faces = append(faces, f)
		}
		face[i] = id
	}
	capID, ok := ids[capFace]
	if !ok {
		return refuse("mesh has no end cap facet")
	}
	ext, err := loftface.CapExtension(ctx, loftface.HeldMesh{Vertices: vertices, Triangles: triangles,
		Face: face, VertexBound: beta}, capID, plane, big.NewRat(capExtensionLength, 1), link)
	if err != nil {
		return nil, loftface.Slab{}, err
	}
	next := new(topo.Mesh)
	*next = *held
	next.Vertices = ext.Vertices
	next.Triangles = ext.Triangles
	next.VertexBound = ext.VertexBound
	next.Source = make([]*topo.Face, len(ext.Face))
	for i, id := range ext.Face {
		next.Source[i] = faces[id]
	}
	next.FaceBound = maps.Clone(held.FaceBound)
	for id, bound := range ext.FaceBound {
		next.FaceBound[faces[id]] = bound
	}
	next.Bound = 0
	for _, bound := range next.FaceBound {
		next.Bound = max(next.Bound, bound)
	}
	next.CoordBound = proofbound.AbsSumUpper(held.CoordBound, ext.CoordCharge)
	if proofbound.IsNonFinite(next.Bound) || proofbound.IsNonFinite(next.CoordBound) {
		return refuse("cap extension has an unbounded bound")
	}
	return next, ext.Slab, nil
}
