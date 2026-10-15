package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

const spiralFirstJoinCapOffset = 0.032 // millimetres; every actual move is charged below

// spiralOffsetToothMesh makes a temporary held operand for the authenticated
// first spiral join. The exact source solid remains the original tooth. Every
// moved vertex changes only Y; its incident side facets follow the same indexed
// mesh and are audited again before the caller can use the proxy in a Boolean.
//
// The old and new boundary surfaces agree outside the swept changed facets.
// Their solids can differ only inside those sweeps, each of which lies above
// the same exact XZ triangle with vertical thickness at most maxShift. This
// proves the symmetric-difference charge even if an intermediate surface
// during the linear Y motion crosses itself.
func spiralOffsetToothMesh(ctx context.Context, blank, tooth *Body, original *Mesh,
	src []int, proof *spiralFirstJoinSource) (*Mesh, bool, error) {
	if ctx == nil || blank == nil || tooth == nil || original == nil || proof == nil ||
		proof.tooth == nil || !original.n.BoundaryOK || !original.n.SymDiffOK ||
		len(src) != len(original.n.Triangles) || len(original.n.Vertices) == 0 ||
		len(original.n.Vertices) > 1<<16 || len(original.n.Triangles) > 1<<17 ||
		len(original.n.VertexBound) != len(original.n.Vertices) ||
		len(original.n.Source) != len(original.n.Triangles) {
		return nil, false, nil
	}
	p, ok := tooth.n.Payload.(facetedPayload)
	if !ok || p.loftFacesTrim != proof.tooth {
		return nil, false, nil
	}
	toeFace := spiralTrimFaceGroup(tooth, proof.tooth.toeCapGroup)
	heelFace := spiralTrimFaceGroup(tooth, proof.tooth.heelCapGroup)
	if toeFace < 0 || heelFace < 0 || toeFace == heelFace {
		return nil, false, nil
	}
	toeGroup, heelGroup := len(blank.Faces())+toeFace, len(blank.Faces())+heelFace
	move := make([]int8, len(original.n.Vertices))
	var capFacets [2]int
	for i, tri := range original.n.Triangles {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
		}
		side := int8(0)
		switch src[i] {
		case toeGroup:
			side = -1
			capFacets[0]++
		case heelGroup:
			side = 1
			capFacets[1]++
		}
		for _, vertex := range tri {
			if vertex < 0 || vertex >= len(move) {
				return nil, false, nil
			}
			if side != 0 {
				if move[vertex] != 0 && move[vertex] != side {
					return nil, false, nil
				}
				move[vertex] = side
			}
		}
	}
	if capFacets[0] == 0 || capFacets[1] == 0 ||
		len(original.n.FaceBound) == 0 {
		return nil, false, nil
	}
	next := bornMesh(&Mesh{n: original.n})
	next.n.Vertices = append([]r3.Vec(nil), original.n.Vertices...)
	next.n.VertexBound = append([]float64(nil), original.n.VertexBound...)
	next.n.FaceBound = make(map[*topo.Face]float64, len(original.n.FaceBound))
	for face, bound := range original.n.FaceBound {
		if proofbound.IsNonFinite(bound) || bound < 0 {
			return nil, false, nil
		}
		next.n.FaceBound[face] = bound
	}
	maxShift := new(big.Rat)
	moved := 0
	for i, side := range move {
		if side == 0 {
			continue
		}
		old := original.n.Vertices[i]
		if !proofbound.FiniteVec(old) || old.Y <= 0 {
			return nil, false, nil
		}
		next.n.Vertices[i].Y += float64(side) * spiralFirstJoinCapOffset
		if !proofbound.FiniteVec(next.n.Vertices[i]) || next.n.Vertices[i].Y <= 0 {
			return nil, false, nil
		}
		actual := lfAbs(lfSub(lfF(next.n.Vertices[i].Y), lfF(old.Y)))
		if actual.Cmp(maxShift) > 0 {
			maxShift = actual
		}
		shiftUp := proofbound.RatFloatUp(actual)
		if proofbound.IsNonFinite(shiftUp) || proofbound.IsNonFinite(next.n.VertexBound[i]) ||
			next.n.VertexBound[i] < 0 {
			return nil, false, nil
		}
		next.n.VertexBound[i] = proofbound.AbsSumUpper(next.n.VertexBound[i], shiftUp)
		moved++
	}
	if moved == 0 || maxShift.Sign() == 0 {
		return nil, false, nil
	}
	maxShiftUp := proofbound.RatFloatUp(maxShift)
	if proofbound.IsNonFinite(maxShiftUp) {
		return nil, false, nil
	}
	areaXZ := new(big.Rat)
	changed := make([][3]int, 0)
	seenFace := make(map[*topo.Face]bool)
	for i, tri := range original.n.Triangles {
		if move[tri[0]] == 0 && move[tri[1]] == 0 && move[tri[2]] == 0 {
			continue
		}
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
		}
		changed = append(changed, tri)
		// The source certificate is per facet. Its old maximum may sit at an
		// unmoved corner, so charging only moved corners can understate the
		// new facet departure read from the maximum of its vertex bounds.
		largest := tri[0]
		for _, vertex := range tri {
			bound := original.n.VertexBound[vertex]
			if proofbound.IsNonFinite(bound) || bound < 0 {
				return nil, false, nil
			}
			if bound > original.n.VertexBound[largest] {
				largest = vertex
			}
		}
		required := proofbound.AbsSumUpper(original.n.VertexBound[largest], maxShiftUp)
		if proofbound.IsNonFinite(required) {
			return nil, false, nil
		}
		next.n.VertexBound[largest] = max(next.n.VertexBound[largest], required)
		face := original.n.Source[i]
		bound, present := original.n.FaceBound[face]
		if !present {
			return nil, false, nil
		}
		if !seenFace[face] {
			next.n.FaceBound[face] = proofbound.AbsSumUpper(bound, maxShiftUp)
			seenFace[face] = true
		}
		a, b, c := original.n.Vertices[tri[0]], original.n.Vertices[tri[1]], original.n.Vertices[tri[2]]
		cross := lfSub(lfMul(lfSub(lfF(b.X), lfF(a.X)), lfSub(lfF(c.Z), lfF(a.Z))),
			lfMul(lfSub(lfF(b.Z), lfF(a.Z)), lfSub(lfF(c.X), lfF(a.X))))
		areaXZ.Add(areaXZ, lfAbs(cross))
	}
	areaXZ.Quo(areaXZ, big.NewRat(2, 1))
	volumeCharge := proofbound.RatFloatUp(lfMul(maxShift, areaXZ))
	if proofbound.IsNonFinite(volumeCharge) {
		return nil, false, nil
	}
	next.n.VolSymDiff = proofbound.AbsSumUpper(original.n.VolSymDiff, volumeCharge)
	next.n.AreaSlack = proofbound.AbsSumUpper(original.n.AreaSlack,
		facetproof.MeshAreaUpper(original.n.Vertices, changed),
		facetproof.MeshAreaUpper(next.n.Vertices, changed))
	next.n.CoordBound = proofbound.AbsSumUpper(original.n.CoordBound, maxShiftUp)
	for _, bound := range next.n.FaceBound {
		next.n.Bound = max(next.n.Bound, bound)
	}
	if proofbound.IsNonFinite(next.n.VolSymDiff) || proofbound.IsNonFinite(next.n.AreaSlack) ||
		proofbound.IsNonFinite(next.n.Bound) || proofbound.IsNonFinite(next.n.CoordBound) {
		return nil, false, nil
	}
	if err := massmoment.AuditMesh(ctx, next.n.Vertices, next.n.Triangles, false); err != nil {
		if ctx.Err() != nil {
			return nil, false, ctx.Err()
		}
		return nil, false, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	return next, true, nil
}
