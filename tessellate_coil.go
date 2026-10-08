package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// This file is docs/helix-design.md Table CD row CD2: a coil's mesh is an
// exact restatement of the held triangles coil_build.go built and audited,
// with each vertex's β as its per-vertex bound, and the §8.1 area slack and
// §8.2 occupied-volume proof that admit it to the mesh boolean (CD3), to
// interference (CD4) and to the mass-property ladder (CD9).

// tessellateCoil restates a coil's held triangles. A tolerance below δ asks
// for a mesh closer to the body than its held vertices are, which no
// restatement can give (docs/tessellation-design.md §7's rule). Every wall
// triangle attributes to its segment's side(i, k) face and every cap
// triangle to its cap. The boundary proof is the build's own crossing audit,
// so the mesh runs no facet-contact audit of its own. The area slack and the
// occupied-volume proof run at VerifyAll alone; tessellateContext withholds
// both below it.
func tessellateCoil(ctx context.Context, b *Body, cp coilPayload, chord float64, verify Verification) (*Mesh, error) {
	if chord < cp.delta {
		return nil, fmt.Errorf(`%w: a coil restates its held vertices, which sit up to %g mm from the body; a tolerance of %g mm is finer than that`, ErrUnsupported, cp.delta, chord)
	}
	rec, err := readCoilRecord(cp)
	if err != nil {
		return nil, err
	}
	faceOfRole := map[string]*Face{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			faceOfRole[o.Role] = f
		}
	}
	face := func(role string) (*Face, error) {
		f, ok := faceOfRole[role]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, role)
		}
		return f, nil
	}
	stride := len(rec.pts)
	segment := make([]*Face, 0, stride)
	for i, idx := range rec.loopIdx {
		for k := range idx {
			f, err := face(fmt.Sprintf("side(%d,%d)", i, k))
			if err != nil {
				return nil, err
			}
			segment = append(segment, f)
		}
	}
	capStart, err := face(roleCapStart)
	if err != nil {
		return nil, err
	}
	capEnd, err := face(roleCapEnd)
	if err != nil {
		return nil, err
	}
	walls := 2 * int(rec.n) * stride
	capCount := (len(cp.tris) - walls) / 2
	if walls+2*capCount != len(cp.tris) {
		return nil, fmt.Errorf(`%w: the coil's held triangle set does not split into walls and two caps`, ErrDegenerate)
	}
	src := make([]*Face, len(cp.tris))
	for t := range cp.tris {
		switch {
		case t < walls:
			src[t] = segment[(t/2)%stride]
		case t < walls+capCount:
			src[t] = capStart
		default:
			src[t] = capEnd
		}
	}
	if err := tessellation.RequireClosedMesh(cp.tris); err != nil {
		return nil, fmt.Errorf(`%w: the coil's held triangle set is not a closed mesh`, ErrUnsupported)
	}
	mesh := &Mesh{
		vertices:  append([]r3.Vec(nil), cp.verts...),
		triangles: append([][3]int(nil), cp.tris...),
		source:    src,
	}
	mesh.setVertexBounds(cp.vertexBound)
	mesh.bound = cp.delta
	if verify < VerifyAll {
		return mesh, nil
	}
	slack, volSymDiff, err := coilMeshProofs(ctx, rec, cp, walls)
	if err != nil {
		return nil, err
	}
	mesh.areaSlack = slack
	mesh.volSymDiff = volSymDiff
	mesh.symDiffOK = true
	return mesh, nil
}

// coilMeshProofs is docs/helix-design.md §8.1's areaSlack and §8.2's
// volSymDiff over the held set: walls triangles first, two per cell in
// (station, loop, segment) order, then both caps.
//
// areaSlack chains every wall cell from its held triangles to the bilinear
// patch on its held corners (proofbound.CellTwistAreaAllow), to the bilinear
// patch on its true corners (each corner moves at most r, the largest
// station rounding), to the true cell (coil.CellProof.Density), each leg an
// integral of the absolute area-density gap at one shared parameter; the
// plane-coordinate legs are carried through the denoted map by its stretch
// and by twice its defect over the bilinear patch's own area. Each cap
// triangle adds proofbound.PerturbedTriangleAreaAllow at δ.
//
// volSymDiff composes two homotopies of the whole closed boundary: the held
// set to the triangles on the true corners, every vertex moving at most r
// (proofbound.SweptVolumeAllow over the area the motion visits), and those
// triangles to the true walls at matched parameters with both caps fixed
// (coil.CellProof.Swept per cell, carried through the map by |det L|).
func coilMeshProofs(ctx context.Context, rec coilRecord, cp coilPayload, walls int) (float64, float64, error) {
	stride := len(rec.pts)
	n := rec.n
	dt := new(big.Rat).Quo(rec.turns, big.NewRat(n, 1))
	defect := proofbound.RatFloatUp(rec.defect)
	det := proofbound.RatFloatUp(new(big.Rat).Abs(rec.det))
	r := cp.maxRound
	cells := float64(n)
	perSegment := make([]float64, stride)
	ends := make([][2]int, stride)
	swept := 0.0
	pos := 0
	for _, idx := range rec.loopIdx {
		m := len(idx)
		for k := range m {
			v, w := idx[k], idx[(k+1)%m]
			ends[pos] = [2]int{v, w}
			proof, ok := coil.CellProofUpper(rec.rho[v], rec.zeta[v], rec.rho[w], rec.zeta[w], rec.pitch, dt)
			if !ok {
				return 0, 0, fmt.Errorf(`%w: the coil wall of profile segment %d states no mesh proof`, ErrUnsupported, v)
			}
			ruling := proofbound.ProductUpper(rec.stretch, proof.Ruling)
			helix := proofbound.ProductUpper(rec.stretch, proof.Helix)
			// The bilinear patch on the held corners against the one on the
			// true corners: each derivative moves by at most 2r, so the
			// density moves by at most 2r·|∂s| + (|∂λ| + 2r)·2r.
			twoR := proofbound.ProductUpper(2, r)
			roundLeg := proofbound.AbsSumUpper(
				proofbound.ProductUpper(twoR, helix),
				proofbound.ProductUpper(proofbound.AbsSumUpper(ruling, twoR), twoR),
			)
			perSegment[pos] = proofbound.AbsSumUpper(
				proofbound.ProductUpper(rec.stretch, proof.Density),
				proofbound.ProductUpper(proofbound.ProductUpper(2, defect), proofbound.ProductUpper(proof.Ruling, proof.Helix)),
				roundLeg,
			)
			swept = proofbound.AbsSumUpper(swept, proofbound.ProductUpper(proof.Swept, cells))
			pos++
		}
	}

	slack := 0.0
	for c := range walls / 2 {
		if c%stride == 0 {
			if err := ctx.Err(); err != nil {
				return 0, 0, err
			}
		}
		j, p := c/stride, c%stride
		v, w := ends[p][0], ends[p][1]
		at := func(station, vertex int) r3.Vec { return cp.verts[station*stride+vertex] }
		twist := proofbound.CellTwistAreaAllow(at(j, v), at(j+1, v), at(j, w), at(j+1, w))
		slack = proofbound.AbsSumUpper(slack, twist, perSegment[p])
	}
	for _, t := range cp.tris[walls:] {
		slack = proofbound.AbsSumUpper(slack, proofbound.PerturbedTriangleAreaAllow(cp.verts[t[0]], cp.verts[t[1]], cp.verts[t[2]], cp.delta))
	}

	area, err := proofbound.PerturbedAreaUpperContext(ctx, cp.verts, cp.tris, r)
	if err != nil {
		return 0, 0, err
	}
	volSymDiff := proofbound.AbsSumUpper(
		proofbound.SweptVolumeAllow(r, area),
		proofbound.ProductUpper(det, swept),
	)
	if !finiteMeasurementValues(slack, volSymDiff) {
		return 0, 0, fmt.Errorf(`%w: the coil states no finite proof for a required mesh bound`, ErrUnsupported)
	}
	return slack, volSymDiff, nil
}
