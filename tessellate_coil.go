package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// This file is docs/helix-design.md Table CD row CD2: a coil's mesh is an
// exact restatement of the held triangles internal/coilshell built and audited,
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
	rec, err := coilRecordOfPayload(cp)
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
	stride := len(rec.Pts)
	segment := make([]*Face, 0, stride)
	for i, idx := range rec.LoopIdx {
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
	walls := 2 * int(rec.N) * stride
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
	slack, volSymDiff, err := coilshell.MeshProofs(ctx, rec, coilshell.MeshInput{
		Verts: cp.verts, Tris: cp.tris, Delta: cp.delta, MaxRound: cp.maxRound,
	}, walls)
	if err != nil {
		return nil, err
	}
	mesh.areaSlack = slack
	mesh.volSymDiff = volSymDiff
	mesh.symDiffOK = true
	return mesh, nil
}
