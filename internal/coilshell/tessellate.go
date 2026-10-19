package coilshell

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// This file is docs/helix-design.md Table CD row CD2: a coil's mesh is an
// exact restatement of the held triangles Build made and audited, with each
// vertex's β as its per-vertex bound, and the §8.1 area slack and §8.2
// occupied-volume proof that admit it to the mesh boolean (CD3), to
// interference (CD4) and to the mass-property ladder (CD9).

// Tessellate restates the held triangles of body, whose payload is cp, as a
// mesh node. A tolerance below δ asks for a mesh closer to the body than its
// held vertices are, which no restatement can give
// (docs/tessellation-design.md §7's rule). Every wall triangle attributes to
// the side(i, k) face of the recorded segment whose chord its cell sweeps,
// and every cap triangle to its cap. The boundary proof is the build's own
// crossing audit, so the mesh runs no facet-contact audit of its own.
// verifyAll decides whether the area slack and the occupied-volume proof are
// composed at all.
func Tessellate(ctx context.Context, body *topo.Body, cp Payload, chord float64, verifyAll bool) (*topo.Mesh, error) {
	if chord < cp.Delta {
		return nil, fmt.Errorf(`%w: a coil restates its held vertices, which sit up to %g mm from the body; a tolerance of %g mm is finer than that`, decaderr.ErrUnsupported, cp.Delta, chord)
	}
	rec, err := cp.Record()
	if err != nil {
		return nil, err
	}
	faceOfRole := map[string]*topo.Face{}
	for _, f := range topobuild.Faces(body) {
		for _, o := range f.Origins {
			faceOfRole[o.Role] = f
		}
	}
	face := func(role string) (*topo.Face, error) {
		f, ok := faceOfRole[role]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, decaderr.ErrDegenerate, role)
		}
		return f, nil
	}
	stride := len(rec.Pts)
	segment := make([]*topo.Face, 0, len(rec.Profile.Segments))
	for _, seg := range rec.Profile.Segments {
		f, err := face(fmt.Sprintf("side(%d,%d)", seg.Loop, seg.Index))
		if err != nil {
			return nil, err
		}
		segment = append(segment, f)
	}
	capStart, err := face(provenance.RoleCapStart)
	if err != nil {
		return nil, err
	}
	capEnd, err := face(provenance.RoleCapEnd)
	if err != nil {
		return nil, err
	}
	walls := 2 * int(rec.N) * stride
	capCount := (len(cp.Tris) - walls) / 2
	if walls+2*capCount != len(cp.Tris) {
		return nil, fmt.Errorf(`%w: the coil's held triangle set does not split into walls and two caps`, decaderr.ErrDegenerate)
	}
	src := make([]*topo.Face, len(cp.Tris))
	for t := range cp.Tris {
		switch {
		case t < walls:
			src[t] = segment[rec.Profile.Chord[(t/2)%stride].Segment]
		case t < walls+capCount:
			src[t] = capStart
		default:
			src[t] = capEnd
		}
	}
	if err := tessellation.RequireClosedMesh(cp.Tris); err != nil {
		return nil, fmt.Errorf(`%w: the coil's held triangle set is not a closed mesh`, decaderr.ErrUnsupported)
	}
	mesh := &topo.Mesh{
		Vertices:  append([]r3.Vec(nil), cp.Verts...),
		Triangles: append([][3]int(nil), cp.Tris...),
		Source:    src,
	}
	topobuild.SetVertexBounds(mesh, cp.VertexBound)
	mesh.Bound = cp.Delta
	if !verifyAll {
		return mesh, nil
	}
	slack, volSymDiff, err := MeshProofs(ctx, rec, MeshInput{
		Verts: cp.Verts, Tris: cp.Tris, Delta: cp.Delta, MaxRound: cp.MaxRound,
	}, walls)
	if err != nil {
		return nil, err
	}
	mesh.AreaSlack = slack
	mesh.VolSymDiff = volSymDiff
	mesh.SymDiffOK = true
	return mesh, nil
}
