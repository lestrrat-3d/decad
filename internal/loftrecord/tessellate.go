package loftrecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// Tessellate restates body's held triangle set, whose payload is lp, as a mesh
// node (docs/tessellation-design.md §2's "loftPayload exact restatement", §4's
// source-face table; docs/tessellation-reach-design.md §4).
// internal/loftmesh.RestateLoft copies the set, names each triangle's
// face by its provenance role, publishes the payload's own proof record and
// audits the result; this adapter numbers the body's faces for it and maps the
// numbers back to the face nodes. privateRoles, when given, replaces the
// roles the faces' origins carry: a twisted sweep's held loft names its
// triangles by private roles mapped onto the merged faces.
//
// It takes no chord tolerance, and that is the design's own reading rather
// than an omission: the path adds no chording of its own, so the whole
// published Bound is inherited payload displacement, which §1's Tolerance row
// lets ride above tol exactly as a prism's per-end axial displacement does.
// Orientation needs no work either: docs/loft-design.md §5's whole-shell step
// already turned every triangle outward, and placed re-runs that step, so
// repeating it would reverse a mirrored shell twice. A solid runs closure and
// signed-volume audits; a sheet runs the manifold-with-boundary and
// vertex-link audits instead, because the signed sum is anchor-dependent
// without caps. A missing source role is ErrDegenerate because the live
// topology contradicts its payload.
func Tessellate(ctx context.Context, body *topo.Body, lp Payload, privateRoles ...map[string]*topo.Face) (*topo.Mesh, error) {
	faces := topobuild.Faces(body)
	number := make(map[*topo.Face]int, len(faces))
	faceOfRole := map[string]int{}
	for i, f := range faces {
		number[f] = i
		for _, o := range f.Origins {
			faceOfRole[o.Role] = i
		}
	}
	if len(privateRoles) != 0 {
		faceOfRole = make(map[string]int, len(privateRoles[0]))
		for role, face := range privateRoles[0] {
			i, ok := number[face]
			if !ok {
				return nil, fmt.Errorf(`%w: a held loft source has no live face`, decaderr.ErrDegenerate)
			}
			faceOfRole[role] = i
		}
	}
	sheet := body.Kind == topo.BodySheet
	in := loftmesh.LoftInput{
		Vertices: lp.Verts, Triangles: lp.Tris,
		WallCount: lp.Walls, StartCapCount: lp.CapStartCount,
		WallCell: lp.Cell, WallSide: lp.Side,
		Sheet:      sheet,
		FaceOfRole: faceOfRole, StartCapRole: provenance.RoleCapStart, EndCapRole: provenance.RoleCapEnd,
		FacetDepartureMM: lp.Proof.FacetDeparture,
		AreaSlackMM2:     lp.Proof.AreaSlack,
		VolumeSymDiffMM3: lp.Proof.VolSymDiff,
	}
	if sheet {
		counts := topobuild.FreeChainCountsByFace(body)
		in.FreeChainCounts = make(map[int]int, len(counts))
		for f, n := range counts {
			in.FreeChainCounts[number[f]] = n
		}
	} else {
		// The same anchor Build used (docs/loft-design.md §5's whole-shell
		// orientation rule).
		in.Anchor = lp.Xform.Apply(lp.Plane0.Origin)
	}
	out, err := loftmesh.RestateLoft(ctx, in)
	if err != nil {
		return nil, topobuild.LiftAuditError(err)
	}
	src := make([]*topo.Face, len(out.SourceFaces))
	for k, i := range out.SourceFaces {
		src[k] = faces[i]
	}
	mesh := &topo.Mesh{Vertices: out.Vertices, Triangles: out.Triangles, Source: src}
	for _, f := range src {
		topobuild.SetFaceBound(mesh, f, out.BoundMM)
	}
	mesh.Bound = out.BoundMM
	mesh.AreaSlack = out.AreaSlackMM2
	if out.VolumeProof {
		mesh.VolSymDiff = out.VolumeSymDiffMM3
		mesh.SymDiffOK = true
	}
	return mesh, nil
}
