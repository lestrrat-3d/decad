package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// solidFromMeshScans is the pair-scan budget of the crossing audit SolidFromMesh runs.
const solidFromMeshScans = 16_000_000

// SolidFromMesh turns a closed [MeshBody] into a solid [Body] of d, so an imported
// or built mesh takes part in Booleans, mass properties, selection and Verify as a
// body does (agent-docs/mesh-solid-design.md). The mesh is the solid: its vertices
// are held exactly, so the body's boundary bound is zero and its volume is the
// mesh's exact volume, read from the exact signed volume of its triangles. The body is
// Faceted: coplanar triangles that share an exact plane through their vertices, found
// in rational arithmetic, form one face, so a block read from STL has six faces.
//
// m MUST be closed (every edge in two triangles), wound outward, and free of
// self-crossing, which the exact contact audit of the lofted bodies proves: a mesh with
// a crossing the audit finds, or one the audit cannot decide, is refused. A mesh that is
// open, non-manifold or wound inward is ErrNotSolid, one with no positive volume is
// ErrNotSolid, one with a crossing is ErrDegenerate, and one of more than 65,528
// triangles, or whose audit exceeds its work ceiling, is ErrUnsupported. A nil
// context, document or mesh is ErrDegenerate. A failure leaves d unchanged.
func SolidFromMesh(ctx context.Context, d *Document, m *MeshBody) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationSolidFromMesh, retErr) }()
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a conversion`, ErrDegenerate)
	}
	if d == nil || m == nil {
		return nil, fmt.Errorf(`%w: a conversion needs a document and a mesh body`, ErrDegenerate)
	}
	report := m.Report()
	switch {
	case !report.Closed || report.NonManifoldEdges != 0:
		return nil, fmt.Errorf(`%w: a solid needs a closed manifold mesh`, ErrNotSolid)
	case report.Inverted:
		return nil, fmt.Errorf(`%w: the mesh is wound inward`, ErrNotSolid)
	}
	verts, tris := m.Vertices(), m.Triangles()
	if len(tris) > loftmesh.MaxLoftAuditTriangles {
		return nil, fmt.Errorf(`%w: a mesh of %d triangles is past the %d the crossing audit proves`,
			ErrUnsupported, len(tris), loftmesh.MaxLoftAuditTriangles)
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx), verts, tris,
		loftmesh.MaxLoftAuditTriangles, solidFromMeshScans); err != nil {
		if errors.Is(err, ErrUnsupported) || ctx.Err() != nil {
			return nil, err
		}
		return nil, fmt.Errorf("%w: the mesh crosses itself: %s", ErrDegenerate, err)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, verts, tris)
	if err != nil {
		return nil, err
	}
	volume, volumeBound, held, err := booleaneval.VolumeReading(ctx, audit, 0)
	if err != nil {
		return nil, err
	}
	if held.Sign() <= 0 {
		return nil, fmt.Errorf(`%w: the mesh encloses no positive volume`, ErrNotSolid)
	}
	lo, hi := verts[0], verts[0]
	for _, v := range verts[1:] {
		lo = r3.NewVec(math.Min(lo.X, v.X), math.Min(lo.Y, v.Y), math.Min(lo.Z, v.Z))
		hi = r3.NewVec(math.Max(hi.X, v.X), math.Max(hi.Y, v.Y), math.Max(hi.Z, v.Z))
	}
	diameter := proofbound.DvLenUpper(proofbound.HeldDelta(hi, lo))
	if proofbound.IsNonFinite(diameter) || !(diameter > 0) {
		return nil, fmt.Errorf(`%w: the mesh has no finite diameter`, ErrNotFinite)
	}
	ref := d.producerID(ctx)
	env := envOf(documentNode(d), ref)
	src, groups := meshPlaneGroups(env.Ref, verts, tris)
	node, err := facetedrecord.BuildWithProof(ctx, env, facetedrecord.Payload{
		Verts: verts, Tris: tris, Src: src, Groups: groups,
		VertexBound: make([]float64, len(verts)), DPair: diameter, ContactAudited: true, Xform: r3.Identity(),
	}, audit, measurement.Measurement{Value: units.CubicMillimeters(volume),
		Exactness: measurement.ExactnessOf(volumeBound), Bound: units.CubicMillimeters(volumeBound)}, held)
	if err != nil {
		return nil, err
	}
	body := bodyHandle(node)
	if err := d.commitOperationIn(ctx, nil, OperationSolidFromMesh, nil, body); err != nil {
		return nil, err
	}
	return body, nil
}

// meshPlaneGroups assigns each triangle a source group: triangles whose vertices lie on
// one plane share a group, the plane found exactly from the floats the vertices hold, as
// the normal (b−a)×(c−a) and offset n·a of each triangle divided by the first non-zero
// magnitude among its normal's parts, so a plane and its triangles' orientation have one key.
// A triangle of no area is a group of its own.
func meshPlaneGroups(producer provenance.Producer, verts []r3.Vec, tris [][3]int) ([]int, []facetedrecord.Group) {
	exact := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
	}
	sub := func(a, b [3]*big.Rat) [3]*big.Rat {
		var out [3]*big.Rat
		for i := range out {
			out[i] = new(big.Rat).Sub(a[i], b[i])
		}
		return out
	}
	cross := func(u, v [3]*big.Rat) [3]*big.Rat {
		term := func(a, b, c, d *big.Rat) *big.Rat {
			return new(big.Rat).Sub(new(big.Rat).Mul(a, b), new(big.Rat).Mul(c, d))
		}
		return [3]*big.Rat{term(u[1], v[2], u[2], v[1]), term(u[2], v[0], u[0], v[2]), term(u[0], v[1], u[1], v[0])}
	}
	index := map[string]int{}
	src := make([]int, len(tris))
	var groups []facetedrecord.Group
	for ti, t := range tris {
		a, b, c := exact(verts[t[0]]), exact(verts[t[1]]), exact(verts[t[2]])
		n := cross(sub(b, a), sub(c, a))
		key, planar := fmt.Sprintf("triangle %d", ti), false
		for _, part := range n {
			if part.Sign() == 0 {
				continue
			}
			scale := new(big.Rat).Abs(part)
			offset := new(big.Rat)
			for i := range n {
				offset.Add(offset, new(big.Rat).Mul(n[i], a[i]))
			}
			key = fmt.Sprintf("%s %s %s %s", new(big.Rat).Quo(n[0], scale).RatString(), new(big.Rat).Quo(n[1], scale).RatString(),
				new(big.Rat).Quo(n[2], scale).RatString(), new(big.Rat).Quo(offset, scale).RatString())
			planar = true
			break
		}
		gi, ok := index[key]
		if !ok {
			gi = len(groups)
			index[key] = gi
			groups = append(groups, facetedrecord.Group{
				Origins: []provenance.Ref{{Producer: producer, Role: fmt.Sprintf("mesh_face(%d)", gi)}},
				Planar:  planar,
			})
		}
		src[ti] = gi
	}
	return src, groups
}
