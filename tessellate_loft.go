package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/r3"
)

// This file is docs/tessellation-reach-design.md §4 (docs/tessellation-design.md
// §13's increment T6): the loftPayload EXACT RESTATEMENT. A loft body already
// holds the complete, globally oriented triangle set its construction built and
// its §6 crossing audit classified. A solid copies that set; a sheet copies its
// recorded wall range and omits the cap ranges. Both preserve the held wall
// triangles and their source faces without chording, retriangulation or motion.

// tessellateLoft restates a lofted body's held triangle set as a Mesh
// (docs/tessellation-design.md §2's "loftPayload exact restatement", §4's
// source-face table).
//
// It takes no chord tolerance, and that is the design's own reading rather than
// an omission (docs/tessellation-reach-design.md §4): this path adds no chording
// of its own, so there is no chording component for a tolerance to bind, and the
// whole published Bound is inherited payload displacement, which §1's Tolerance
// row lets ride above tol exactly as a prism's per-end axial displacement does.
// A prism reserves its section displacement from the requested tolerance because
// its own chords are still to be chosen against what is left; a loft has no such
// choice to make, so it neither reserves nor refuses.
//
// Orientation needs no work here either. docs/loft-design.md §5's whole-shell
// step already turned every triangle outward from the signed tetrahedron sum,
// and placed re-runs that step on the placed triangle set, so §4's "a reflected
// placement reverses every final triangle once" rule is discharged by the
// payload and repeating it would reverse a mirrored shell twice. The signed
// orientation audit below applies to the closed solid; the sheet takes its
// winding from that same whole-shell build before the caps are omitted.
//
// A solid runs closure and signed-volume orientation audits. An open sheet runs
// the manifold-with-boundary and vertex-link audits instead: the signed sum is
// anchor-dependent without caps and cannot decide orientation. A missing source
// role is ErrDegenerate because the live topology contradicts its payload.
func tessellateLoft(ctx context.Context, b *Body, lp loftPayload) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := requireLoftTriangleSplit(lp); err != nil {
		return nil, err
	}

	// Provenance roles are how the payload's cells name the faces
	// buildLoftTopology built from them (docs/evaluator-design.md §3): one
	// side(i,j,k) face per wall triangle — a loft coalesces no wall, so the two
	// halves of a cell keep their two distinct faces even when coplanar
	// (docs/tessellation-design.md §4). A sheet carries no cap role.
	byRole := map[string]*Face{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			byRole[o.Role] = f
		}
	}
	faceOfRole := func(role string) (*Face, error) {
		f, ok := byRole[role]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, role)
		}
		return f, nil
	}
	sheet := b.Kind() == BodySheet
	var capStart, capEnd *Face
	if !sheet {
		var err error
		capStart, err = faceOfRole(roleCapStart)
		if err != nil {
			return nil, err
		}
		capEnd, err = faceOfRole(roleCapEnd)
		if err != nil {
			return nil, err
		}
	}

	budget := newWorkBudget(ctx)
	// The payload records walls first, then capStart, then capEnd. Dropping
	// exactly the latter two ranges uses that provenance rather than testing
	// a triangle's coordinates against a section plane.
	triangles := lp.tris
	if sheet {
		triangles = lp.tris[:lp.walls]
	}
	src := make([]*Face, len(triangles))
	for k := range lp.walls {
		if err := budget.step(); err != nil {
			return nil, err
		}
		f, err := faceOfRole(fmt.Sprintf("side(%d,%d,%d)", lp.cell[k][0], lp.cell[k][1], lp.side[k]))
		if err != nil {
			return nil, err
		}
		src[k] = f
	}
	if !sheet {
		for k := lp.walls; k < lp.walls+lp.capStartCount; k++ {
			src[k] = capStart
		}
		for k := lp.walls + lp.capStartCount; k < len(lp.tris); k++ {
			src[k] = capEnd
		}
	}
	if err := budget.err(); err != nil {
		return nil, err
	}

	// Fresh slices: Mesh's accessors copy on the way out, but the held mesh
	// must not alias the payload's own arrays, or a future consumer writing
	// through one would rewrite the body's boundary.
	mesh := &Mesh{
		vertices:  append([]r3.Vec(nil), lp.verts...),
		triangles: append([][3]int(nil), triangles...),
		source:    src,
	}
	if err := publishLoftMeshProof(mesh, lp.proof, !sheet); err != nil {
		return nil, err
	}
	if sheet {
		// The payload's areaSlack includes nonnegative cap allowances. They
		// conservatively cover the wall-only mesh without subtracting an
		// unproven cap contribution. A sheet publishes no volume proof.
		if err := requireSheetMesh(ctx, b, mesh); err != nil {
			return nil, err
		}
		if err := requireSheetVertexLinks(ctx, mesh); err != nil {
			return nil, err
		}
		return mesh, nil
	}
	if err := requireClosedMesh(mesh); err != nil {
		return nil, fmt.Errorf(`%w: the loft payload's held triangle set is not a closed mesh, so it restates no boundary`, ErrUnsupported)
	}
	// docs/tessellation-design.md §4's signed-volume audit, over the same
	// identity docs/loft-design.md §5's whole-shell orientation rule reads and
	// at the same anchor evalLoft used. The shell is not a void, so its sum
	// must come out positive.
	anchor := lp.xform.Apply(lp.plane0.Origin)
	if !finiteVec(anchor) {
		return nil, fmt.Errorf(`%w: the loft payload states no finite anchor to audit its own orientation against`, ErrUnsupported)
	}
	if meshOrientationSign(mesh.vertices, mesh.triangles, anchor) <= 0 {
		return nil, fmt.Errorf(`%w: the loft payload's held triangle set does not enclose a positive volume, so it restates no solid`, ErrUnsupported)
	}
	return mesh, nil
}

// requireLoftTriangleSplit checks the payload's own wall/cap split before a
// single source face is read: the three counts index the triangle set and the
// two parallel arrays name every wall triangle's cell, so a payload disagreeing
// with itself must refuse rather than index out of its own arrays. Nothing a
// build produces reaches it — evalLoft copies all five fields from one assembly
// — so it stands for a payload no evaluator wrote.
func requireLoftTriangleSplit(lp loftPayload) error {
	if len(lp.tris) == 0 || len(lp.verts) == 0 {
		return fmt.Errorf(`%w: the loft payload holds no triangle set to restate`, ErrDegenerate)
	}
	if lp.walls < 0 || lp.capStartCount < 0 || lp.walls+lp.capStartCount > len(lp.tris) {
		return fmt.Errorf(`%w: the loft payload's wall and cap triangle counts do not partition its own triangle set`, ErrDegenerate)
	}
	if len(lp.cell) != lp.walls || len(lp.side) != lp.walls {
		return fmt.Errorf(`%w: the loft payload names a cell for %d of its %d wall triangles`, ErrDegenerate, len(lp.cell), lp.walls)
	}
	return nil
}

// publishLoftMeshProof writes the payload's boundary and area proofs onto the
// restated mesh. The solid also publishes its occupied-volume proof. A sheet's
// areaSlack may include cap allowances, all nonnegative, so it remains a bound
// on the retained wall triangles without a new subtraction.
//
// sourceBound is that facet departure for EVERY face, and Bound with it, since
// each face's facets are exactly the payload's triangles for it. A zero is
// published only where the payload proved both facet-departure terms zero by
// value (docs/loft-design.md §5.2), which is what admits such a loft to the
// boolean's all-planar zero-bound path; every other loft is an ordinary
// positive-bound operand.
//
// A non-finite published term refuses (docs/tessellation-design.md §12): an
// absent proof must never reach a consumer as a bound. The sheet never
// publishes volSymDiff, so its value does not gate sheet export.
func publishLoftMeshProof(m *Mesh, p loftMeshProof, solid bool) error {
	if isNonFinite(p.facetDeparture) || isNonFinite(p.areaSlack) || (solid && isNonFinite(p.volSymDiff)) {
		return fmt.Errorf(`%w: the loft payload states no finite proof for a required mesh bound`, ErrUnsupported)
	}
	for _, f := range m.source {
		m.setFaceBound(f, p.facetDeparture)
	}
	m.bound = p.facetDeparture
	m.areaSlack = p.areaSlack
	if solid {
		m.volSymDiff = p.volSymDiff
		m.symDiffOK = true
	}
	return nil
}
