package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/r3"
)

// This file is docs/tessellation-reach-design.md §4 (docs/tessellation-design.md
// §13's increment T6): the loftPayload EXACT RESTATEMENT. A loft body already
// holds the complete, globally oriented triangle set its construction built and
// its §6 crossing audit classified, so the tessellation copies that set and its
// source faces and publishes the proof record the payload composed for the same
// triangles. It chords nothing, retriangulates nothing, welds nothing and moves
// no coordinate.

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
// payload and repeating it would reverse a mirrored shell twice. The audit below
// is what holds that claim to account rather than assuming it.
//
// The two audits are the payload's own invariants restated over the copied set,
// so failing either can only mean a payload that never passed §6's audit reached
// this path. Both refuse with ErrUnsupported (docs/tessellation-design.md §12)
// and return no partial mesh. A missing source role is ErrDegenerate instead —
// §4's rule — because there the body's live topology contradicts its own payload.
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
	// (docs/tessellation-design.md §4) — plus the two caps.
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
	capStart, err := faceOfRole(roleCapStart)
	if err != nil {
		return nil, err
	}
	capEnd, err := faceOfRole(roleCapEnd)
	if err != nil {
		return nil, err
	}

	budget := newWorkBudget(ctx)
	src := make([]*Face, len(lp.tris))
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
	for k := lp.walls; k < lp.walls+lp.capStartCount; k++ {
		src[k] = capStart
	}
	for k := lp.walls + lp.capStartCount; k < len(lp.tris); k++ {
		src[k] = capEnd
	}
	if err := budget.err(); err != nil {
		return nil, err
	}

	// Fresh slices: Mesh's accessors copy on the way out, but the held mesh
	// must not alias the payload's own arrays, or a future consumer writing
	// through one would rewrite the body's boundary.
	mesh := &Mesh{
		vertices:  append([]r3.Vec(nil), lp.verts...),
		triangles: append([][3]int(nil), lp.tris...),
		source:    src,
	}
	if err := publishLoftMeshProof(mesh, lp.proof); err != nil {
		return nil, err
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
	if loftOrientationSign(mesh.vertices, mesh.triangles, anchor) <= 0 {
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

// publishLoftMeshProof writes docs/tessellation-design.md §2's three private
// proofs onto the restated mesh, unchanged from the payload's own composition:
// the restatement introduces no displacement of its own, so every term the mesh
// states is the term the payload already carries for the same triangles.
//
// sourceBound is that facet departure for EVERY face, and Bound with it, since
// each face's facets are exactly the payload's triangles for it. A zero is
// published only where the payload proved both facet-departure terms zero by
// value (docs/loft-design.md §5.2), which is what admits such a loft to the
// boolean's all-planar zero-bound path; every other loft is an ordinary
// positive-bound operand.
//
// A non-finite term refuses (docs/tessellation-design.md §12) before any of the
// three is published: an absent proof must never reach a consumer as a bound,
// and volSymDiff in particular would otherwise be composed into a boolean
// result's own volume error.
func publishLoftMeshProof(m *Mesh, p loftMeshProof) error {
	if isNonFinite(p.facetDeparture) || isNonFinite(p.areaSlack) || isNonFinite(p.volSymDiff) {
		return fmt.Errorf(`%w: the loft payload states no finite proof of how far its held facets, their area, or the volume they enclose sit from the boundary they stand for`, ErrUnsupported)
	}
	for _, f := range m.source {
		m.setFaceBound(f, p.facetDeparture)
	}
	m.bound = p.facetDeparture
	m.areaSlack = p.areaSlack
	m.volSymDiff = p.volSymDiff
	m.symDiffOK = true
	return nil
}
