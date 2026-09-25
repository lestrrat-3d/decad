package decad

import (
	"fmt"

	"github.com/lestrrat-go/option/v3"
)

// This file owns how much proof a tessellation runs and what the mesh it
// returns says about its own: docs/tessellation-design.md §1's three
// verification levels, the option that chooses one, and the two readings a
// [Mesh] publishes about the proofs it carries. tessellate.go applies the
// choice; the payload tessellators read it to decide which proof to compute.

// Verification is how much of docs/tessellation-design.md §1's proof
// [Body.Tessellate] runs. It is an ORDERED level, because choosing how much
// work to do is a total order: each level adds to the one below it.
//
// What a mesh then PUBLISHES is two independent readings rather than a level
// ([Mesh.BoundaryVerified], [Mesh.VolumeVerified]), because the proofs are not
// a chain: a stitched body's occupied-volume proof rests on its vertices being
// exact and its lumps proven separate rather than on more work, so a mesh can
// carry one proof without the other, and a later proof may be independent of
// the facet-contact audit rather than stronger than it.
type Verification int

const (
	// VerifyNone asks for a mesh built to be drawn or exported rather than to
	// support a proof. The facet-contact audit, the area-slack proof and the
	// occupied-volume proof do not run, and the mesh states none of them:
	// [Mesh.BoundaryVerified] and [Mesh.VolumeVerified] are both false, and no
	// boolean composes such a mesh.
	//
	// Everything else holds unchanged. The mesh is chorded the same way, it
	// publishes the same proven [Mesh.Bound] and the same per-face
	// displacement behind it, it attributes the same source faces, it runs the
	// closed-mesh or manifold-with-boundary audit, the vertex-link audit, the
	// per-facet positive-area check and the signed-volume orientation audit,
	// and it is byte-for-byte the mesh a higher level builds — same vertices,
	// same indices, same order.
	VerifyNone Verification = iota
	// VerifyBoundary adds the facet-contact audit
	// (docs/tessellation-design.md §9): no non-adjacent facet pair touches,
	// and an adjacent pair meets only along the vertex or edge its indices
	// share. That is what proves the facet set EMBEDDED, and so what makes
	// "outside the body" a geometric statement rather than the construction's
	// own convention ([Mesh.Triangles]). A mesh built here publishes
	// [Mesh.BoundaryVerified] true.
	VerifyBoundary
	// VerifyAll adds every proof the body's payload supports — the area slack
	// and, where the payload proves one, the occupied-volume bound that admits
	// the mesh to [Union], [Cut] and [Intersect]. It is the default, and it is
	// the only level any boolean accepts. A payload whose own occupied-volume
	// proof has not landed still publishes [Mesh.VolumeVerified] false here.
	VerifyAll
)

// TessellateOption configures [Body.Tessellate]. The export package accepts
// the same options for its STL and OBJ writers.
type TessellateOption interface {
	option.Interface
	tessellateOption()
}

type tessellateOption struct{ option.Interface }

func (tessellateOption) tessellateOption() {}

type identVerification struct{}

// WithVerification sets how much of the mesh's proof the call runs.
//
// [Body.Tessellate] defaults to [VerifyAll]. The export package's STL and OBJ
// writers default to [VerifyNone] because they consume vertices and indices
// without reading proof terms. Pass WithVerification([VerifyAll]) to demand
// stronger proofs and accept their refusals.
//
// A value naming none of the three levels is [ErrUnsupported].
func WithVerification(v Verification) TessellateOption {
	return tessellateOption{option.New(identVerification{}, v)}
}

// BoundaryVerified reports whether every boundary audit this body's payload
// supports ran and passed — including, on a payload that carries one, the
// facet-contact audit that proves the facet set embedded and free of
// non-adjacent contact (docs/tessellation-design.md §1's Embedding row).
//
// It is false exactly when [VerifyNone] was asked of a payload whose boundary
// proof includes that audit. Every other boundary audit — directed-edge
// closure or the manifold-with-boundary audit, vertex links, per-facet
// positive area, signed-volume orientation — runs at every level, so no other
// payload's reading moves with the level.
func (m *Mesh) BoundaryVerified() bool { return m.boundaryOK }

// VolumeVerified reports whether this mesh carries the proof of the volume it
// and the body it stands for differ by (docs/tessellation-design.md §2's
// volSymDiff). It is what admits the mesh to [Union], [Cut] and [Intersect].
//
// It is false below [VerifyAll], where that proof never ran, and false at
// [VerifyAll] too for a body whose payload class publishes no occupied-volume
// proof at all — a cap-loop chamfer, a sheet, or a stitched body whose
// vertices are not proven exact. A false reading is the proof's ABSENCE, never
// a bound of zero: a zero would be the claim that the mesh occupies exactly
// the denoted volume.
func (m *Mesh) VolumeVerified() bool { return m.symDiffOK }

// withholdProofs drops the two proofs a level below [VerifyAll] never ran
// (docs/tessellation-design.md §2). Dropping them is mandatory rather than
// tidy: a path that skipped some of their terms would otherwise publish a
// partly composed figure, and areaSlack carries no flag of its own to say so.
// The mesh's own Bound and per-face displacements are untouched — they rest on
// no facet-pair audit and no volume term (§10.1), and a zero Bound would be a
// positive claim of exactness rather than an absent proof.
func (m *Mesh) withholdProofs() {
	m.areaSlack = 0
	m.volSymDiff = 0
	m.symDiffOK = false
}

// payloadAuditsFacetContact reports whether this payload's own tessellation
// runs the facet-contact audit that proves its facet set embedded
// (docs/tessellation-design.md §9). It names the same payload classes
// revolveContactAudit is reached from, including a curved stitched body
// backed by one revolve sheet. A payload class that gains such an
// audit MUST be added here in the same change — a class missing from this list
// publishes BoundaryVerified() true at every level, which is the whole of what
// it claims today and would be an over-claim once it had an audit to decline.
func payloadAuditsFacetContact(p featurePayload) bool {
	_, ok := p.(revolvePayload)
	if ok {
		return true
	}
	sp, ok := p.(stitchPayload)
	return ok && sp.tris == nil
}

// foldVerification reads the level a call's options name, starting from the
// caller's own default. The last [WithVerification] wins, matching how every
// other option in this package folds.
func foldVerification(opts []option.Interface, fallback Verification) (Verification, error) {
	chosen := fallback
	for _, o := range opts {
		if o == nil {
			return 0, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		if _, ok := o.Ident().(identVerification); !ok {
			continue
		}
		v, ok := option.Get[Verification](o)
		if !ok {
			return 0, fmt.Errorf(`%w: WithVerification carries no verification level`, ErrDegenerate)
		}
		chosen = v
	}
	if chosen < VerifyNone || chosen > VerifyAll {
		return 0, fmt.Errorf(`%w: verification level %d names none of the levels this tessellation knows`, ErrUnsupported, int(chosen))
	}
	return chosen, nil
}
