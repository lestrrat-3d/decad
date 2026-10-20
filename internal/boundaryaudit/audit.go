package boundaryaudit

import (
	"context"

	"github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// AuditSolidBoundary checks the structural invariants of a solid's held
// boundary: every edge bounds exactly two faces (manifold and watertight at
// once, for a closed skin), and every face carries at least one loop with at
// least one edge. auditedFacetedImage reports whether b's payload is a faceted
// image whose embedded closed mesh passed the build's contact audit and holds
// at least one triangle; the caller reads it because that payload's package
// imports this one.
func AuditSolidBoundary(b *topo.Body, auditedFacetedImage bool) bool {
	faces := topobuild.Faces(b)
	if len(faces) == 0 {
		return false
	}
	for _, f := range faces {
		if len(f.Loops) == 0 {
			// A closed surface of revolution bounds a solid with no edges
			// at all. A faceted one-face image may have the same topology
			// only after its embedded closed mesh passed the build audit.
			switch f.Surface.Kind() {
			case surfacegeom.KindSphere, surfacegeom.KindTorus:
				continue
			case surfacegeom.KindFaceted:
				if auditedFacetedImage && len(faces) == 1 && len(topobuild.Edges(b)) == 0 {
					continue
				}
				return false
			default:
				return false
			}
		}
		for _, l := range f.Loops {
			if len(l.CoEdges) == 0 {
				return false
			}
		}
	}
	for _, e := range topobuild.Edges(b) {
		if len(e.Faces) != 2 {
			return false
		}
	}
	return true
}

// AuditSheetBoundary decides Table V's sheet validity audit
// (docs/surface-design.md §9.1): three structural legs read off the recorded
// topology, plus non-self-intersection admitted BY CONSTRUCTION alone. It is
// deliberately NOT §6.4's closure audit (`loftmesh.LoftCrossingAudit`) with closure
// dropped: that audit runs over a triangulated, all-planar face set sharing
// one exact vertex table, which a surface-extruded wall's `Plane`,
// `Cylinder` or `NURBSSurface` geometry — rimmed by `Arc3`, `Circle3` or
// `NURBSCurve` segments — cannot supply without first being chorded. Admitting
// a sheet because its chord mesh audits clean would be an admission gate
// resting on an approximation, which CLAUDE.md's reject-only rule forbids
// outright.
//
// Leg 1 — every face has at least one loop, and every loop has at least one
// coedge. Unlike AuditSolidBoundary's closed-solid audit, there is NO
// sphere/torus exemption: that allowance is a fact about a closed surface of
// revolution bounding a solid, and does not carry to a sheet, whose free edges
// are already the expected shape (docs/surface-design.md §2.2).
//
// Leg 2 — every edge is adjacent to one or two faces. Three or more is
// non-manifold, a defect on either body kind.
//
// Leg 3 — every edge adjacent to two faces is traversed by exactly one
// forward and one backward coedge, counted over every face's every loop's
// every coedge use. A coedge-use count that disagrees with the edge's own
// adjacent-face count is also a violation: it means some face's loop walks an
// edge the edge's face list does not know about, or fails to walk one it does.
//
// Leg 4 — non-self-intersection, decided by topo.ProvesSimple on the
// body's own payload. See topo.SimpleProver and each payload's ProvesSimple
// method for the admission rule, one payload at a time.
func AuditSheetBoundary(ctx context.Context, b *topo.Body) SheetAuditOutcome {
	faces := topobuild.Faces(b)
	if len(faces) == 0 {
		return SheetAuditViolated
	}
	for _, f := range faces {
		if len(f.Loops) == 0 {
			return SheetAuditViolated
		}
		for _, l := range f.Loops {
			if len(l.CoEdges) == 0 {
				return SheetAuditViolated
			}
		}
	}

	edges := topobuild.Edges(b)
	for _, e := range edges {
		if len(e.Faces) == 0 || len(e.Faces) > 2 {
			return SheetAuditViolated
		}
	}

	uses := map[*topo.Edge]int{}
	forward := map[*topo.Edge]int{}
	backward := map[*topo.Edge]int{}
	for _, f := range faces {
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				uses[ce.Edge]++
				if ce.Forward {
					forward[ce.Edge]++
				} else {
					backward[ce.Edge]++
				}
			}
		}
	}
	for _, e := range edges {
		if uses[e] != len(e.Faces) {
			return SheetAuditViolated
		}
		if len(e.Faces) == 2 && (forward[e] != 1 || backward[e] != 1) {
			return SheetAuditViolated
		}
	}

	if topo.ProvesSimple(ctx, b.Payload) {
		return SheetAuditProven
	}
	return SheetAuditUndecided
}

// UndecidedPairReason picks the code and message for a pair whose overlap
// volume the evaluator could not measure (verification §1.1): payload,
// contact, and in-pipeline limits each keep their own code and action; only
// an overlap with an otherwise undecided measurement is
// DiagUndecidedInterference; an unresolved partition is DiagUndecidedPair.
// Verify emits only this cause-specific diagnostic. A pair holding a sheet
// operand never reaches this function: it is resolved earlier, by box
// separation or the sheet decision procedure (pairkernel.SheetSolid,
// internal/clearance/pairkernel/kernel.go), and takes
// DiagUnsupportedPairSheet or DiagSheetSolidCrossing instead
// (docs/surface-design.md §9.3).
func UndecidedPairReason(a, b *topo.Body, verdict PairVerdict, outcome InterferenceOutcome) (report.DiagnosticCode, string) {
	switch {
	case outcome == InterferenceUnsupportedPayloadFirst:
		return report.DiagUnsupportedPairPayload,
			"the first operand cannot enter the read-only intersection at the chord tolerance this check derives from the pair's own size, which no option sets: its tessellation refused that tolerance, or its held facets where the pair meets are coarser than it; simplify that operand or reduce the pair's extent, or wait for wider tessellation reach"
	case outcome == InterferenceUnsupportedPayloadSecond:
		return report.DiagUnsupportedPairPayload,
			"the second operand cannot enter the read-only intersection at the chord tolerance this check derives from the pair's own size, which no option sets: its tessellation refused that tolerance, or its held facets where the pair meets are coarser than it; simplify that operand or reduce the pair's extent, or wait for wider tessellation reach"
	case outcome == InterferenceUnsupportedVolumeProofFirst:
		return report.DiagUnsupportedPairPayload,
			"the first operand tessellates, but its mesh carries no proof of the volume it and the body it stands for differ by, so no read-only intersection may compose it; keep this body out of overlapping pairs, or wait for its occupied-volume proof"
	case outcome == InterferenceUnsupportedVolumeProofSecond:
		return report.DiagUnsupportedPairPayload,
			"the second operand tessellates, but its mesh carries no proof of the volume it and the body it stands for differ by, so no read-only intersection may compose it; keep this body out of overlapping pairs, or wait for its occupied-volume proof"
	case outcome == InterferenceUnsupportedContact:
		if sharesFacePlane(a, b) {
			return report.DiagUnsupportedPairContact,
				"the pair reaches a contact or near-contact that the read-only intersection cannot classify, and the operands share a face plane; offset the operands so no face plane is shared, or adjust the geometry to create clear separation or deeper overlap, or wait for contact support"
		}
		return report.DiagUnsupportedPairContact,
			"the pair reaches a contact or near-contact that the read-only intersection cannot classify; adjust the geometry to create clear separation or deeper overlap, or wait for contact support"
	case outcome == InterferenceUnsupportedPipeline:
		return report.DiagUnsupportedPairPipeline,
			"both operands tessellate, but later read-only intersection geometry exceeds the boolean pipeline's reach; simplify the boolean geometry or wait for pipeline support"
	case outcome == InterferenceUndecided && verdict == PairOverlapping:
		return report.DiagUndecidedInterference, "the pair is proven to overlap but the overlap volume is unmeasured"
	default:
		return report.DiagUndecidedPair, "the disjoint/overlap partition proof resolved neither way"
	}
}

// sharesFacePlane reports whether a and b have a planar face whose infinite
// plane coincides — exact float comparison on the canonicalized normal and
// offset, reject-only in spirit: it can only fail to name a shared plane it
// cannot exactly confirm, never invent one that is not there. It is a
// diagnostic aid, not a proof: it changes only whether the contact message
// names the cause, never the code.
func sharesFacePlane(a, b *topo.Body) bool {
	keys := make(map[surfacegeom.DiagnosticPlaneKey]struct{})
	for _, f := range topobuild.Faces(a) {
		if p, ok := f.Surface.(surfacegeom.Plane); ok {
			keys[surfacegeom.CanonicalDiagnosticPlaneKey(p)] = struct{}{}
		}
	}
	for _, f := range topobuild.Faces(b) {
		p, ok := f.Surface.(surfacegeom.Plane)
		if !ok {
			continue
		}
		if _, found := keys[surfacegeom.CanonicalDiagnosticPlaneKey(p)]; found {
			return true
		}
	}
	return false
}
